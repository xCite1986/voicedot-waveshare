/*
  VoiceDot for Waveshare ESP32-S3-AUDIO-Board
  Firmware v0.4.0-test

  Target:
    Waveshare ESP32-S3-AUDIO-Board
    ESP32-S3R8, 16MB Flash, 8MB PSRAM
    ES8311 audio codec
    ES7210 microphone ADC
    TCA9555 I/O expander
    7x WS2812 RGB LEDs

  IMPORTANT:
    This version deliberately focuses on a reliable board bring-up:
      - WiFi provisioning + captive portal
      - Web UI
      - NVS config
      - OTA
      - Home Assistant REST connectivity test
      - automatic hardware revision / I2C bus detection
      - ES8311 / ES7210 / TCA9555 detection
      - RGB status ring
      - amplifier enable via TCA9555 when supported
      - hardware diagnostics
      - onboard buttons for volume, mute, and LED brightness
      - ES8311/ES7210 codec init
      - I2S microphone level meter
      - speaker test tone
      - K2 short press wake / record / send to Home Assistant Assist
      - K2 long press mute

    Home Assistant Assist audio streaming is the next layer after this test build.

  Arduino IDE dependency:
    - Adafruit NeoPixel

  Board:
    ESP32S3 Dev Module
    Flash Size: 16MB
    PSRAM: OPI PSRAM
    USB CDC On Boot: Enabled
    Partition scheme: any 16MB scheme with OTA partitions
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <ctype.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Firmware
// -----------------------------------------------------------------------------

static const char* FW_VERSION = "0.4.0-test";
static const char* HOSTNAME = "voicedot";
static const char* AP_PASSWORD = "voicedot";

// -----------------------------------------------------------------------------
// Known Waveshare board profiles
//
// Profile A is the pin map reported by current community examples.
// Profile B follows an alternate Waveshare schematic mapping.
//
// We detect the profile by probing the audio devices over I2C.
// -----------------------------------------------------------------------------

struct BoardProfile {
  const char* name;
  int i2cSda;
  int i2cScl;

  int i2sMclk;
  int i2sBclk;
  int i2sLrclk;
  int micData;
  int speakerData;

  int ledPin;
};

static const BoardProfile PROFILE_A = {
  "Waveshare current",
  11, 10,
  12, 13, 14, 15, 16,
  38
};

static const BoardProfile PROFILE_B = {
  "Waveshare schematic",
  18, 7,
  2, 6, 46, 8, 9,
  38
};

BoardProfile activeProfile = PROFILE_A;
bool profileDetected = false;

// -----------------------------------------------------------------------------
// I2C device addresses
// -----------------------------------------------------------------------------

static constexpr uint8_t ADDR_ES8311  = 0x18;
static constexpr uint8_t ADDR_ES7210  = 0x40;
static constexpr uint8_t ADDR_TCA9555 = 0x20;
static constexpr uint8_t ADDR_RTC     = 0x51;

static constexpr i2s_port_t AUDIO_I2S_PORT = I2S_NUM_0;
static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;
static constexpr uint32_t AUDIO_MCLK_HZ = 12288000;
static constexpr uint16_t AUDIO_FRAME_SAMPLES = 128;
static constexpr uint16_t K2_LONG_PRESS_MS = 850;
static constexpr uint32_t WAKE_RECORD_MS = 3500;
static constexpr uint32_t WAKE_MAX_BYTES = (AUDIO_SAMPLE_RATE * 2 * WAKE_RECORD_MS) / 1000;

// Onboard buttons:
// BOOT is connected directly to GPIO0. K1/K2/K3 are active-low on TCA9555
// port 1 bits 1..3, corresponding to EXIO09..EXIO11.
static constexpr uint8_t PIN_BOOT_BUTTON = 0;
static constexpr uint8_t TCA_KEY1_BIT = 1;  // EXIO09
static constexpr uint8_t TCA_KEY2_BIT = 2;  // EXIO10
static constexpr uint8_t TCA_KEY3_BIT = 3;  // EXIO11

static constexpr uint16_t BUTTON_DEBOUNCE_MS = 35;
static constexpr uint8_t VOLUME_STEP = 5;
static constexpr uint8_t LED_BRIGHTNESS_STEP = 10;
static constexpr uint8_t LED_BRIGHTNESS_MAX = 100;

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

WebServer server(80);
DNSServer dnsServer;
Preferences prefs;

bool apMode = false;
bool restartRequested = false;
unsigned long restartAt = 0;

bool es8311Present = false;
bool es7210Present = false;
bool tca9555Present = false;
bool rtcPresent = false;
bool amplifierEnabled = false;
bool muted = false;
uint8_t volumeBeforeMute = 60;

bool audioI2sReady = false;
bool codecPlaybackReady = false;
bool codecRecordReady = false;
bool speakerTestActive = false;
uint8_t micLevel = 0;
int16_t micPeak = 0;
float micDb = -96.0f;
uint32_t lastMicPoll = 0;
uint32_t micReadCount = 0;
uint32_t micReadErrors = 0;
uint32_t micSilentFrames = 0;
uint32_t micLastBytes = 0;

bool wakeBusy = false;
bool wakeRecording = false;
bool wakeSending = false;
uint32_t wakeLastDurationMs = 0;
uint32_t wakeLastBytes = 0;
uint32_t wakeLastHttpCode = 0;
String wakeLastState = "idle";
String wakeLastMessage = "Bereit.";
String wakeTranscript = "";
String wakeTtsUrl = "";

bool k2RawPressed = false;
bool k2StablePressed = false;
bool k2LongHandled = false;
uint32_t k2ChangedAt = 0;
uint32_t k2PressedAt = 0;

Adafruit_NeoPixel* pixels = nullptr;
uint8_t ledBrightness = 32;

struct ButtonState {
  bool rawPressed;
  bool stablePressed;
  bool lastStablePressed;
  uint32_t changedAt;
};

ButtonState btnVolumeUp = {false, false, false, 0};
ButtonState btnVolumeDown = {false, false, false, 0};
ButtonState btnMute = {false, false, false, 0};
ButtonState btnLedBrightness = {false, false, false, 0};

struct Config {
  String wifiSsid;
  String wifiPass;
  String deviceName;
  String haUrl;
  String haToken;
  uint8_t volume;
  uint8_t ledBrightness;
};

Config cfg;

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------

String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"':  out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", (uint8_t)c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

String normalizedHaUrl(String url) {
  url.trim();
  while (url.endsWith("/")) url.remove(url.length() - 1);

  if (url.length() > 0 &&
      !url.startsWith("http://") &&
      !url.startsWith("https://")) {
    url = "http://" + url;
  }
  return url;
}

bool parseHaUrl(const String &url, String &host, uint16_t &port, bool &secure) {
  String u = normalizedHaUrl(url);
  secure = u.startsWith("https://");
  int start = secure ? 8 : 7;

  if (!u.startsWith("http://") && !u.startsWith("https://")) return false;

  int slash = u.indexOf('/', start);
  String authority = slash >= 0 ? u.substring(start, slash) : u.substring(start);
  int colon = authority.lastIndexOf(':');

  if (colon > 0) {
    host = authority.substring(0, colon);
    port = authority.substring(colon + 1).toInt();
  } else {
    host = authority;
    port = secure ? 443 : 8123;
  }

  return host.length() > 0 && port > 0;
}

String jsonFindString(const String &json, const String &key) {
  int k = json.indexOf("\"" + key + "\"");
  if (k < 0) return "";

  int colon = json.indexOf(':', k);
  int quote = json.indexOf('"', colon + 1);
  if (colon < 0 || quote < 0) return "";

  String out;
  bool esc = false;
  for (int i = quote + 1; i < (int)json.length(); i++) {
    char c = json[i];
    if (esc) {
      out += c;
      esc = false;
    } else if (c == '\\') {
      esc = true;
    } else if (c == '"') {
      break;
    } else {
      out += c;
    }
  }

  return out;
}

int jsonFindInt(const String &json, const String &key, int fallback = -1) {
  int k = json.indexOf("\"" + key + "\"");
  if (k < 0) return fallback;

  int colon = json.indexOf(':', k);
  if (colon < 0) return fallback;

  int i = colon + 1;
  while (i < (int)json.length() && isspace((unsigned char)json[i])) i++;
  return json.substring(i).toInt();
}

String makeApSsid() {
  uint64_t chip = ESP.getEfuseMac();
  uint32_t suffix = (uint32_t)(chip & 0xFFFFFF);

  char buf[32];
  snprintf(buf, sizeof(buf), "VoiceDot-%06lX", (unsigned long)suffix);
  return String(buf);
}

void scheduleRestart(uint32_t delayMs = 1000) {
  restartRequested = true;
  restartAt = millis() + delayMs;
}

// -----------------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------------

void loadConfig() {
  prefs.begin("voicedot", true);

  cfg.wifiSsid = prefs.getString("wifi_ssid", "");
  cfg.wifiPass = prefs.getString("wifi_pass", "");
  cfg.deviceName = prefs.getString("dev_name", "VoiceDot");
  cfg.haUrl = prefs.getString("ha_url", "");
  cfg.haToken = prefs.getString("ha_token", "");
  cfg.volume = prefs.getUChar("volume", 60);
  cfg.ledBrightness = prefs.getUChar("led_bri", 32);

  prefs.end();

  if (cfg.volume > 100) cfg.volume = 60;
  if (cfg.ledBrightness > LED_BRIGHTNESS_MAX) cfg.ledBrightness = 30;
  ledBrightness = cfg.ledBrightness;
  muted = (cfg.volume == 0);
  volumeBeforeMute = cfg.volume > 0 ? cfg.volume : 60;
}

void saveConfig() {
  prefs.begin("voicedot", false);

  prefs.putString("wifi_ssid", cfg.wifiSsid);
  prefs.putString("wifi_pass", cfg.wifiPass);
  prefs.putString("dev_name", cfg.deviceName);
  prefs.putString("ha_url", cfg.haUrl);
  prefs.putString("ha_token", cfg.haToken);
  prefs.putUChar("volume", cfg.volume);
  prefs.putUChar("led_bri", cfg.ledBrightness);

  prefs.end();
}

void clearConfig() {
  prefs.begin("voicedot", false);
  prefs.clear();
  prefs.end();
}

void saveRuntimeConfig() {
  prefs.begin("voicedot", false);
  prefs.putUChar("volume", cfg.volume);
  prefs.putUChar("led_bri", cfg.ledBrightness);
  prefs.end();
}

// -----------------------------------------------------------------------------
// I2C / board detection
// -----------------------------------------------------------------------------

bool i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

uint8_t countAudioDevicesOnBus(int sda, int scl) {
  Wire.end();
  delay(20);

  if (!Wire.begin(sda, scl, 100000)) {
    return 0;
  }

  delay(30);

  uint8_t score = 0;

  if (i2cProbe(ADDR_ES8311)) score += 3;
  if (i2cProbe(ADDR_ES7210)) score += 3;
  if (i2cProbe(ADDR_TCA9555)) score += 2;
  if (i2cProbe(ADDR_RTC)) score += 1;

  return score;
}

void detectBoardProfile() {
  Serial.println();
  Serial.println("Detecting Waveshare hardware profile...");

  uint8_t scoreA = countAudioDevicesOnBus(PROFILE_A.i2cSda, PROFILE_A.i2cScl);
  Serial.printf("Profile A (%s, SDA=%d SCL=%d) score: %u\n",
                PROFILE_A.name, PROFILE_A.i2cSda, PROFILE_A.i2cScl, scoreA);

  uint8_t scoreB = countAudioDevicesOnBus(PROFILE_B.i2cSda, PROFILE_B.i2cScl);
  Serial.printf("Profile B (%s, SDA=%d SCL=%d) score: %u\n",
                PROFILE_B.name, PROFILE_B.i2cSda, PROFILE_B.i2cScl, scoreB);

  if (scoreB > scoreA) {
    activeProfile = PROFILE_B;
    profileDetected = scoreB > 0;
  } else {
    activeProfile = PROFILE_A;
    profileDetected = scoreA > 0;
  }

  Wire.end();
  delay(20);

  Wire.begin(activeProfile.i2cSda, activeProfile.i2cScl, 400000);
  delay(30);

  es8311Present = i2cProbe(ADDR_ES8311);
  es7210Present = i2cProbe(ADDR_ES7210);
  tca9555Present = i2cProbe(ADDR_TCA9555);
  rtcPresent = i2cProbe(ADDR_RTC);

  Serial.printf("Selected profile: %s\n", activeProfile.name);
  Serial.printf("I2C SDA=%d SCL=%d\n",
                activeProfile.i2cSda, activeProfile.i2cScl);

  Serial.printf("ES8311 0x18: %s\n", es8311Present ? "FOUND" : "not found");
  Serial.printf("ES7210 0x40: %s\n", es7210Present ? "FOUND" : "not found");
  Serial.printf("TCA9555 0x20: %s\n", tca9555Present ? "FOUND" : "not found");
  Serial.printf("RTC 0x51: %s\n", rtcPresent ? "FOUND" : "not found");
}

// -----------------------------------------------------------------------------
// TCA9555
//
// TCA9555 register map:
// 0x00 input port 0
// 0x01 input port 1
// 0x02 output port 0
// 0x03 output port 1
// 0x04 polarity port 0
// 0x05 polarity port 1
// 0x06 config port 0
// 0x07 config port 1
//
// Current Waveshare community mapping reports PA on EXIO8,
// corresponding to port 1 bit 0.
// -----------------------------------------------------------------------------

bool tcaWriteReg(uint8_t reg, uint8_t value) {
  if (!tca9555Present) return false;

  Wire.beginTransmission(ADDR_TCA9555);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool tcaReadReg(uint8_t reg, uint8_t &value) {
  if (!tca9555Present) return false;

  Wire.beginTransmission(ADDR_TCA9555);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR_TCA9555, 1) != 1) return false;

  value = Wire.read();
  return true;
}

bool setAmplifier(bool enable) {
  if (!tca9555Present) {
    amplifierEnabled = false;
    return false;
  }

  uint8_t cfg1 = 0xFF;
  uint8_t out1 = 0x00;

  if (!tcaReadReg(0x07, cfg1)) return false;
  if (!tcaReadReg(0x03, out1)) return false;

  // Port 1 bit 0 = EXIO8
  cfg1 &= ~(1 << 0);

  if (enable)
    out1 |= (1 << 0);
  else
    out1 &= ~(1 << 0);

  if (!tcaWriteReg(0x03, out1)) return false;
  if (!tcaWriteReg(0x07, cfg1)) return false;

  amplifierEnabled = enable;
  Serial.printf("Amplifier: %s\n", enable ? "ON" : "OFF");
  return true;
}

bool setupButtons() {
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

  if (!tca9555Present) {
    Serial.println("Buttons: BOOT only, TCA9555 keys unavailable");
    return false;
  }

  uint8_t cfg1 = 0xFF;
  uint8_t pol1 = 0x00;

  if (!tcaReadReg(0x07, cfg1)) return false;
  if (!tcaReadReg(0x05, pol1)) return false;

  cfg1 |= (1 << TCA_KEY1_BIT) | (1 << TCA_KEY2_BIT) | (1 << TCA_KEY3_BIT);
  pol1 &= ~((1 << TCA_KEY1_BIT) | (1 << TCA_KEY2_BIT) | (1 << TCA_KEY3_BIT));

  if (!tcaWriteReg(0x05, pol1)) return false;
  if (!tcaWriteReg(0x07, cfg1)) return false;

  Serial.println("Buttons: K1=volume up, K2 short=wake/record, K2 long=mute, K3=volume down, BOOT=LED brightness");
  return true;
}

// -----------------------------------------------------------------------------
// Audio codec + I2S test layer
// -----------------------------------------------------------------------------

bool codecWrite(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool codecRead(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)address, 1) != 1) return false;

  value = Wire.read();
  return true;
}

bool codecUpdate(uint8_t address, uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t oldValue = 0;
  if (!codecRead(address, reg, oldValue)) return false;
  return codecWrite(address, reg, (oldValue & ~mask) | (value & mask));
}

uint8_t es8311VolumeReg(uint8_t percent) {
  if (percent == 0) return 0x00;
  return map(percent, 1, 100, 32, 255);
}

void es8311SetMute(bool mute) {
  if (!codecPlaybackReady) return;

  uint8_t reg = 0;
  if (!codecRead(ADDR_ES8311, 0x31, reg)) return;
  reg &= 0x9F;
  if (mute) reg |= 0x60;
  codecWrite(ADDR_ES8311, 0x31, reg);
}

void es8311SetVolume(uint8_t percent) {
  if (!codecPlaybackReady) return;

  codecWrite(ADDR_ES8311, 0x32, es8311VolumeReg(percent));
  es8311SetMute(percent == 0 || muted);
}

bool setupI2s() {
  if (audioI2sReady) {
    i2s_driver_uninstall(AUDIO_I2S_PORT);
    audioI2sReady = false;
  }

  i2s_config_t i2sConfig = {};
  i2sConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  i2sConfig.sample_rate = AUDIO_SAMPLE_RATE;
  i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2sConfig.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2sConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2sConfig.dma_buf_count = 6;
  i2sConfig.dma_buf_len = 256;
  i2sConfig.use_apll = false;
  i2sConfig.tx_desc_auto_clear = true;
  i2sConfig.fixed_mclk = AUDIO_MCLK_HZ;
  i2sConfig.mclk_multiple = I2S_MCLK_MULTIPLE_768;
  i2sConfig.bits_per_chan = I2S_BITS_PER_CHAN_16BIT;

  esp_err_t err = i2s_driver_install(AUDIO_I2S_PORT, &i2sConfig, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("I2S install failed: %d\n", (int)err);
    return false;
  }

  i2s_pin_config_t pinConfig = {};
  pinConfig.mck_io_num = activeProfile.i2sMclk;
  pinConfig.bck_io_num = activeProfile.i2sBclk;
  pinConfig.ws_io_num = activeProfile.i2sLrclk;
  pinConfig.data_out_num = activeProfile.speakerData;
  pinConfig.data_in_num = activeProfile.micData;

  err = i2s_set_pin(AUDIO_I2S_PORT, &pinConfig);
  if (err != ESP_OK) {
    Serial.printf("I2S pin setup failed: %d\n", (int)err);
    i2s_driver_uninstall(AUDIO_I2S_PORT);
    return false;
  }

  err = i2s_set_clk(AUDIO_I2S_PORT, AUDIO_SAMPLE_RATE,
                    I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  if (err != ESP_OK) {
    Serial.printf("I2S clock setup failed: %d\n", (int)err);
    i2s_driver_uninstall(AUDIO_I2S_PORT);
    return false;
  }

  i2s_zero_dma_buffer(AUDIO_I2S_PORT);
  audioI2sReady = true;
  Serial.println("I2S: 16 kHz, 16-bit stereo, shared RX/TX ready");
  return true;
}

bool setupEs8311() {
  if (!es8311Present) return false;

  bool ok = true;

  ok &= codecWrite(ADDR_ES8311, 0x44, 0x08);
  ok &= codecWrite(ADDR_ES8311, 0x44, 0x08);
  ok &= codecWrite(ADDR_ES8311, 0x01, 0x30);
  ok &= codecWrite(ADDR_ES8311, 0x02, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x03, 0x10);
  ok &= codecWrite(ADDR_ES8311, 0x16, 0x24);
  ok &= codecWrite(ADDR_ES8311, 0x04, 0x10);
  ok &= codecWrite(ADDR_ES8311, 0x05, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x0B, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x0C, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x10, 0x1F);
  ok &= codecWrite(ADDR_ES8311, 0x11, 0x7F);
  ok &= codecWrite(ADDR_ES8311, 0x00, 0x80);
  ok &= codecWrite(ADDR_ES8311, 0x00, 0x80);  // slave mode
  ok &= codecWrite(ADDR_ES8311, 0x01, 0x3F);  // MCLK from pad
  ok &= codecUpdate(ADDR_ES8311, 0x06, 0x20, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x13, 0x10);
  ok &= codecWrite(ADDR_ES8311, 0x1B, 0x0A);
  ok &= codecWrite(ADDR_ES8311, 0x1C, 0x6A);
  ok &= codecWrite(ADDR_ES8311, 0x44, 0x58);

  // 16 kHz at 12.288 MHz MCLK.
  ok &= codecUpdate(ADDR_ES8311, 0x09, 0x1C, 0x0C);
  ok &= codecUpdate(ADDR_ES8311, 0x0A, 0x1C, 0x0C);
  ok &= codecUpdate(ADDR_ES8311, 0x09, 0x03, 0x00);
  ok &= codecUpdate(ADDR_ES8311, 0x0A, 0x03, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x02, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x05, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x03, 0x10);
  ok &= codecWrite(ADDR_ES8311, 0x04, 0x10);
  ok &= codecWrite(ADDR_ES8311, 0x07, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x08, 0xFF);
  ok &= codecWrite(ADDR_ES8311, 0x06, 0x04);

  ok &= codecWrite(ADDR_ES8311, 0x00, 0x80);
  ok &= codecWrite(ADDR_ES8311, 0x01, 0x3F);
  ok &= codecUpdate(ADDR_ES8311, 0x09, 0x40, 0x00);
  ok &= codecUpdate(ADDR_ES8311, 0x0A, 0x40, 0x40);
  ok &= codecWrite(ADDR_ES8311, 0x17, 0xBF);
  ok &= codecWrite(ADDR_ES8311, 0x0E, 0x02);
  ok &= codecWrite(ADDR_ES8311, 0x12, 0x00);
  ok &= codecWrite(ADDR_ES8311, 0x14, 0x1A);
  ok &= codecWrite(ADDR_ES8311, 0x0D, 0x01);
  ok &= codecWrite(ADDR_ES8311, 0x15, 0x40);
  ok &= codecWrite(ADDR_ES8311, 0x37, 0x08);
  ok &= codecWrite(ADDR_ES8311, 0x45, 0x00);

  codecPlaybackReady = ok;
  if (ok) {
    es8311SetVolume(cfg.volume);
    Serial.println("ES8311: playback codec ready");
  } else {
    Serial.println("ES8311: init failed");
  }

  return ok;
}

uint8_t es7210GainReg(float db) {
  db += 0.5f;
  if (db < 3.0f) return 0;
  if (db < 33.0f) return constrain((int)(db / 3.0f), 0, 10);
  if (db < 34.5f) return 10;
  if (db < 36.0f) return 12;
  if (db < 37.0f) return 13;
  return 14;
}

bool setupEs7210() {
  if (!es7210Present) return false;

  const uint8_t gain = es7210GainReg(30.0f);
  bool ok = true;

  ok &= codecWrite(ADDR_ES7210, 0x00, 0xFF);
  ok &= codecWrite(ADDR_ES7210, 0x00, 0x41);
  ok &= codecWrite(ADDR_ES7210, 0x01, 0x3F);
  ok &= codecWrite(ADDR_ES7210, 0x09, 0x30);
  ok &= codecWrite(ADDR_ES7210, 0x0A, 0x30);
  ok &= codecWrite(ADDR_ES7210, 0x23, 0x2A);
  ok &= codecWrite(ADDR_ES7210, 0x22, 0x0A);
  ok &= codecWrite(ADDR_ES7210, 0x20, 0x0A);
  ok &= codecWrite(ADDR_ES7210, 0x21, 0x2A);
  ok &= codecUpdate(ADDR_ES7210, 0x08, 0x01, 0x00); // slave mode
  ok &= codecWrite(ADDR_ES7210, 0x40, 0x43);
  ok &= codecWrite(ADDR_ES7210, 0x41, 0x70);
  ok &= codecWrite(ADDR_ES7210, 0x42, 0x70);
  ok &= codecWrite(ADDR_ES7210, 0x07, 0x20);
  ok &= codecWrite(ADDR_ES7210, 0x02, 0xC1);

  ok &= codecUpdate(ADDR_ES7210, 0x11, 0xE0, 0x60); // 16-bit
  ok &= codecUpdate(ADDR_ES7210, 0x11, 0x03, 0x00); // I2S normal
  ok &= codecWrite(ADDR_ES7210, 0x12, 0x00);        // non-TDM stereo
  ok &= codecUpdate(ADDR_ES7210, 0x14, 0x03, 0x00);
  ok &= codecUpdate(ADDR_ES7210, 0x15, 0x03, 0x00);

  ok &= codecWrite(ADDR_ES7210, 0x43, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x44, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x45, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x46, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x4B, 0xFF);
  ok &= codecWrite(ADDR_ES7210, 0x4C, 0xFF);
  ok &= codecUpdate(ADDR_ES7210, 0x01, 0x0B, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x4B, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x43, 0x10 | gain);
  ok &= codecWrite(ADDR_ES7210, 0x44, 0x10 | gain);

  ok &= codecWrite(ADDR_ES7210, 0x01, 0x34);
  ok &= codecWrite(ADDR_ES7210, 0x06, 0x00);
  ok &= codecWrite(ADDR_ES7210, 0x40, 0x43);
  ok &= codecWrite(ADDR_ES7210, 0x47, 0x08);
  ok &= codecWrite(ADDR_ES7210, 0x48, 0x08);
  ok &= codecWrite(ADDR_ES7210, 0x49, 0x08);
  ok &= codecWrite(ADDR_ES7210, 0x4A, 0x08);
  ok &= codecWrite(ADDR_ES7210, 0x00, 0x71);
  ok &= codecWrite(ADDR_ES7210, 0x00, 0x41);

  codecRecordReady = ok;
  Serial.println(ok ? "ES7210: microphone ADC ready" : "ES7210: init failed");
  return ok;
}

bool setupAudio() {
  codecPlaybackReady = false;
  codecRecordReady = false;
  micLevel = 0;
  micPeak = 0;
  micDb = -96.0f;
  micReadCount = 0;
  micReadErrors = 0;
  micSilentFrames = 0;
  micLastBytes = 0;

  if (!es8311Present && !es7210Present) {
    Serial.println("Audio: no ES8311/ES7210 devices found");
    return false;
  }

  bool i2sOk = setupI2s();
  bool outOk = setupEs8311();
  bool inOk = setupEs7210();

  Serial.printf("Audio: I2S=%s playback=%s record=%s\n",
                i2sOk ? "OK" : "FAIL",
                outOk ? "OK" : "FAIL",
                inOk ? "OK" : "FAIL");
  return i2sOk && (outOk || inOk);
}

void pollMicLevel() {
  if (!audioI2sReady || !codecRecordReady || speakerTestActive) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastMicPoll) < 45) return;
  lastMicPoll = now;

  int16_t samples[AUDIO_FRAME_SAMPLES * 2];
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(AUDIO_I2S_PORT, samples, sizeof(samples),
                           &bytesRead, pdMS_TO_TICKS(1));
  if (err != ESP_OK || bytesRead < sizeof(int16_t) * 2) {
    micReadErrors++;
    micLastBytes = bytesRead;
    return;
  }

  micReadCount++;
  micLastBytes = bytesRead;

  size_t count = bytesRead / sizeof(int16_t);
  int32_t peak = 0;
  uint64_t sumSquares = 0;

  for (size_t i = 0; i < count; i++) {
    int32_t s = samples[i];
    int32_t a = abs(s);
    if (a > peak) peak = a;
    sumSquares += (uint32_t)(s * s);
  }

  float rms = sqrtf((float)sumSquares / (float)count);
  micPeak = constrain(peak, 0, 32767);
  micDb = rms > 1.0f ? 20.0f * log10f(rms / 32768.0f) : -96.0f;
  if (micPeak < 4) micSilentFrames++;
  micLevel = constrain((int)map(micPeak, 0, 4000, 0, 100), 0, 100);
}

void playSpeakerTestTone() {
  if (!audioI2sReady || !codecPlaybackReady) return;

  speakerTestActive = true;
  setAmplifier(true);
  es8311SetMute(false);
  es8311SetVolume(cfg.volume > 0 ? cfg.volume : 60);
  i2s_zero_dma_buffer(AUDIO_I2S_PORT);

  const uint32_t durationMs = 900;
  const uint32_t totalFrames = (AUDIO_SAMPLE_RATE * durationMs) / 1000;
  const float freq = 440.0f;
  const float step = 2.0f * PI * freq / (float)AUDIO_SAMPLE_RATE;
  const int16_t amplitude = 6000;
  int16_t buffer[128 * 2];
  uint32_t frame = 0;

  while (frame < totalFrames) {
    uint32_t framesThisChunk = min<uint32_t>(128, totalFrames - frame);

    for (uint32_t i = 0; i < framesThisChunk; i++) {
      float env = 1.0f;
      uint32_t pos = frame + i;
      if (pos < 800) env = (float)pos / 800.0f;
      if (totalFrames - pos < 800) env = (float)(totalFrames - pos) / 800.0f;

      int16_t s = (int16_t)(sinf((float)pos * step) * amplitude * env);
      buffer[i * 2] = s;
      buffer[i * 2 + 1] = s;
    }

    size_t bytesWritten = 0;
    i2s_write(AUDIO_I2S_PORT, buffer, framesThisChunk * 2 * sizeof(int16_t),
              &bytesWritten, pdMS_TO_TICKS(100));
    frame += framesThisChunk;
    server.handleClient();
    yield();
  }

  i2s_zero_dma_buffer(AUDIO_I2S_PORT);
  es8311SetVolume(cfg.volume);
  if (cfg.volume == 0 || muted) es8311SetMute(true);
  delay(40);
  setAmplifier(false);
  speakerTestActive = false;
}

bool wsSendFrame(Client &client, uint8_t opcode, const uint8_t *payload, size_t len) {
  uint8_t header[14];
  size_t h = 0;

  header[h++] = 0x80 | opcode;
  if (len < 126) {
    header[h++] = 0x80 | (uint8_t)len;
  } else if (len <= 65535) {
    header[h++] = 0x80 | 126;
    header[h++] = (uint8_t)(len >> 8);
    header[h++] = (uint8_t)len;
  } else {
    header[h++] = 0x80 | 127;
    for (int i = 7; i >= 0; i--) header[h++] = (uint8_t)((uint64_t)len >> (i * 8));
  }

  const uint8_t mask[4] = {0x56, 0x44, 0x30, 0x34};
  for (uint8_t i = 0; i < 4; i++) header[h++] = mask[i];

  if (client.write(header, h) != h) return false;

  uint8_t buf[256];
  size_t sent = 0;
  while (sent < len) {
    size_t chunk = min<size_t>(sizeof(buf), len - sent);
    for (size_t i = 0; i < chunk; i++) buf[i] = payload[sent + i] ^ mask[(sent + i) & 3];
    if (client.write(buf, chunk) != chunk) return false;
    sent += chunk;
  }

  return true;
}

bool wsSendText(Client &client, const String &text) {
  return wsSendFrame(client, 0x1, (const uint8_t*)text.c_str(), text.length());
}

bool wsReadFrame(Client &client, String &text, uint8_t &opcode, uint32_t timeoutMs) {
  uint32_t start = millis();
  text = "";
  opcode = 0;

  while (client.connected() && client.available() < 2) {
    if ((uint32_t)(millis() - start) > timeoutMs) return false;
    delay(2);
    yield();
  }

  if (client.available() < 2) return false;

  uint8_t b0 = client.read();
  uint8_t b1 = client.read();
  opcode = b0 & 0x0F;
  bool masked = (b1 & 0x80) != 0;
  uint64_t len = b1 & 0x7F;

  if (len == 126) {
    while (client.available() < 2) delay(1);
    len = ((uint16_t)client.read() << 8) | client.read();
  } else if (len == 127) {
    len = 0;
    while (client.available() < 8) delay(1);
    for (int i = 0; i < 8; i++) len = (len << 8) | client.read();
  }

  uint8_t mask[4] = {0, 0, 0, 0};
  if (masked) {
    while (client.available() < 4) delay(1);
    for (uint8_t i = 0; i < 4; i++) mask[i] = client.read();
  }

  if (len > 8192) {
    for (uint64_t i = 0; i < len; i++) {
      while (!client.available()) {
        if ((uint32_t)(millis() - start) > timeoutMs) return false;
        delay(1);
      }
      client.read();
    }
    return true;
  }

  text.reserve((size_t)len + 1);
  for (uint64_t i = 0; i < len; i++) {
    while (!client.available()) {
      if ((uint32_t)(millis() - start) > timeoutMs) return false;
      delay(1);
      yield();
    }
    char c = (char)client.read();
    if (masked) c ^= mask[i & 3];
    text += c;
  }

  return true;
}

bool wsConnectHa(Client &client, const String &host, uint16_t port, const String &token) {
  if (!client.connect(host.c_str(), port)) return false;

  String req;
  req.reserve(420 + token.length());
  req += "GET /api/websocket HTTP/1.1\r\n";
  req += "Host: " + host + ":" + String(port) + "\r\n";
  req += "Upgrade: websocket\r\n";
  req += "Connection: Upgrade\r\n";
  req += "Sec-WebSocket-Key: Vm9pY2VEb3QtVjA0LVRlc3Q=\r\n";
  req += "Sec-WebSocket-Version: 13\r\n";
  req += "Origin: http://voicedot.local\r\n\r\n";

  client.print(req);

  String line;
  uint32_t start = millis();
  bool upgraded = false;
  while ((uint32_t)(millis() - start) < 5000) {
    line = client.readStringUntil('\n');
    line.trim();
    if (line.startsWith("HTTP/1.1 101")) upgraded = true;
    if (line.length() == 0 && upgraded) break;
  }

  if (!upgraded) return false;

  String msg;
  uint8_t opcode = 0;
  if (!wsReadFrame(client, msg, opcode, 3000)) return false;
  if (msg.indexOf("auth_required") < 0) return false;

  wsSendText(client, "{\"type\":\"auth\",\"access_token\":\"" + token + "\"}");
  if (!wsReadFrame(client, msg, opcode, 5000)) return false;

  return msg.indexOf("auth_ok") >= 0;
}

bool recordWakeAudio(uint8_t **audioOut, size_t *bytesOut) {
  *audioOut = nullptr;
  *bytesOut = 0;

  if (!audioI2sReady || !codecRecordReady) {
    wakeLastMessage = "Mikrofon ist nicht bereit.";
    return false;
  }

  uint8_t *audio = (uint8_t*)ps_malloc(WAKE_MAX_BYTES);
  if (!audio) {
    wakeLastMessage = "PSRAM-Aufnahmepuffer konnte nicht reserviert werden.";
    return false;
  }

  wakeRecording = true;
  wakeLastState = "recording";
  wakeLastMessage = "Aufnahme läuft.";
  wakeLastBytes = 0;
  uint32_t started = millis();

  int16_t stereo[AUDIO_FRAME_SAMPLES * 2];
  size_t bytesRead = 0;

  while ((uint32_t)(millis() - started) < WAKE_RECORD_MS &&
         wakeLastBytes + AUDIO_FRAME_SAMPLES * sizeof(int16_t) <= WAKE_MAX_BYTES) {
    esp_err_t err = i2s_read(AUDIO_I2S_PORT, stereo, sizeof(stereo),
                             &bytesRead, pdMS_TO_TICKS(100));
    if (err == ESP_OK && bytesRead >= sizeof(int16_t) * 2) {
      size_t stereoSamples = bytesRead / sizeof(int16_t);
      for (size_t i = 0; i + 1 < stereoSamples &&
           wakeLastBytes + sizeof(int16_t) <= WAKE_MAX_BYTES; i += 2) {
        int32_t mono = ((int32_t)stereo[i] + (int32_t)stereo[i + 1]) / 2;
        audio[wakeLastBytes++] = (uint8_t)(mono & 0xFF);
        audio[wakeLastBytes++] = (uint8_t)((mono >> 8) & 0xFF);
      }
    }
    server.handleClient();
    yield();
  }

  wakeRecording = false;
  wakeLastDurationMs = millis() - started;
  *audioOut = audio;
  *bytesOut = wakeLastBytes;

  if (wakeLastBytes < AUDIO_SAMPLE_RATE / 2) {
    free(audio);
    *audioOut = nullptr;
    *bytesOut = 0;
    wakeLastMessage = "Aufnahme war zu kurz.";
    return false;
  }

  return true;
}

bool sendAudioToHaAssist(const uint8_t *audio, size_t bytes) {
  if (WiFi.status() != WL_CONNECTED) {
    wakeLastMessage = "WLAN ist nicht verbunden.";
    return false;
  }

  if (cfg.haUrl.isEmpty() || cfg.haToken.isEmpty()) {
    wakeLastMessage = "Home-Assistant-URL oder Token fehlt.";
    return false;
  }

  String host;
  uint16_t port = 0;
  bool secure = false;
  if (!parseHaUrl(cfg.haUrl, host, port, secure)) {
    wakeLastMessage = "Home-Assistant-URL ist ungültig.";
    return false;
  }

  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  Client *client = nullptr;
  if (secure) {
    secureClient.setInsecure();
    client = &secureClient;
  } else {
    client = &plainClient;
  }

  wakeSending = true;
  wakeLastState = "sending";
  wakeLastMessage = "Sende Audio an Home Assistant.";
  wakeLastHttpCode = 0;
  wakeTranscript = "";
  wakeTtsUrl = "";

  if (!wsConnectHa(*client, host, port, cfg.haToken)) {
    wakeSending = false;
    wakeLastMessage = "HA WebSocket/Auth fehlgeschlagen.";
    client->stop();
    return false;
  }

  wakeLastHttpCode = 101;
  String run = "{\"id\":1,\"type\":\"assist_pipeline/run\",\"start_stage\":\"stt\",\"end_stage\":\"tts\",\"input\":{\"sample_rate\":16000},\"timeout\":30}";
  wsSendText(*client, run);

  int handlerId = -1;
  String msg;
  uint8_t opcode = 0;
  uint32_t waitStart = millis();

  while ((uint32_t)(millis() - waitStart) < 8000) {
    if (!wsReadFrame(*client, msg, opcode, 8000)) break;
    if (opcode == 0x8) break;
    int id = jsonFindInt(msg, "stt_binary_handler_id", -1);
    if (id >= 0) {
      handlerId = id;
      break;
    }
    if (msg.indexOf("\"type\":\"event\"") >= 0 && msg.indexOf("\"error\"") >= 0) {
      wakeLastMessage = "HA meldet Fehler vor Audio: " + msg.substring(0, min(180, (int)msg.length()));
      client->stop();
      wakeSending = false;
      return false;
    }
  }

  if (handlerId < 0 || handlerId > 255) {
    wakeLastMessage = "HA hat keinen STT-Audiokanal geliefert.";
    client->stop();
    wakeSending = false;
    return false;
  }

  uint8_t frame[513];
  frame[0] = (uint8_t)handlerId;
  size_t sent = 0;
  while (sent < bytes) {
    size_t chunk = min<size_t>(512, bytes - sent);
    memcpy(frame + 1, audio + sent, chunk);
    if (!wsSendFrame(*client, 0x2, frame, chunk + 1)) {
      wakeLastMessage = "Audio-Upload zu HA abgebrochen.";
      client->stop();
      wakeSending = false;
      return false;
    }
    sent += chunk;
    delay(2);
    yield();
  }

  uint8_t endFrame[1] = {(uint8_t)handlerId};
  wsSendFrame(*client, 0x2, endFrame, 1);

  bool ok = false;
  uint32_t resultStart = millis();
  while ((uint32_t)(millis() - resultStart) < 30000) {
    if (!wsReadFrame(*client, msg, opcode, 30000)) break;
    if (opcode == 0x8) break;

    String text = jsonFindString(msg, "text");
    if (text.length() > 0) wakeTranscript = text;

    String url = jsonFindString(msg, "url");
    if (url.length() > 0) wakeTtsUrl = url;

    if (msg.indexOf("\"type\":\"event\"") >= 0 && msg.indexOf("\"error\"") >= 0) {
      wakeLastMessage = "HA Assist Fehler: " + msg.substring(0, min(180, (int)msg.length()));
      break;
    }

    if (msg.indexOf("\"event_type\":\"run-end\"") >= 0 || msg.indexOf("\"type\":\"result\"") >= 0) {
      ok = true;
      break;
    }

    server.handleClient();
    yield();
  }

  client->stop();
  wakeSending = false;
  wakeLastState = ok ? "done" : "error";
  if (ok) {
    wakeLastMessage = wakeTranscript.length() > 0
                      ? "HA Assist fertig: " + wakeTranscript
                      : "HA Assist fertig.";
  } else if (wakeLastMessage.length() == 0 || wakeLastMessage == "Sende Audio an Home Assistant.") {
    wakeLastMessage = "HA Assist hat nicht rechtzeitig geantwortet.";
  }

  return ok;
}

void runWakeCaptureAndHa() {
  if (wakeBusy) return;

  wakeBusy = true;
  wakeLastState = "wake";
  wakeLastMessage = "Wake erkannt.";
  wakeTranscript = "";
  wakeTtsUrl = "";
  flashLeds(0, 80, 160);

  uint8_t *audio = nullptr;
  size_t bytes = 0;
  bool ok = recordWakeAudio(&audio, &bytes);
  if (ok && audio) {
    ok = sendAudioToHaAssist(audio, bytes);
  }

  if (audio) free(audio);
  if (!ok && wakeLastState != "error") wakeLastState = "error";
  if (apMode) ledsStatusSetup();
  else ledsStatusReady();
  wakeBusy = false;
}

// -----------------------------------------------------------------------------
// LED ring
// -----------------------------------------------------------------------------

void setupLeds() {
  if (pixels != nullptr) {
    delete pixels;
    pixels = nullptr;
  }

  pixels = new Adafruit_NeoPixel(
    7,
    activeProfile.ledPin,
    NEO_GRB + NEO_KHZ800
  );

  pixels->begin();
  pixels->setBrightness(ledBrightness);
  pixels->clear();
  pixels->show();
}

void applyLedBrightness() {
  ledBrightness = cfg.ledBrightness;

  if (!pixels) return;

  pixels->setBrightness(ledBrightness);
  pixels->show();
}

void setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  if (!pixels) return;

  for (int i = 0; i < 7; i++) {
    pixels->setPixelColor(i, pixels->Color(r, g, b));
  }

  pixels->show();
}

void ledsBootAnimation() {
  if (!pixels) return;

  pixels->clear();
  pixels->show();

  for (int i = 0; i < 7; i++) {
    pixels->clear();
    pixels->setPixelColor(i, pixels->Color(0, 80, 180));
    pixels->show();
    delay(50);
  }

  pixels->clear();
  pixels->show();
}

void ledsStatusReady() {
  setAllLeds(0, 80, 25);
}

void ledsStatusSetup() {
  setAllLeds(180, 60, 0);
}

void ledsStatusError() {
  setAllLeds(180, 0, 0);
}

void flashLeds(uint8_t r, uint8_t g, uint8_t b) {
  if (!pixels) return;

  setAllLeds(r, g, b);
  delay(70);

  if (apMode) ledsStatusSetup();
  else ledsStatusReady();
}

bool updateButton(ButtonState &button, bool pressedNow) {
  uint32_t now = millis();

  if (pressedNow != button.rawPressed) {
    button.rawPressed = pressedNow;
    button.changedAt = now;
  }

  if ((uint32_t)(now - button.changedAt) >= BUTTON_DEBOUNCE_MS &&
      button.stablePressed != button.rawPressed) {
    button.lastStablePressed = button.stablePressed;
    button.stablePressed = button.rawPressed;
    return button.stablePressed && !button.lastStablePressed;
  }

  return false;
}

void setVolume(uint8_t volume) {
  cfg.volume = constrain(volume, 0, 100);
  muted = (cfg.volume == 0);

  if (cfg.volume > 0) {
    volumeBeforeMute = cfg.volume;
  }

  es8311SetVolume(cfg.volume);
  saveRuntimeConfig();
  Serial.printf("Volume: %u%s\n", cfg.volume, muted ? " (muted)" : "");
}

void volumeUp() {
  uint8_t base = muted ? volumeBeforeMute : cfg.volume;
  muted = false;
  setVolume(min<uint8_t>(100, base + VOLUME_STEP));
  flashLeds(0, 90, 30);
}

void volumeDown() {
  muted = false;
  setVolume(cfg.volume > VOLUME_STEP ? cfg.volume - VOLUME_STEP : 0);
  flashLeds(0, 60, 110);
}

void toggleMute() {
  if (!muted && cfg.volume > 0) {
    volumeBeforeMute = cfg.volume;
    setVolume(0);
  } else {
    setVolume(volumeBeforeMute > 0 ? volumeBeforeMute : 60);
  }

  flashLeds(muted ? 120 : 0, muted ? 0 : 90, muted ? 0 : 30);
}

void stepLedBrightness() {
  if (cfg.ledBrightness >= LED_BRIGHTNESS_MAX) {
    cfg.ledBrightness = 0;
  } else {
    cfg.ledBrightness = min<uint8_t>(LED_BRIGHTNESS_MAX,
                                     cfg.ledBrightness + LED_BRIGHTNESS_STEP);
  }

  applyLedBrightness();
  saveRuntimeConfig();
  Serial.printf("LED brightness: %u\n", cfg.ledBrightness);
  flashLeds(60, 60, 160);
}

void updateK2WakeMute(bool pressedNow) {
  uint32_t now = millis();

  if (pressedNow != k2RawPressed) {
    k2RawPressed = pressedNow;
    k2ChangedAt = now;
  }

  if ((uint32_t)(now - k2ChangedAt) < BUTTON_DEBOUNCE_MS ||
      k2StablePressed == k2RawPressed) {
    if (k2StablePressed && !k2LongHandled &&
        (uint32_t)(now - k2PressedAt) >= K2_LONG_PRESS_MS) {
      k2LongHandled = true;
      toggleMute();
      Serial.println("K2 long press: mute toggle");
    }
    return;
  }

  bool wasPressed = k2StablePressed;
  k2StablePressed = k2RawPressed;

  if (k2StablePressed && !wasPressed) {
    k2PressedAt = now;
    k2LongHandled = false;
    return;
  }

  if (!k2StablePressed && wasPressed) {
    uint32_t held = now - k2PressedAt;
    if (!k2LongHandled && held < K2_LONG_PRESS_MS) {
      Serial.println("K2 short press: wake/record");
      runWakeCaptureAndHa();
    }
  }
}

void pollButtons() {
  bool bootPressed = digitalRead(PIN_BOOT_BUTTON) == LOW;

  bool key1Pressed = false;
  bool key2Pressed = false;
  bool key3Pressed = false;

  if (tca9555Present) {
    uint8_t in1 = 0xFF;

    if (tcaReadReg(0x01, in1)) {
      key1Pressed = (in1 & (1 << TCA_KEY1_BIT)) == 0;
      key2Pressed = (in1 & (1 << TCA_KEY2_BIT)) == 0;
      key3Pressed = (in1 & (1 << TCA_KEY3_BIT)) == 0;
    }
  }

  if (updateButton(btnVolumeUp, key1Pressed)) volumeUp();
  updateK2WakeMute(key2Pressed);
  if (updateButton(btnVolumeDown, key3Pressed)) volumeDown();
  if (updateButton(btnLedBrightness, bootPressed)) stepLedBrightness();
}

// -----------------------------------------------------------------------------
// WiFi
// -----------------------------------------------------------------------------

bool connectToWiFi() {
  if (cfg.wifiSsid.isEmpty()) return false;

  Serial.printf("Connecting to WiFi: %s\n", cfg.wifiSsid.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());

  uint32_t started = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - started < 15000) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  WiFi.disconnect(true, true);
  delay(200);
  return false;
}

void startProvisioningAp() {
  apMode = true;

  WiFi.mode(WIFI_AP);
  delay(100);

  String ssid = makeApSsid();

  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(apIP, gateway, subnet);
  WiFi.softAP(ssid.c_str(), AP_PASSWORD);

  dnsServer.start(53, "*", WiFi.softAPIP());

  Serial.printf("Setup AP: %s\n", ssid.c_str());
  Serial.printf("Password: %s\n", AP_PASSWORD);
  Serial.println("URL: http://192.168.4.1");
}

void startMdns() {
  if (apMode) return;

  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS: http://voicedot.local");
  }
}

// -----------------------------------------------------------------------------
// Web UI
// -----------------------------------------------------------------------------

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VoiceDot</title>
<style>
:root{
 --bg:#0c0f14;--card:#151a22;--card2:#1b222d;--line:#2a3442;
 --text:#f4f7fb;--muted:#95a3b5;--accent:#79a6ff;
 --green:#47d885;--red:#ff6b75;--orange:#ffb454;--blue:#79a6ff
}
*{box-sizing:border-box}
body{
 margin:0;background:radial-gradient(circle at 50% -20%,#1b2945 0,#0c0f14 38%);
 color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif
}
.wrap{width:min(1040px,calc(100% - 28px));margin:28px auto 60px}
header{display:flex;align-items:center;justify-content:space-between;gap:15px;margin-bottom:18px}
.brand{display:flex;align-items:center;gap:13px}
.logo{
 width:48px;height:48px;border-radius:50%;
 background:conic-gradient(#76a7ff,#8d7aff,#53d7ff,#76a7ff);
 padding:4px;box-shadow:0 0 30px rgba(100,150,255,.25)
}
.logo:after{content:"";display:block;width:100%;height:100%;border-radius:50%;background:#10151d}
h1{margin:0;font-size:25px}
.subtitle{font-size:13px;color:var(--muted);margin-top:2px}
.tag{border:1px solid var(--line);background:#171e28;padding:8px 11px;border-radius:999px;color:var(--muted);font-size:12px}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}
.card{
 background:rgba(21,26,34,.96);border:1px solid var(--line);
 border-radius:16px;padding:18px;box-shadow:0 15px 40px rgba(0,0,0,.2)
}
.card.full{grid-column:1/-1}
.card h2{margin:0 0 14px;color:var(--muted);font-size:12px;letter-spacing:.14em}
.stat{display:flex;justify-content:space-between;gap:12px;padding:8px 0;border-bottom:1px solid rgba(42,52,66,.6)}
.stat:last-child{border:0}
.value{text-align:right;font-weight:650}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--red);margin-right:7px}
.dot.ok{background:var(--green);box-shadow:0 0 10px rgba(71,216,133,.4)}
.dot.warn{background:var(--orange)}
label{display:block;color:var(--muted);font-size:12px;margin:12px 0 6px}
input{
 width:100%;background:var(--card2);border:1px solid var(--line);border-radius:10px;
 padding:11px 12px;color:var(--text);outline:none
}
input:focus{border-color:var(--accent)}
.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.actions{display:flex;flex-wrap:wrap;gap:9px;margin-top:14px}
button{
 cursor:pointer;border:0;border-radius:10px;padding:10px 14px;font-weight:700;
 background:var(--accent);color:#091224
}
button.secondary{background:#252e3b;color:var(--text);border:1px solid var(--line)}
button.danger{background:#3b2025;color:#ffc0c5;border:1px solid #63313a}
button.green{background:#20412f;color:#a9f3c8;border:1px solid #376b4d}
button.orange{background:#45351e;color:#ffd69a;border:1px solid #71562c}
.help{display:block;margin-top:8px;color:var(--muted);font-size:12px;line-height:1.5}
.hw{
 display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin-top:6px
}
.hwitem{padding:11px;border:1px solid var(--line);background:#111720;border-radius:10px}
.hwtitle{font-size:12px;color:var(--muted)}
.hwstate{font-weight:750;margin-top:4px}
.pinbox{
 font-family:ui-monospace,SFMono-Regular,Consolas,monospace;
 background:#0d1219;border:1px solid var(--line);border-radius:10px;padding:12px;
 line-height:1.65;font-size:12px;color:#c9d7ea;overflow:auto
}
#toast{
 position:fixed;right:18px;bottom:18px;background:#202936;border:1px solid var(--line);
 padding:12px 14px;border-radius:10px;display:none;max-width:360px;z-index:99
}
@media(max-width:720px){
 .grid{grid-template-columns:1fr}.card.full{grid-column:auto}.row{grid-template-columns:1fr}
 .hw{grid-template-columns:1fr}
}
</style>
</head>
<body>
<div class="wrap">
<header>
 <div class="brand">
  <div class="logo"></div>
  <div>
   <h1>VoiceDot</h1>
   <div class="subtitle">Waveshare ESP32-S3 Audio Board</div>
  </div>
 </div>
 <div class="tag">Firmware <span id="fw">...</span></div>
</header>

<div class="grid">

<section class="card">
<h2>SYSTEM</h2>
<div class="stat"><span>Status</span><span class="value"><i class="dot ok"></i>Online</span></div>
<div class="stat"><span>Uptime</span><span id="uptime" class="value">...</span></div>
<div class="stat"><span>IP</span><span id="ip" class="value">...</span></div>
<div class="stat"><span>Heap frei</span><span id="heap" class="value">...</span></div>
<div class="stat"><span>PSRAM frei</span><span id="psram" class="value">...</span></div>
</section>

<section class="card">
<h2>BOARD</h2>
<div class="stat"><span>Profil</span><span id="profile" class="value">...</span></div>
<div class="stat"><span>I²C</span><span id="i2c" class="value">...</span></div>
<div class="stat"><span>LED GPIO</span><span id="ledpin" class="value">...</span></div>
<div class="stat"><span>Audio</span><span id="audioPins" class="value">...</span></div>
</section>

<section class="card full">
<h2>HARDWARE DIAGNOSE</h2>
<div class="hw">
 <div class="hwitem"><div class="hwtitle">ES8311 Audio Codec · 0x18</div><div id="es8311" class="hwstate">...</div></div>
 <div class="hwitem"><div class="hwtitle">ES7210 Dual-Mic ADC · 0x40</div><div id="es7210" class="hwstate">...</div></div>
 <div class="hwitem"><div class="hwtitle">TCA9555 I/O Expander · 0x20</div><div id="tca" class="hwstate">...</div></div>
 <div class="hwitem"><div class="hwtitle">RTC PCF85063 · 0x51</div><div id="rtc" class="hwstate">...</div></div>
</div>
<div class="actions">
 <button class="secondary" onclick="refreshStatus()">Neu prüfen</button>
 <button class="green" onclick="ledTest()">LED-Ring testen</button>
 <button class="orange" onclick="ampToggle()">Verstärker umschalten</button>
</div>
<small class="help">Der Hardwaretest prüft die onboard Bausteine direkt über I²C. So sehen wir sofort, welche Board-Revision geliefert wurde.</small>
</section>

<section class="card">
<h2>NETWORK</h2>
<div class="stat"><span>Modus</span><span id="mode" class="value">...</span></div>
<div class="stat"><span>SSID</span><span id="ssidNow" class="value">...</span></div>
<div class="stat"><span>Signal</span><span id="rssi" class="value">...</span></div>
<div class="stat"><span>Hostname</span><span class="value">voicedot.local</span></div>

<label>WLAN SSID</label>
<input id="wifi_ssid" placeholder="Mein WLAN">

<label>WLAN Passwort</label>
<input id="wifi_pass" type="password" placeholder="Passwort nur eingeben wenn geändert">

<div class="actions"><button class="secondary" onclick="scanWifi()">WLAN suchen</button></div>
<small class="help" id="scanResult"></small>
</section>

<section class="card">
<h2>VOICE DOT</h2>
<label>Gerätename</label>
<input id="device_name" placeholder="VoiceDot Wohnzimmer">

<label>Lautstärke</label>
<input id="volume" type="range" min="0" max="100" value="60"
 oninput="volLabel.textContent=this.value+' %'">
<small id="volLabel" class="help">60 %</small>

<label>LED-Helligkeit</label>
<input id="led_bri" type="range" min="0" max="100" value="30"
 oninput="briLabel.textContent=this.value">
<small id="briLabel" class="help">30</small>
</section>

<section class="card full">
<h2>HOME ASSISTANT</h2>
<div class="row">
 <div>
  <label>Home Assistant URL</label>
  <input id="ha_url" placeholder="http://homeassistant.local:8123">
 </div>
 <div>
  <label>Long-Lived Access Token</label>
  <input id="ha_token" type="password" placeholder="Nur eingeben wenn geändert">
 </div>
</div>
<div class="actions"><button class="secondary" onclick="testHA()">Verbindung testen</button></div>
<small class="help" id="haResult">Noch nicht getestet.</small>
</section>

<section class="card full">
<h2>AUDIO PIPELINE</h2>
<div class="row">
 <div>
  <div class="stat"><span>I²S</span><span id="i2sState" class="value">...</span></div>
  <div class="stat"><span>Playback</span><span id="playState" class="value">...</span></div>
  <div class="stat"><span>Recording</span><span id="recState" class="value">...</span></div>
 </div>
 <div>
  <label>Mic-Level</label>
  <div style="height:14px;background:#202735;border:1px solid #334052;border-radius:8px;overflow:hidden">
   <div id="micBar" style="height:100%;width:0%;background:#47d885"></div>
  </div>
  <small id="micText" class="help">0 %</small>
  <small id="micDiag" class="help">RX ...</small>
 </div>
</div>
<div class="pinbox" id="pinbox">Lade Pinbelegung ...</div>
<div class="pinbox" id="assistBox">Assist bereit ...</div>
<div class="actions">
 <button onclick="wakeTest()">Wake testen</button>
 <button class="secondary" onclick="speakerTest()">Lautsprecher testen</button>
</div>
<small class="help">
K2 kurz startet eine Aufnahme und sendet sie an Home Assistant Assist. K2 lang schaltet Mute.
</small>
</section>

<section class="card full">
<h2>KONFIGURATION</h2>
<div class="actions">
 <button onclick="saveConfig()">Speichern & neu starten</button>
 <button class="secondary" onclick="reboot()">Neu starten</button>
 <button class="danger" onclick="factoryReset()">Werkseinstellungen</button>
</div>
</section>

<section class="card full">
<h2>FIRMWARE UPDATE</h2>
<form id="otaForm">
 <input id="firmware" type="file" accept=".bin,application/octet-stream">
 <div class="actions"><button type="submit">Firmware installieren</button></div>
</form>
<small class="help" id="otaResult">Arduino IDE → Sketch → Export Compiled Binary.</small>
</section>

</div>
</div>

<div id="toast"></div>

<script>
)HTML"
"\n"
"const $=id=>document.getElementById(id);\n"
"\n"
"function toast(m){\n"
" const t=$('toast');t.textContent=m;t.style.display='block';\n"
" clearTimeout(window.__t);window.__t=setTimeout(()=>t.style.display='none',3500);\n"
"}\n"
"\n"
"function bytes(n){\n"
" n=Number(n||0);\n"
" if(n>=1048576)return(n/1048576).toFixed(1)+' MB';\n"
" if(n>=1024)return(n/1024).toFixed(1)+' KB';\n"
" return n+' B';\n"
"}\n"
"\n"
"function uptime(s){\n"
" s=Number(s||0);let d=Math.floor(s/86400);s%=86400;\n"
" let h=Math.floor(s/3600);s%=3600;let m=Math.floor(s/60);let x=s%60;\n"
" return(d?d+'d ':'')+String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(x).padStart(2,'0');\n"
"}\n"
"\n"
"function hw(id,ok){\n"
" $(id).innerHTML=(ok?'<span style=\"color:#47d885\">● GEFUNDEN</span>':'<span style=\"color:#ff6b75\">● NICHT GEFUNDEN</span>');\n"
"}\n"
"\n"
"async function refreshStatus(){\n"
" try{\n"
"  const r=await fetch('/api/status',{cache:'no-store'});\n"
"  const j=await r.json();\n"
"\n"
"  $('fw').textContent=j.firmware;\n"
"  $('uptime').textContent=uptime(j.uptime);\n"
"  $('ip').textContent=j.ip;\n"
"  $('heap').textContent=bytes(j.free_heap);\n"
"  $('psram').textContent=bytes(j.free_psram);\n"
"\n"
"  $('profile').textContent=j.board.profile;\n"
"  $('i2c').textContent='SDA '+j.board.i2c_sda+' / SCL '+j.board.i2c_scl;\n"
"  $('ledpin').textContent=j.board.led_pin;\n"
"  $('audioPins').textContent='MCLK '+j.board.i2s_mclk+' · BCLK '+j.board.i2s_bclk;\n"
"\n"
"  $('mode').textContent=j.ap_mode?'Setup AP':'WiFi Client';\n"
"  $('ssidNow').textContent=j.ssid||'-';\n"
"  $('rssi').textContent=j.ap_mode?'-':j.rssi+' dBm';\n"
"\n"
"  hw('es8311',j.hardware.es8311);\n"
"  hw('es7210',j.hardware.es7210);\n"
"  hw('tca',j.hardware.tca9555);\n"
"  hw('rtc',j.hardware.rtc);\n"
"\n"
"  $('i2sState').textContent=j.audio.i2s?'OK':'OFF';\n"
"  $('playState').textContent=j.audio.playback?'OK':'OFF';\n"
"  $('recState').textContent=j.audio.record?'OK':'OFF';\n"
"  $('micBar').style.width=(j.audio.mic_level||0)+'%';\n"
"  $('micText').textContent=(j.audio.mic_level||0)+' % / '+(j.audio.mic_db||-96).toFixed(1)+' dBFS';\n"
"  $('micDiag').textContent='RX reads '+(j.audio.mic_reads||0)+' · bytes '+(j.audio.mic_last_bytes||0)+' · silent '+(j.audio.mic_silent||0)+' · errors '+(j.audio.mic_errors||0);\n"
"\n"
"  $('assistBox').textContent=\n"
"   'STATE       '+(j.assist.state||'-')+'\\n'+\n"
"   'MESSAGE     '+(j.assist.message||'-')+'\\n'+\n"
"   'REC BYTES   '+(j.assist.bytes||0)+'\\n'+\n"
"   'REC TIME    '+(j.assist.duration_ms||0)+' ms\\n'+\n"
"   'WS CODE     '+(j.assist.ws_code||0)+'\\n'+\n"
"   'TRANSCRIPT  '+(j.assist.transcript||'-')+'\\n'+\n"
"   'TTS URL     '+(j.assist.tts_url||'-');\n"
"\n"
"  $('pinbox').textContent=\n"
"   'I2C SDA     GPIO'+j.board.i2c_sda+'\\n'+\n"
"   'I2C SCL     GPIO'+j.board.i2c_scl+'\\n\\n'+\n"
"   'I2S MCLK    GPIO'+j.board.i2s_mclk+'\\n'+\n"
"   'I2S BCLK    GPIO'+j.board.i2s_bclk+'\\n'+\n"
"   'I2S LRCLK   GPIO'+j.board.i2s_lrclk+'\\n'+\n"
"   'MIC DATA    GPIO'+j.board.mic_data+'\\n'+\n"
"   'SPK DATA    GPIO'+j.board.speaker_data+'\\n\\n'+\n"
"   'RGB RING    GPIO'+j.board.led_pin+'\\n'+\n"
"   'AMP         '+(j.hardware.amp_enabled?'ENABLED':'DISABLED');\n"
" }catch(e){}\n"
"}\n"
"\n"
"async function loadConfig(){\n"
" const r=await fetch('/api/config',{cache:'no-store'});\n"
" const j=await r.json();\n"
"\n"
" $('wifi_ssid').value=j.wifi_ssid||'';\n"
" $('device_name').value=j.device_name||'VoiceDot';\n"
" $('ha_url').value=j.ha_url||'';\n"
"\n"
" $('volume').value=j.volume??60;\n"
" $('volLabel').textContent=$('volume').value+' %';\n"
"\n"
" $('led_bri').value=j.led_brightness??32;\n"
" $('briLabel').textContent=$('led_bri').value;\n"
"}\n"
"\n"
"async function saveConfig(){\n"
" const p=new URLSearchParams();\n"
" p.set('wifi_ssid',$('wifi_ssid').value);\n"
" p.set('wifi_pass',$('wifi_pass').value);\n"
" p.set('device_name',$('device_name').value);\n"
" p.set('ha_url',$('ha_url').value);\n"
" p.set('ha_token',$('ha_token').value);\n"
" p.set('volume',$('volume').value);\n"
" p.set('led_brightness',$('led_bri').value);\n"
"\n"
" const r=await fetch('/api/config',{\n"
"  method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},\n"
"  body:p.toString()\n"
" });\n"
"\n"
" toast(await r.text());\n"
"}\n"
"\n"
"async function scanWifi(){\n"
" $('scanResult').textContent='Suche läuft ...';\n"
"\n"
" try{\n"
"  const r=await fetch('/api/wifi/scan',{cache:'no-store'});\n"
"  const j=await r.json();\n"
"  $('scanResult').innerHTML='';\n"
"\n"
"  (j.networks||[]).forEach(n=>{\n"
"   const b=document.createElement('button');\n"
"   b.type='button';b.className='secondary';\n"
"   b.style.margin='4px 5px 0 0';\n"
"   b.textContent=n.ssid+' ('+n.rssi+' dBm)';\n"
"   b.onclick=()=>{$('wifi_ssid').value=n.ssid};\n"
"   $('scanResult').appendChild(b);\n"
"  });\n"
"\n"
"  if(!j.networks?.length)$('scanResult').textContent='Keine Netze gefunden.';\n"
" }catch(e){$('scanResult').textContent='Scan fehlgeschlagen.'}\n"
"}\n"
"\n"
"async function testHA(){\n"
" $('haResult').textContent='Teste ...';\n"
"\n"
" const p=new URLSearchParams();\n"
" p.set('ha_url',$('ha_url').value);\n"
" p.set('ha_token',$('ha_token').value);\n"
"\n"
" const r=await fetch('/api/ha/test',{\n"
"  method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},\n"
"  body:p.toString()\n"
" });\n"
"\n"
" $('haResult').textContent=await r.text();\n"
"}\n"
"\n"
"async function ledTest(){\n"
" const r=await fetch('/api/hardware/led-test',{method:'POST'});\n"
" toast(await r.text());\n"
"}\n"
"\n"
"async function ampToggle(){\n"
" const r=await fetch('/api/hardware/amp-toggle',{method:'POST'});\n"
" toast(await r.text());\n"
" setTimeout(refreshStatus,300);\n"
"}\n"
"\n"
"async function speakerTest(){\n"
" const r=await fetch('/api/audio/speaker-test',{method:'POST'});\n"
" toast(await r.text());\n"
" setTimeout(refreshStatus,500);\n"
"}\n"
"\n"
"async function wakeTest(){\n"
" const r=await fetch('/api/assist/wake',{method:'POST'});\n"
" toast(await r.text());\n"
" setTimeout(refreshStatus,1000);\n"
"}\n"
"\n"
"async function reboot(){\n"
" if(!confirm('VoiceDot neu starten?'))return;\n"
" await fetch('/api/system/reboot',{method:'POST'});\n"
" toast('Neustart ...');\n"
"}\n"
"\n"
"async function factoryReset(){\n"
" if(!confirm('Alle Einstellungen wirklich löschen?'))return;\n"
" await fetch('/api/system/factory-reset',{method:'POST'});\n"
" toast('Werkseinstellungen werden geladen ...');\n"
"}\n"
"\n"
"$('otaForm').addEventListener('submit',async e=>{\n"
" e.preventDefault();\n"
"\n"
" const f=$('firmware').files[0];\n"
" if(!f){$('otaResult').textContent='Bitte .bin-Datei auswählen.';return}\n"
"\n"
" const fd=new FormData();\n"
" fd.append('firmware',f);\n"
" $('otaResult').textContent='Upload läuft ...';\n"
"\n"
" try{\n"
"  const r=await fetch('/api/ota',{method:'POST',body:fd});\n"
"  $('otaResult').textContent=await r.text();\n"
" }catch(e){$('otaResult').textContent='Upload fehlgeschlagen.'}\n"
"});\n"
"\n"
"loadConfig();\n"
"refreshStatus();\n"
"setInterval(refreshStatus,3000);\n"
"\n"
R"HTML(</script>
</body>
</html>
)HTML";

// -----------------------------------------------------------------------------
// HTTP handlers
// -----------------------------------------------------------------------------

void sendIndex() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  // refresh device presence dynamically
  es8311Present = i2cProbe(ADDR_ES8311);
  es7210Present = i2cProbe(ADDR_ES7210);
  tca9555Present = i2cProbe(ADDR_TCA9555);
  rtcPresent = i2cProbe(ADDR_RTC);

  String ip;
  String ssid;
  int32_t rssi = 0;

  if (apMode) {
    ip = WiFi.softAPIP().toString();
    ssid = WiFi.softAPSSID();
  } else {
    ip = WiFi.localIP().toString();
    ssid = WiFi.SSID();
    rssi = WiFi.RSSI();
  }

  String json;
  json.reserve(1200);

  json += "{";
  json += "\"firmware\":\"" + String(FW_VERSION) + "\",";
  json += "\"uptime\":" + String(millis() / 1000UL) + ",";
  json += "\"ip\":\"" + jsonEscape(ip) + "\",";
  json += "\"ssid\":\"" + jsonEscape(ssid) + "\",";
  json += "\"rssi\":" + String(rssi) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"free_psram\":" + String(ESP.getFreePsram()) + ",";
  json += "\"ap_mode\":" + String(apMode ? "true" : "false") + ",";

  json += "\"board\":{";
  json += "\"profile\":\"" + jsonEscape(activeProfile.name) + "\",";
  json += "\"detected\":" + String(profileDetected ? "true" : "false") + ",";
  json += "\"i2c_sda\":" + String(activeProfile.i2cSda) + ",";
  json += "\"i2c_scl\":" + String(activeProfile.i2cScl) + ",";
  json += "\"i2s_mclk\":" + String(activeProfile.i2sMclk) + ",";
  json += "\"i2s_bclk\":" + String(activeProfile.i2sBclk) + ",";
  json += "\"i2s_lrclk\":" + String(activeProfile.i2sLrclk) + ",";
  json += "\"mic_data\":" + String(activeProfile.micData) + ",";
  json += "\"speaker_data\":" + String(activeProfile.speakerData) + ",";
  json += "\"led_pin\":" + String(activeProfile.ledPin);
  json += "},";

  json += "\"hardware\":{";
  json += "\"es8311\":" + String(es8311Present ? "true" : "false") + ",";
  json += "\"es7210\":" + String(es7210Present ? "true" : "false") + ",";
  json += "\"tca9555\":" + String(tca9555Present ? "true" : "false") + ",";
  json += "\"rtc\":" + String(rtcPresent ? "true" : "false") + ",";
  json += "\"amp_enabled\":" + String(amplifierEnabled ? "true" : "false");
  json += "},";

  json += "\"audio\":{";
  json += "\"i2s\":" + String(audioI2sReady ? "true" : "false") + ",";
  json += "\"playback\":" + String(codecPlaybackReady ? "true" : "false") + ",";
  json += "\"record\":" + String(codecRecordReady ? "true" : "false") + ",";
  json += "\"speaker_test\":" + String(speakerTestActive ? "true" : "false") + ",";
  json += "\"mic_level\":" + String(micLevel) + ",";
  json += "\"mic_peak\":" + String(micPeak) + ",";
  json += "\"mic_db\":" + String(micDb, 1) + ",";
  json += "\"mic_reads\":" + String(micReadCount) + ",";
  json += "\"mic_errors\":" + String(micReadErrors) + ",";
  json += "\"mic_silent\":" + String(micSilentFrames) + ",";
  json += "\"mic_last_bytes\":" + String(micLastBytes);
  json += "},";

  json += "\"assist\":{";
  json += "\"busy\":" + String(wakeBusy ? "true" : "false") + ",";
  json += "\"recording\":" + String(wakeRecording ? "true" : "false") + ",";
  json += "\"sending\":" + String(wakeSending ? "true" : "false") + ",";
  json += "\"state\":\"" + jsonEscape(wakeLastState) + "\",";
  json += "\"message\":\"" + jsonEscape(wakeLastMessage) + "\",";
  json += "\"bytes\":" + String(wakeLastBytes) + ",";
  json += "\"duration_ms\":" + String(wakeLastDurationMs) + ",";
  json += "\"ws_code\":" + String(wakeLastHttpCode) + ",";
  json += "\"transcript\":\"" + jsonEscape(wakeTranscript) + "\",";
  json += "\"tts_url\":\"" + jsonEscape(wakeTtsUrl) + "\"";
  json += "}";

  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

void handleGetConfig() {
  String json;
  json.reserve(500);

  json += "{";
  json += "\"wifi_ssid\":\"" + jsonEscape(cfg.wifiSsid) + "\",";
  json += "\"device_name\":\"" + jsonEscape(cfg.deviceName) + "\",";
  json += "\"ha_url\":\"" + jsonEscape(cfg.haUrl) + "\",";
  json += "\"ha_token_set\":" + String(cfg.haToken.isEmpty() ? "false" : "true") + ",";
  json += "\"volume\":" + String(cfg.volume) + ",";
  json += "\"led_brightness\":" + String(cfg.ledBrightness);
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

void handlePostConfig() {
  if (server.hasArg("wifi_ssid")) {
    String oldSsid = cfg.wifiSsid;
    String newSsid = server.arg("wifi_ssid");
    newSsid.trim();

    // If SSID changes and password field is empty, clear old password.
    if (newSsid != oldSsid && server.arg("wifi_pass").isEmpty()) {
      cfg.wifiPass = "";
    }

    cfg.wifiSsid = newSsid;
  }

  if (server.hasArg("wifi_pass")) {
    String p = server.arg("wifi_pass");
    if (!p.isEmpty()) cfg.wifiPass = p;
  }

  if (server.hasArg("device_name")) {
    cfg.deviceName = server.arg("device_name");
    cfg.deviceName.trim();
    if (cfg.deviceName.isEmpty()) cfg.deviceName = "VoiceDot";
  }

  if (server.hasArg("ha_url")) {
    cfg.haUrl = normalizedHaUrl(server.arg("ha_url"));
  }

  if (server.hasArg("ha_token")) {
    String token = server.arg("ha_token");
    if (!token.isEmpty()) cfg.haToken = token;
  }

  if (server.hasArg("volume")) {
    cfg.volume = constrain(server.arg("volume").toInt(), 0, 100);
  }

  if (server.hasArg("led_brightness")) {
    cfg.ledBrightness = constrain(server.arg("led_brightness").toInt(),
                                  0, LED_BRIGHTNESS_MAX);
  }

  saveConfig();

  server.send(200, "text/plain; charset=utf-8",
              "Gespeichert. VoiceDot startet neu ...");

  scheduleRestart(1100);
}

void handleWifiScan() {
  int n = WiFi.scanNetworks(false, true);

  String json = "{\"networks\":[";
  bool first = true;

  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) continue;

    if (!first) json += ",";
    first = false;

    json += "{";
    json += "\"ssid\":\"" + jsonEscape(ssid) + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI(i));
    json += "}";
  }

  json += "]}";

  WiFi.scanDelete();
  server.send(200, "application/json; charset=utf-8", json);
}

void handleHaTest() {
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/plain; charset=utf-8",
                "Kein WLAN-Client verbunden.");
    return;
  }

  String url = server.hasArg("ha_url")
                 ? normalizedHaUrl(server.arg("ha_url"))
                 : cfg.haUrl;

  String token = server.hasArg("ha_token")
                   ? server.arg("ha_token")
                   : "";

  if (token.isEmpty()) token = cfg.haToken;

  if (url.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8",
                "Home-Assistant-URL fehlt.");
    return;
  }

  if (token.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8",
                "Home-Assistant-Token fehlt.");
    return;
  }

  if (url.startsWith("https://")) {
    server.send(400, "text/plain; charset=utf-8",
                "HTTPS-Test ist in v0.2 noch nicht aktiviert. Für den lokalen Test bitte die interne http:// URL verwenden.");
    return;
  }

  HTTPClient http;

  if (!http.begin(url + "/api/")) {
    server.send(500, "text/plain; charset=utf-8",
                "HTTP-Client konnte nicht initialisiert werden.");
    return;
  }

  http.setConnectTimeout(5000);
  http.setTimeout(7000);
  http.addHeader("Authorization", "Bearer " + token);

  int code = http.GET();
  http.end();

  if (code == 200) {
    server.send(200, "text/plain; charset=utf-8",
                "✓ Home Assistant erreichbar, Token gültig.");
  } else if (code == 401) {
    server.send(401, "text/plain; charset=utf-8",
                "Home Assistant erreichbar, Token ungültig (401).");
  } else if (code > 0) {
    server.send(502, "text/plain; charset=utf-8",
                "Home Assistant antwortet mit HTTP " + String(code) + ".");
  } else {
    server.send(502, "text/plain; charset=utf-8",
                "Home Assistant nicht erreichbar (" + String(code) + ").");
  }
}

void handleLedTest() {
  if (!pixels) {
    server.send(500, "text/plain; charset=utf-8", "LED-Treiber nicht initialisiert.");
    return;
  }

  // short synchronous diagnostic sequence
  setAllLeds(180, 0, 0); delay(180);
  setAllLeds(0, 180, 0); delay(180);
  setAllLeds(0, 0, 180); delay(180);

  for (int i = 0; i < 7; i++) {
    pixels->clear();
    pixels->setPixelColor(i, pixels->Color(100, 40, 160));
    pixels->show();
    delay(70);
  }

  if (apMode) ledsStatusSetup();
  else ledsStatusReady();

  server.send(200, "text/plain; charset=utf-8", "LED-Test abgeschlossen.");
}

void handleAmpToggle() {
  bool target = !amplifierEnabled;

  if (!setAmplifier(target)) {
    server.send(500, "text/plain; charset=utf-8",
                "Verstärker konnte nicht geschaltet werden. TCA9555 nicht gefunden oder andere Board-Revision.");
    return;
  }

  server.send(200, "text/plain; charset=utf-8",
              target ? "Verstärker eingeschaltet." : "Verstärker ausgeschaltet.");
}

void handleSpeakerTest() {
  if (!audioI2sReady || !codecPlaybackReady) {
    server.send(500, "text/plain; charset=utf-8",
                "Audioausgabe ist noch nicht bereit.");
    return;
  }

  server.send(200, "text/plain; charset=utf-8", "Testton läuft.");
  playSpeakerTestTone();
}

void handleAssistWake() {
  if (wakeBusy) {
    server.send(409, "text/plain; charset=utf-8",
                "Assist läuft bereits.");
    return;
  }

  server.send(200, "text/plain; charset=utf-8",
              "Wake gestartet. Jetzt sprechen ...");
  runWakeCaptureAndHa();
}

void handleReboot() {
  server.send(200, "text/plain", "Restarting");
  scheduleRestart(500);
}

void handleFactoryReset() {
  clearConfig();
  server.send(200, "text/plain", "Factory reset");
  scheduleRestart(700);
}

void handleOtaUpload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    setAllLeds(0, 0, 160);

    Serial.printf("OTA: %s\n", upload.filename.c_str());

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    ledsStatusError();
  }
}

void handleOtaFinished() {
  if (Update.hasError()) {
    ledsStatusError();
    server.send(500, "text/plain; charset=utf-8",
                "Firmware-Update fehlgeschlagen.");
  } else {
    setAllLeds(0, 160, 30);
    server.send(200, "text/plain; charset=utf-8",
                "Firmware installiert. Neustart ...");
    scheduleRestart(1000);
  }
}

void redirectPortal() {
  server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

// -----------------------------------------------------------------------------
// Server
// -----------------------------------------------------------------------------

void setupWebServer() {
  server.on("/", HTTP_GET, sendIndex);

  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/ha/test", HTTP_POST, handleHaTest);

  server.on("/api/hardware/led-test", HTTP_POST, handleLedTest);
  server.on("/api/hardware/amp-toggle", HTTP_POST, handleAmpToggle);
  server.on("/api/audio/speaker-test", HTTP_POST, handleSpeakerTest);
  server.on("/api/assist/wake", HTTP_POST, handleAssistWake);

  server.on("/api/system/reboot", HTTP_POST, handleReboot);
  server.on("/api/system/factory-reset", HTTP_POST, handleFactoryReset);

  server.on("/api/ota", HTTP_POST, handleOtaFinished, handleOtaUpload);

  // Captive portal detection
  server.on("/generate_204", HTTP_ANY, redirectPortal);
  server.on("/gen_204", HTTP_ANY, redirectPortal);
  server.on("/hotspot-detect.html", HTTP_ANY, redirectPortal);
  server.on("/ncsi.txt", HTTP_ANY, redirectPortal);
  server.on("/connecttest.txt", HTTP_ANY, redirectPortal);
  server.on("/fwlink", HTTP_ANY, redirectPortal);

  server.onNotFound([]() {
    if (apMode)
      redirectPortal();
    else
      server.send(404, "text/plain", "404");
  });

  server.begin();
}

// -----------------------------------------------------------------------------
// Arduino
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(600);

  Serial.println();
  Serial.println("==============================================");
  Serial.printf(" VoiceDot Waveshare Firmware v%s\n", FW_VERSION);
  Serial.println("==============================================");

  Serial.printf("Flash size : %lu\n", (unsigned long)ESP.getFlashChipSize());
  Serial.printf("PSRAM size : %lu\n", (unsigned long)ESP.getPsramSize());
  Serial.printf("Free heap  : %lu\n", (unsigned long)ESP.getFreeHeap());

  loadConfig();

  // Detect the exact Waveshare audio hardware first.
  detectBoardProfile();
  setupButtons();

  // RGB ring
  setupLeds();
  ledsBootAnimation();

  // Keep amp muted during boot.
  if (tca9555Present) {
    setAmplifier(false);
  }

  setupAudio();

  bool wifiOk = connectToWiFi();

  if (!wifiOk) {
    startProvisioningAp();
    ledsStatusSetup();
  } else {
    apMode = false;
    startMdns();
    ledsStatusReady();
  }

  setupWebServer();

  Serial.println();
  Serial.println("VoiceDot ready.");

  if (apMode) {
    Serial.printf("Setup WiFi: %s\n", WiFi.softAPSSID().c_str());
    Serial.printf("Password  : %s\n", AP_PASSWORD);
    Serial.println("Open      : http://192.168.4.1");
  } else {
    Serial.printf("Open      : http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("or        : http://voicedot.local");
  }

  Serial.println();
  Serial.printf("Audio profile: %s\n", activeProfile.name);
  Serial.printf("I2C SDA/SCL : %d/%d\n", activeProfile.i2cSda, activeProfile.i2cScl);
  Serial.printf("I2S MCLK    : %d\n", activeProfile.i2sMclk);
  Serial.printf("I2S BCLK    : %d\n", activeProfile.i2sBclk);
  Serial.printf("I2S LRCLK   : %d\n", activeProfile.i2sLrclk);
  Serial.printf("MIC DATA    : %d\n", activeProfile.micData);
  Serial.printf("SPK DATA    : %d\n", activeProfile.speakerData);
  Serial.printf("RGB GPIO    : %d\n", activeProfile.ledPin);
}

void loop() {
  if (apMode) {
    dnsServer.processNextRequest();
  }

  server.handleClient();
  pollButtons();
  pollMicLevel();

  if (!apMode && WiFi.status() != WL_CONNECTED) {
    static uint32_t lastReconnect = 0;

    if (millis() - lastReconnect > 10000) {
      lastReconnect = millis();
      WiFi.reconnect();
    }
  }

  if (restartRequested &&
      (int32_t)(millis() - restartAt) >= 0) {
    ESP.restart();
  }

  delay(2);
}
