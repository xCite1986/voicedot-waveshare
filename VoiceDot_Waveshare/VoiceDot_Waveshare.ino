/*
  VoiceDot for Waveshare ESP32-S3-AUDIO-Board
  Firmware v0.7.0

  Target:
    Waveshare ESP32-S3-AUDIO-Board
    ESP32-S3R8, 16MB Flash, 8MB PSRAM
    ES8311 audio codec
    ES7210 microphone ADC
    TCA9555 I/O expander
    7x WS2812 RGB LEDs

  Feature set:
      - WiFi provisioning + captive portal
      - Web UI with live status polling
      - NVS config, live apply without reboot where possible
      - OTA firmware update
      - Home Assistant REST connectivity test (http and https)
      - automatic hardware revision / I2C bus detection
      - ES8311 / ES7210 / TCA9555 detection
      - RGB status ring with listen / think / speak / error phases
      - amplifier enable via TCA9555 when supported
      - hardware diagnostics
      - onboard buttons for volume, mute, and LED brightness
      - ES8311/ES7210 codec init
      - I2S microphone level meter
      - speaker test tone
      - local wake word via Espressif WakeNet ("Hi ESP"), hands free
      - K2 short press starts an Assist turn, K2 long press toggles mute
      - energy based voice activity detection with pre-roll buffer,
        so a turn ends when the speaker stops instead of after a fixed time
      - Home Assistant Assist pipeline over WebSocket, optional pipeline id
      - conversation_id continuity and automatic follow-up turns
      - Home Assistant TTS WAV/PCM and MP3 playback over ES8311
      - all long running jobs are driven from loop(), so the web server
        is never re-entered from inside one of its own handlers
      - ESP-IDF I2S STD driver

  Arduino IDE dependency:
    - Adafruit NeoPixel
    - ESP_SR (bundled with the ESP32 Arduino core, ESP32-S3 only)
    - local mp3_decoder.cpp / mp3_decoder.h files in this sketch folder

  Board:
    ESP32S3 Dev Module
    Flash Size: 16MB
    PSRAM: OPI PSRAM
    USB CDC On Boot: Enabled
    Partition scheme: "ESP SR 16M (3MB APP/6MB SPIFFS/3.9MB MODEL)"

  The wake word models live in their own "model" flash partition, which only
  that partition scheme provides. Build with any other 16MB OTA scheme and
  everything still works, the detector simply reports that the partition is
  missing and stays off. Set VOICEDOT_WAKEWORD to 0 to drop ESP-SR entirely.
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
#include <LittleFS.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>

// Local wake word. Set to 0 to build without ESP-SR, without the ~3.3 MB model
// blob and without the special partition scheme.
#define VOICEDOT_WAKEWORD 1

#if VOICEDOT_WAKEWORD
// Deliberately not <ESP_SR.h>: that wrapper always grabs the first WakeNet it
// finds and drags MultiNet in with it. The esp-sr libraries are linked
// unconditionally by the core, so the low level API is available directly and
// lets us pick the model at runtime.
#include <esp_afe_config.h>
#include <esp_afe_sr_iface.h>
#include <esp_afe_sr_models.h>
#include <esp_wn_iface.h>
#include <esp_wn_models.h>
#include <model_path.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#endif

#include <stdarg.h>
#include <ctype.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Firmware
// -----------------------------------------------------------------------------

static const char* FW_VERSION = "0.13.2";
static const char* DEFAULT_HOSTNAME = "voicedot";
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

static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;
static constexpr uint32_t AUDIO_MCLK_HZ = 12288000;
static constexpr uint16_t AUDIO_FRAME_SAMPLES = 128;
static constexpr uint16_t K2_LONG_PRESS_MS = 850;
static constexpr uint16_t WAKE_COOLDOWN_MS = 900;
static constexpr uint16_t TTS_AMP_WARMUP_MS = 260;
static constexpr uint8_t TTS_PREROLL_BLOCKS = 8;
static constexpr uint8_t TTS_TAIL_BLOCKS = 6;
static constexpr uint16_t TTS_AMP_HOLD_MS = 90;
// Largest WebSocket payload we keep. Anything beyond is still read off the
// socket - the stream would desynchronise otherwise - but only the first part
// is parsed. Dropping the whole frame, as before, silently lost events.
static constexpr size_t WS_FRAME_KEEP_MAX = 12288;

static constexpr uint32_t TTS_MP3_TIMEOUT_MS = 60000;
static constexpr uint16_t TTS_MP3_IDLE_DONE_MS = 3200;
static constexpr uint16_t TTS_MP3_STALL_LOG_MS = 2500;
static constexpr size_t TTS_MP3_READ_CHUNK_MAX = 1024;

// Spoken acknowledgement played right after the wake word, so the user knows
// the device is listening before they start talking. The clips are rendered
// once by the user's own Home Assistant TTS and cached in LittleFS, which
// keeps the voice consistent with the assistant's answers.
// Playback speed. The device cannot ask Home Assistant to talk faster, but it
// can clock the decoded audio out faster. That shifts the pitch up as well,
// which is why the range stays modest.
static constexpr uint8_t TTS_SPEED_MIN = 75;
static constexpr uint8_t TTS_SPEED_MAX = 135;

// Day/night profile. Austria and Germany share this rule, and it carries the
// DST switch dates so the board does not need a timezone database.
static const char *DEFAULT_TIMEZONE = "CET-1CEST,M3.5.0,M10.5.0/3";

static constexpr uint8_t PIPELINE_MAX = 8;

// User uploaded sound effects, played from an announcement by writing
// "[dingdong.mp3] Es hat geläutet." They live next to the acknowledgement
// clips in LittleFS, in their own directory so the two never collide.
// Several VoiceDots in one flat hear the same wake word. They agree over UDP
// broadcast which of them heard it loudest; that one runs the dialogue, the
// others go back to sleep without recording anything.
static constexpr uint16_t MULTI_PORT = 4210;
static constexpr uint16_t MULTI_HELLO_INTERVAL_MS = 30000;
static constexpr uint32_t MULTI_PEER_TTL_MS = 180000;
static constexpr uint8_t MULTI_MAX_PEERS = 8;

// A claim that arrived shortly before our own detection still belongs to the
// same event - the devices never trigger at exactly the same instant.
static constexpr uint32_t MULTI_CLAIM_LOOKBACK_MS = 600;

// Equal loudness has to resolve the same way on every device, so the smaller
// id wins. This is the margin below which two scores count as equal.
static constexpr float MULTI_SCORE_TIE_DB = 0.5f;

// Firmware updates come from the releases of this repository.
static const char *UPDATE_REPO = "xCite1986/voicedot-waveshare";
static constexpr uint8_t UPDATE_MAX_RELEASES = 8;
static constexpr uint32_t UPDATE_STALL_TIMEOUT_MS = 20000;
static constexpr uint32_t UPDATE_AUTO_CHECK_MS = 12UL * 60UL * 60UL * 1000UL;
static constexpr uint32_t UPDATE_FIRST_CHECK_MS = 45000;

// Room noise at or below this counts as quiet and earns no boost. Measured on
// a real device a silent living room sits around -74 dBFS, a running 3D printer
// well above -55 dBFS.
static constexpr float AUTO_VOLUME_QUIET_DB = -65.0f;

// The codec's volume register moves in half-decibel steps.
static constexpr float ES8311_DB_PER_STEP = 0.5f;

// Web radio. The output is fixed at 16 kHz so the shared I2S clock leaves the
// wake word detector at the rate it needs - see the radio section for why.
static constexpr uint32_t RADIO_OUTPUT_RATE = 16000;
static constexpr size_t RADIO_IN_BUFFER_SIZE = 32768;
static constexpr uint8_t RADIO_MAX_STATIONS = 20;
static constexpr uint8_t RADIO_FRAMES_PER_TICK = 4;
static constexpr uint32_t RADIO_WRITE_TIMEOUT_MS = 8;
static constexpr uint32_t RADIO_STALL_TIMEOUT_MS = 15000;
static constexpr uint32_t RADIO_RECONNECT_DELAY_MS = 3000;
static constexpr uint8_t RADIO_MAX_RECONNECTS = 5;
static const char *RADIO_STATION_FILE = "/radio.txt";

// Named groups of Home Assistant entities, switched in one service call.
static constexpr uint8_t GROUP_MAX = 16;
static constexpr uint16_t HA_ENTITY_LIMIT = 40;
static const char *GROUP_FILE = "/groups.txt";

// A TLS session costs about 43 kB of internal RAM on this chip. Measured with
// an https:// stream running, the web interface still answered - after 25
// seconds. Below this much free heap the stream is refused instead.
// How long an answer to "which station?" still counts as an answer.
static constexpr uint32_t RADIO_ANSWER_WINDOW_MS = 30000;

static constexpr uint32_t RADIO_TLS_MIN_HEAP = 20000;

// What one costs, measured: 52 kB free before, 8.8 kB after. Attempting a
// handshake that cannot leave enough room is worse than useless - a failing one
// took 60 seconds, and the web interface was starved for every one of them.
static constexpr uint32_t RADIO_TLS_HEAP_COST = 43000;

// The release list and the firmware itself both come from GitHub, so both need
// TLS. Below this much free heap the wake word detector is shut down for the
// duration of the transfer. Measured on the device: 49 kB free with the
// detector running, 123 kB without it.
static constexpr uint32_t UPDATE_TLS_MIN_HEAP = 90000;

static const char *SOUND_DIR = "/snd";
static constexpr size_t SOUND_MAX_BYTES = 512 * 1024;
static constexpr uint8_t SOUND_MAX_FILES = 20;

static constexpr uint8_t ACK_MAX_CLIPS = 8;
static constexpr size_t ACK_MAX_CLIP_BYTES = 96 * 1024;
static const char *ACK_DEFAULT_PHRASES =
  "Ja bitte|Was gibt's?|Was kann ich für dich tun?|Jawohl|Ich höre";
static constexpr size_t MP3_INPUT_BUFFER_SIZE = 8192;
static constexpr size_t MP3_PCM_MAX_SAMPLES = 1152 * 2;
static constexpr bool DEBUG_SERIAL = true;

// Recording window.
//
// WAKE_RECORD_MS is the fixed window used when voice activity detection is
// switched off. With VAD enabled the recorder waits for speech, then stops
// after WAKE_SILENCE_MS of trailing silence, bounded by min/max length.
static constexpr uint32_t WAKE_RECORD_MS = 3500;
// 9 s used to cut off longer questions. PSRAM is not the constraint here -
// 20 s of 16 kHz mono is 640 kB out of the 7 MB that sit unused.
static constexpr uint32_t WAKE_MAX_RECORD_MS = 20000;
static constexpr uint32_t WAKE_MIN_RECORD_MS = 700;
static constexpr uint32_t WAKE_SPEECH_TIMEOUT_MS = 4500;
static constexpr uint32_t WAKE_SILENCE_MS = 1400;
static constexpr uint32_t WAKE_PREROLL_MS = 320;
static constexpr uint32_t WAKE_MAX_BYTES = (AUDIO_SAMPLE_RATE * 2 * WAKE_MAX_RECORD_MS) / 1000;
static constexpr uint32_t WAKE_PREROLL_BYTES = (AUDIO_SAMPLE_RATE * 2 * WAKE_PREROLL_MS) / 1000;

// Voice activity detection. The noise floor is tracked while the recorder is
// waiting for speech, so a noisy room raises the threshold instead of
// triggering immediately.
static constexpr float VAD_SPEECH_MARGIN_DB = 9.0f;
static constexpr float VAD_RELEASE_MARGIN_DB = 3.0f;

// While someone is talking, anything within this many dB of their loudest
// frame still counts as speech. Quiet syllables and trailing consonants sit
// far below the peak but well above the room.
static constexpr float VAD_SPEECH_DROP_DB = 22.0f;

// What one frame above the release threshold costs in accumulated silence.
// Resetting the silence clock outright was the old behaviour, and in a room
// whose own noise straddles that threshold it never got past 50 ms of a needed
// 1400. Speech produces frames in runs and still drains the clock in a few
// hundred milliseconds; a lone blip only sets it back this far.
static constexpr uint32_t VAD_BLIP_CREDIT_MS = 120;
static constexpr float VAD_MIN_SPEECH_DB = -52.0f;
static constexpr float VAD_NOISE_FLOOR_START_DB = -70.0f;
static constexpr float VAD_NOISE_FLOOR_MAX_DB = -30.0f;
static constexpr uint32_t VAD_CALIBRATE_MS = 180;

// Wake word.
//
// Nothing here is tied to a particular wake word. The list offered in the web
// UI is built from whatever WakeNet models the "model" partition actually
// holds, so reflashing srmodels.bin is enough to change the choices.
static constexpr uint8_t SR_MAX_MODELS = 8;

// Our I2S RX stream carries the two ES7210 microphone channels.
static constexpr uint8_t SR_RX_CHANNELS = 2;

// Two microphone channels, matching the ES7210 stereo stream.
static const char *WAKE_WORD_INPUT_FORMAT = "MM";

// How long srStopEngine() waits for the detector tasks to leave their loops.
static constexpr uint32_t SR_TASK_STOP_TIMEOUT_MS = 2500;

// Upper bound for the feed task to confirm it let go of the microphone.
static constexpr uint16_t SR_PAUSE_SETTLE_MS = 200;

// Safety net: if something left the detector paused, re-arm it.
static constexpr uint32_t SR_STUCK_RESUME_MS = 5000;

// The detector listens to the room all the time, so its noise estimate can seed
// the recorder. Older than this and we measure again instead.
static constexpr uint32_t SR_ROOM_NOISE_TTL_MS = 30000;

// Follow-up turns triggered by continue_conversation should start quickly.
static constexpr uint16_t FOLLOW_UP_COOLDOWN_MS = 350;
static constexpr uint32_t CONVERSATION_TTL_MS = 300000;
static constexpr int ERR_MP3_NONE = 0;
static constexpr int ERR_MP3_INDATA_UNDERFLOW = -1;
static constexpr int ERR_MP3_MAINDATA_UNDERFLOW = -2;

bool MP3Decoder_AllocateBuffers(void);
void MP3Decoder_FreeBuffers();
int MP3Decode(unsigned char *inbuf, int *bytesLeft, short *outbuf, int useSize);
int MP3FindSyncWord(unsigned char *buf, int nBytes);
int MP3GetSampRate();
int MP3GetChannels();
int MP3GetOutputSamps();

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

// Byte order of the WS2812 ring on this board. Measured, not assumed: sending
// Color(0, 90, 0) with NEO_GRB puts 90 on the wire first and the ring lit up
// red, so the first byte is the red channel.
static constexpr neoPixelType LED_COLOR_ORDER = NEO_RGB + NEO_KHZ800;

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
bool ttsPlaybackActive = false;
i2s_chan_handle_t audioTxChan = nullptr;
i2s_chan_handle_t audioRxChan = nullptr;
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
uint32_t wakeIgnoreUntil = 0;
uint32_t wakeLastDurationMs = 0;
uint32_t wakeLastBytes = 0;
uint32_t wakeLastHttpCode = 0;
String wakeLastState = "idle";
String wakeLastMessage = "Bereit.";
String wakeLastWsStage = "-";
String wakeLastWsDetail = "-";
String wakeTranscript = "";
String wakeAssistantText = "";
String wakeTtsUrl = "";
String wakeTtsStatus = "-";
String haConversationId = "";
uint32_t haConversationSeenAt = 0;
bool haContinueConversation = false;
bool localCommandHandled = false;
bool wakeRequested = false;
const char *wakeRequestSource = "-";

// Wake word detector state. These stay defined even when VOICEDOT_WAKEWORD is
// 0, so the status API and the web UI do not need their own #ifdefs.
bool srAvailable = false;
bool srRunning = false;
bool srPaused = false;
volatile bool srWakeFlag = false;
bool srApplyRequested = false;
uint32_t srDetections = 0;
uint32_t srLastDetectMs = 0;
uint32_t srPausedAt = 0;
float srRoomNoiseDb = VAD_NOISE_FLOOR_START_DB;
uint32_t srRoomNoiseAt = 0;
bool srRoomNoiseValid = false;
String srStatus = "nicht gestartet";

// Wake word models found in the partition, and the one currently loaded.
// Assist pipelines as reported by Home Assistant. Each pipeline carries its own
// TTS engine and voice, so switching pipeline is how the voice is changed.
String pipelineId[PIPELINE_MAX];
String pipelineName[PIPELINE_MAX];
String pipelineVoice[PIPELINE_MAX];
uint8_t pipelineCount = 0;
String pipelinePreferred = "";
bool pipelineListRequested = false;
String pipelineListStatus = "noch nicht abgerufen";

String haTtsLanguage = "";
String haTtsVoice = "";

// Peers and the state of one arbitration round.
struct VoiceDotPeer {
  String id;
  String name;
  String ip;
  uint32_t lastSeen = 0;
  float lastScore = -99.0f;
  bool lastWon = false;
};

WiFiUDP multiUdp;
bool multiReady = false;
VoiceDotPeer multiPeers[MULTI_MAX_PEERS];
uint8_t multiPeerCount = 0;
String multiDeviceId = "";
String multiLastDecision = "-";

bool arbActive = false;
uint32_t arbStartedAt = 0;
float arbBestScore = -99.0f;
String arbBestId = "";
bool arbLost = false;

// A claim seen just before our own detection still counts.
String arbLastClaimId = "";
float arbLastClaimScore = -99.0f;
uint32_t arbLastClaimAt = 0;

// Loudness of the wake word: peak level over the last ~1.5 s, held by the
// detector's meter so it is already known the moment the word is recognised.
float srRecentPeakDb = -96.0f;

// Cached release list. Everything else reads this instead of asking GitHub, so
// a polling Home Assistant cannot exhaust the API budget.
String updateTags[8];
String updateUrls[8];
uint8_t updateReleaseCount = 0;
String updateStatus = "noch nicht geprueft";
uint32_t updateCheckedAt = 0;
bool updateChecked = false;
bool updateInProgress = false;
int updateProgress = -1;
uint32_t updateRebootAt = 0;

// The room noise the boost is based on. Kept apart from srRoomNoiseDb because
// that one keeps tracking while the radio plays, and a loudspeaker measuring
// itself would only wind itself up.
float autoVolumeNoiseDb = VAD_NOISE_FLOOR_START_DB;
bool autoVolumeNoiseValid = false;
uint32_t autoVolumeNoiseAt = 0;
float autoVolumeLastBoost = 0.0f;

// Radio state. Plain globals rather than a struct: Arduino puts its generated
// prototypes above the type definitions, so a struct in a signature would not
// compile.
bool radioActive = false;
bool radioPausedForTurn = false;
String radioStationName = "";
String radioStationUrl = "";
String radioStatus = "aus";
uint32_t radioStartedAt = 0;
uint32_t radioBytesIn = 0;
uint32_t radioLastDataMs = 0;
uint32_t radioRetryAt = 0;
uint32_t radioUnderruns = 0;
uint16_t radioSampleRate = 0;
uint8_t radioChannels = 0;
uint8_t radioReconnects = 0;
bool radioUsingTls = false;

String groupNames[16];
String groupEntities[16];
uint8_t groupCount = 0;

// Alarm and timer. The alarm survives a reboot in the config, the timer does
// not - a countdown that resumes after a restart is a surprise, not a feature.
int alarmFiredDay = -1;
bool alarmRinging = false;
time_t timerEndsAt = 0;
uint32_t timerTotalSec = 0;

// A locally handled command can have something to say. Spoken instead of the
// acknowledgement chime, so "noch zwei Minuten" reaches the room.
String wakeAnnounceText = "";

// Set by the web interface, run from the loop: the agent needs up to a minute
// and no request handler should sit there that long.
String briefingTestRequest = "";

// A "spiele ..." we could not resolve turns into a question of our own.
bool radioAskPending = false;
String radioAskText = "";
uint32_t radioAnswerUntil = 0;

HTTPClient *radioHttp = nullptr;
WiFiClient *radioPlain = nullptr;
WiFiClientSecure *radioSecure = nullptr;
Client *radioStream = nullptr;

uint8_t *radioInBuf = nullptr;
size_t radioInLen = 0;
int16_t *radioPcm = nullptr;
int16_t *radioOut = nullptr;
size_t radioOutFrames = 0;
size_t radioOutOffset = 0;

// Box-filter decimator down to RADIO_OUTPUT_RATE.
uint32_t radioResampRate = 0;
uint32_t radioResampStep = 0;
uint32_t radioResampPhase = 0;
int32_t radioAccL = 0;
int32_t radioAccR = 0;
uint32_t radioAccN = 0;

String radioStationNames[RADIO_MAX_STATIONS];
String radioStationUrls[RADIO_MAX_STATIONS];
uint8_t radioStationCount = 0;

String haPublishStatus = "aus";
String haEntityId = "";
uint32_t haPublishAt = 0;

bool timeValid = false;
int8_t scheduleApplied = -1;  // -1 unknown, 0 day, 1 night

File soundUploadFile;
String soundUploadName = "";
String soundUploadStatus = "-";
size_t soundUploadBytes = 0;
bool soundUploadFailed = false;

String soundPlayRequest = "";

String announceText = "";
String announceStatus = "-";
bool announceRequested = false;

bool ackFsReady = false;
uint8_t ackClipCount = 0;
bool ackBuildRequested = false;
bool ackTestRequested = false;
String ackBuildStatus = "noch nicht erzeugt";
String haTtsEngine = "";

String srModelId[SR_MAX_MODELS];
String srModelWords[SR_MAX_MODELS];
uint8_t srModelCount = 0;
String srActiveModel = "";
String srActiveWords = "";
bool speakerTestRequested = false;
uint32_t wakeLastVadSilenceMs = 0;
float wakeNoiseFloorDb = VAD_NOISE_FLOOR_START_DB;
uint32_t diagSeq = 0;
uint32_t ttsOutputBytes = 0;
uint32_t ttsWriteFailures = 0;

// State of one streamed Assist turn. Defined up here because the Arduino
// preprocessor inserts generated prototypes above the first function in the
// file, and those prototypes already mention this type.
struct AssistStream {
  Client *client = nullptr;
  int handlerId = -1;
  uint8_t frame[1025];   // [0] = handler id, the rest is PCM
  size_t pending = 0;
  bool failed = false;
};

// -----------------------------------------------------------------------------
// Serial log mirror
//
// The same lines that go out over USB are kept in a ring buffer so the web UI
// can show them. Fixed char buffers on purpose: uiLog() runs from the detector
// tasks as well, and a critical section must not allocate.
// -----------------------------------------------------------------------------

static constexpr uint16_t LOG_LINES = 200;
static constexpr uint8_t LOG_LINE_LEN = 160;

char logRing[LOG_LINES][LOG_LINE_LEN];
uint16_t logHead = 0;
uint32_t logTotal = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void uiLog(const char *line) {
  portENTER_CRITICAL(&logMux);
  strncpy(logRing[logHead], line, LOG_LINE_LEN - 1);
  logRing[logHead][LOG_LINE_LEN - 1] = '\0';
  logHead = (uint16_t)((logHead + 1) % LOG_LINES);
  logTotal++;
  portEXIT_CRITICAL(&logMux);
}

// Prints to the serial console and into the web log in one go.
void logPrintf(const char *fmt, ...) {
  char buf[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  Serial.println(buf);
  uiLog(buf);
}

void diagLog(const char *tag, const String &message) {
  // The web log gets a compact line, the serial console keeps the wide format.
  unsigned long ms = millis();
  char line[LOG_LINE_LEN];
  snprintf(line, sizeof(line), "[%lu.%03lu] %-9s %s",
           ms / 1000UL, ms % 1000UL, tag, message.c_str());
  uiLog(line);

  if (!DEBUG_SERIAL) return;
  Serial.printf("[%lu #%lu] %s: %s | wake=%s rec=%u send=%u tts=%u heap=%lu\n",
                (unsigned long)millis(),
                (unsigned long)++diagSeq,
                tag,
                message.c_str(),
                wakeLastState.c_str(),
                wakeRecording ? 1 : 0,
                wakeSending ? 1 : 0,
                ttsPlaybackActive ? 1 : 0,
                (unsigned long)ESP.getFreeHeap());
}

void diagLogf(const char *tag, const char *fmt, ...) {
  char buf[224];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  diagLog(tag, String(buf));
}

bool k2RawPressed = false;
bool k2StablePressed = false;
bool k2LongHandled = false;
uint32_t k2ChangedAt = 0;
uint32_t k2PressedAt = 0;

Adafruit_NeoPixel* pixels = nullptr;
uint8_t ledBrightness = 32;
uint8_t waitRingPosition = 0;
uint32_t waitRingLastMs = 0;

// The ring shows what the device is doing right now.
enum LedPhase {
  LED_PHASE_IDLE,
  LED_PHASE_LISTEN,
  LED_PHASE_THINK,
  LED_PHASE_SPEAK,
  LED_PHASE_RADIO,
  LED_PHASE_YIELD,
  LED_PHASE_ERROR
};

// Losing the arbitration to a louder VoiceDot: two blinks, then dark, then the
// idle dot again - a visible "you are talking to the other one".
static constexpr uint32_t LED_YIELD_BLINK_MS = 200;
static constexpr uint32_t LED_YIELD_GAP_MS = 120;
static constexpr uint32_t LED_YIELD_TOTAL_MS = 780;

LedPhase ledPhase = LED_PHASE_IDLE;
uint32_t ledPhaseSince = 0;
uint8_t ledPulseStep = 0;

// Level meter ballistics for the listening ring.
float eqLevel = 0.0f;

struct ButtonState {
  bool rawPressed;
  bool stablePressed;
  bool lastStablePressed;
  uint32_t changedAt;
};

bool updateButton(ButtonState &button, bool pressedNow);

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
  String haPipeline;   // optional Assist pipeline id, empty = HA default
  uint8_t volume;
  uint8_t ledBrightness;
  bool vadEnabled;     // stop recording when the speaker stops
  bool followUp;       // start another turn on continue_conversation
  bool wakeWord;       // local WakeNet detection
  String wakeWordModel; // WakeNet model id, empty = first one in the partition
  bool ackEnabled;      // speak a short confirmation after the wake word
  String ackPhrases;    // pipe separated, one clip per phrase
  uint32_t listenColor; // ring colour of the level meter, 0xRRGGBB
  uint32_t speakColor;  // ring colour while the assistant talks, 0xRRGGBB
  uint8_t ttsSpeed;     // playback speed in percent, 100 = as delivered
  uint8_t vadReleaseDb; // dB above the background that still counts as speech
  uint16_t vadSilenceMs;// trailing silence that ends the recording

  bool scheduleEnabled;   // apply a day and a night profile
  uint16_t dayStartMin;   // minutes since midnight
  uint16_t nightStartMin;
  uint8_t dayVolumeStep;  // 0..10, same scale as the spoken command
  uint8_t dayBrightness;  // percent
  uint8_t nightVolumeStep;
  uint8_t nightBrightness;
  String timezone;
  bool haPublish;       // push our state into Home Assistant
  bool cleanMarkdown;   // re-render answers without Markdown before speaking
  int16_t alarmMinutes;       // minutes since midnight, -1 = no alarm
  bool alarmDaily;            // ring again the next day
  String alarmSound;          // file from the sound library
  String alarmBriefing;       // spoken after the sound
  String timerSound;          // file for the timer
  uint8_t micGainDb;          // ES7210 analogue gain, 0..36 dB
  bool sttHaVad;              // let Home Assistant end the sentence too
  uint8_t sttNoiseSuppression; // 0..4, Home Assistant side, 0 = off
  uint8_t sttAutoGainDb;      // 0..31 dBFS of automatic gain, 0 = off
  uint16_t sttVolumePercent;  // PCM multiplier in percent, 100 = unchanged
  uint8_t volumeStep;      // what "leiser" and "lauter" move by
  bool updateCheckEnabled; // ask GitHub for newer releases on its own
  bool autoVolumeEnabled;  // lift the voice when the room gets loud
  uint8_t autoVolumeMaxDb; // how far it may lift it
  bool multiEnabled;    // arbitrate the wake word with other VoiceDots
  uint16_t multiWindowMs; // how long to wait for competing claims
};

Config cfg;

void waitRingTick(uint32_t now, bool force);
void ledTick(bool force = false);
void setLedPhase(LedPhase phase);
void ledsStatusIdle();
void ledsStatusReady();
void ledsStatusSetup();
void ledsStatusError();
void setAllLeds(uint8_t r, uint8_t g, uint8_t b);
void requestWake(const char *source);
void setVolume(uint8_t volume);
static String minutesToHhMm(uint16_t minutes);
static uint16_t hhMmToMinutes(const String &value, uint16_t fallback);
static void applySchedule(bool force);
bool playAckSound();
bool playSoundFile(const String &name);
static String multiOwnId();
static float multiWakeScore();
static bool radioHandleVoiceCommand(const String &lower);
static bool alarmTimerVoiceCommand(const String &lower);
static bool groupsVoiceCommand(const String &lower);
static void groupsLoad();
void alarmTimerTick();
static String clockText(int minutes);
static long alarmSecondsUntil();
static long timerSecondsLeft();
bool updateFetchReleases();
static int updateNewestIndex();
static bool updateIsNewer(const String &tag);
static int updateVersionCompare(const String &a, const String &b);
static void updateDownloadAndWrite(const String &tag, const String &url);
static bool updateHasPending();
static void updateRunPending();
static void updateLoadResult();
static void radioLoadStations();
void radioStop(const char *why);
bool radioStart(const String &name, const String &url);
void radioTick();
void radioPauseForTurn();
void radioResumeAfterTurn();
uint8_t ackScanClips();
String stripMarkdownForSpeech(const String &in);

bool wakeCanStart();
bool srBegin();
void srPauseDetection();
void srResumeDetection();
void srApplyConfig();

// -----------------------------------------------------------------------------
// Cooperative service pump
//
// Recording, the Assist WebSocket exchange and TTS playback all run to
// completion inside loop(). They call pumpServices() so the web UI stays
// responsive. The guard makes the call a no-op when we are already inside
// WebServer::handleClient(), because that class is not re-entrant.
// -----------------------------------------------------------------------------

bool servicePumpActive = false;

void pumpServices() {
  if (!servicePumpActive) {
    servicePumpActive = true;
    if (apMode) dnsServer.processNextRequest();
    server.handleClient();
    servicePumpActive = false;
  }
  yield();
}

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------

// A websocket text frame must carry valid UTF-8; a server is required to close
// the connection when it does not. That is exactly what happened to the first
// briefing: a single Latin-1 "u-umlaut" byte from a careless client killed the
// run 65 milliseconds in, with no error event to explain it.
//
// Rather than dropping the bad byte, it is read as Latin-1 and re-encoded -
// which repairs the usual cause instead of mutilating the text.
String sanitizeUtf8(const String &in) {
  String out;
  out.reserve(in.length() + 8);

  size_t i = 0;
  while (i < in.length()) {
    uint8_t c = (uint8_t)in[i];

    if (c < 0x80) { out += (char)c; i++; continue; }

    uint8_t need = 0;
    if ((c & 0xE0) == 0xC0) need = 1;
    else if ((c & 0xF0) == 0xE0) need = 2;
    else if ((c & 0xF8) == 0xF0) need = 3;

    bool valid = need > 0 && (i + need) < in.length() + 0;
    if (valid) {
      for (uint8_t k = 1; k <= need; k++) {
        if (((uint8_t)in[i + k] & 0xC0) != 0x80) { valid = false; break; }
      }
    }

    if (valid) {
      for (uint8_t k = 0; k <= need; k++) out += in[i + k];
      i += need + 1;
      continue;
    }

    // Not valid UTF-8, so treat the byte as Latin-1 and encode it properly.
    out += (char)(0xC0 | (c >> 6));
    out += (char)(0x80 | (c & 0x3F));
    i++;
  }

  return out;
}

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

String jsonFindStringFrom(const String &json, const String &key, int from) {
  int k = json.indexOf("\"" + key + "\"", from);
  if (k < 0) return "";

  int colon = json.indexOf(':', k);
  if (colon < 0) return "";

  // Only accept an actual string value. null, numbers and objects must not be
  // mistaken for the next key that happens to follow.
  int i = colon + 1;
  while (i < (int)json.length() && isspace((unsigned char)json[i])) i++;
  if (i >= (int)json.length() || json[i] != '"') return "";
  int quote = i;

  String out;
  for (int i = quote + 1; i < (int)json.length(); i++) {
    char c = json[i];

    if (c == '"') break;

    if (c != '\\') {
      out += c;
      continue;
    }

    if (++i >= (int)json.length()) break;
    char e = json[i];

    switch (e) {
      case 'n': out += '\n'; break;
      case 'r': out += '\r'; break;
      case 't': out += '\t'; break;
      case 'b': out += '\b'; break;
      case 'f': out += '\f'; break;
      case '"': out += '"'; break;
      case '/': out += '/'; break;
      case '\\': out += '\\'; break;
      case 'u': {
        // \uXXXX -> UTF-8. Surrogate pairs are rare in these payloads and are
        // passed through as the replacement character rather than guessed at.
        if (i + 4 >= (int)json.length()) { i = json.length(); break; }
        uint16_t cp = 0;
        bool okHex = true;
        for (int k = 1; k <= 4; k++) {
          char h = json[i + k];
          cp <<= 4;
          if (h >= '0' && h <= '9') cp |= (uint16_t)(h - '0');
          else if (h >= 'a' && h <= 'f') cp |= (uint16_t)(h - 'a' + 10);
          else if (h >= 'A' && h <= 'F') cp |= (uint16_t)(h - 'A' + 10);
          else { okHex = false; break; }
        }
        i += 4;
        if (!okHex) break;

        if (cp < 0x80) {
          out += (char)cp;
        } else if (cp < 0x800) {
          out += (char)(0xC0 | (cp >> 6));
          out += (char)(0x80 | (cp & 0x3F));
        } else if (cp >= 0xD800 && cp <= 0xDFFF) {
          out += '?';
        } else {
          out += (char)(0xE0 | (cp >> 12));
          out += (char)(0x80 | ((cp >> 6) & 0x3F));
          out += (char)(0x80 | (cp & 0x3F));
        }
        break;
      }
      default: out += e; break;
    }
  }

  return out;
}

String jsonFindString(const String &json, const String &key) {
  return jsonFindStringFrom(json, key, 0);
}

String jsonFindLastString(const String &json, const String &key) {
  String out;
  int from = 0;
  while (true) {
    int k = json.indexOf("\"" + key + "\"", from);
    if (k < 0) break;
    String candidate = jsonFindStringFrom(json, key, k);
    if (candidate.length() > 0) out = candidate;
    from = k + key.length() + 2;
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

bool jsonFindBool(const String &json, const String &key, bool fallback) {
  int k = json.indexOf("\"" + key + "\"");
  if (k < 0) return fallback;

  int colon = json.indexOf(':', k);
  if (colon < 0) return fallback;

  int i = colon + 1;
  while (i < (int)json.length() && isspace((unsigned char)json[i])) i++;
  if (json.startsWith("true", i)) return true;
  if (json.startsWith("false", i)) return false;
  return fallback;
}

// Spoken step 0..10 -> 0..100 %. Digits and German number words both appear
// depending on the speech-to-text engine, so accept either.
static int spokenVolumeStep(const String &lower) {
  static const char *words[] = {
    "null", "eins", "zwei", "drei", "vier",
    "f\xc3\xbcnf", "sechs", "sieben", "acht", "neun", "zehn"
  };

  for (int i = 10; i >= 0; i--) {
    if (lower.indexOf(words[i]) >= 0) return i;
  }

  // "10" has to win over "1", so scan for the two digit form first.
  if (lower.indexOf("10") >= 0) return 10;
  for (int i = 0; i <= 9; i++) {
    if (lower.indexOf(String(i)) >= 0) return i;
  }

  return -1;
}

// "Leiser" and "lauter" without a number, moving by a step the user picks.
// Deliberately only for very short utterances: "mach mal die Musik leiser im
// Wohnzimmer" belongs to the assistant, not to this speaker.
static bool volumeStepVoiceCommand(const String &lower) {
  uint8_t words = 1;
  for (size_t i = 0; i < lower.length(); i++) {
    if (lower[i] == ' ') words++;
  }
  if (words > 4 || lower.length() > 30) return false;

  bool down = lower.indexOf("leiser") >= 0;
  bool up = lower.indexOf("lauter") >= 0;
  if (down == up) return false;  // neither, or a sentence with both

  int target = (int)cfg.volume + (up ? (int)cfg.volumeStep : -(int)cfg.volumeStep);
  target = constrain(target, 0, 100);

  setVolume((uint8_t)target);
  wakeLastMessage = String(up ? "Lauter" : "Leiser") + ": Lautstaerke " +
                    String(target) + " %";
  diagLogf("LOCAL_CMD", "volume %s by %u to %d%% from \"%s\"",
           up ? "up" : "down", cfg.volumeStep, target, lower.c_str());
  return true;
}

// Some commands are ours, not Home Assistant's: the volume of this speaker is
// a local property, and routing it through an LLM would be slow and unreliable.
// Returns true when the transcript was consumed here.
static bool handleLocalVoiceCommand(const String &text) {
  String lower = text;
  lower.toLowerCase();
  lower.trim();

  // Drop leading punctuation so "Lautstärke 5." and ", lautstärke 5" both work.
  while (lower.length() > 0 && !isalnum((unsigned char)lower[0])) lower.remove(0, 1);

  // Radio is ours as well: the station list lives on this device, so resolving
  // a name here saves a round trip through a language model that cannot play
  // music anyway.
  if (radioHandleVoiceCommand(lower)) return true;
  if (volumeStepVoiceCommand(lower)) return true;
  if (alarmTimerVoiceCommand(lower)) return true;
  if (groupsVoiceCommand(lower)) return true;

  // The command has to *be* the sentence, not appear somewhere inside it.
  // Otherwise "stell die Lautstärke im Wohnzimmer auf 5" would turn this
  // speaker down instead of the one the user meant.
  if (!lower.startsWith("lautst") && !lower.startsWith("volume")) return false;

  uint8_t words = 1;
  for (size_t i = 0; i < lower.length(); i++) {
    if (lower[i] == ' ') words++;
  }
  if (words > 3 || lower.length() > 28) {
    diagLogf("LOCAL_CMD", "ignored, looks like a sentence: \"%s\"", text.c_str());
    return false;
  }

  int step = spokenVolumeStep(lower);
  if (step < 0) return false;

  uint8_t percent = (uint8_t)constrain(step * 10, 0, 100);
  setVolume(percent);

  wakeLastMessage = "Lautstärke lokal auf " + String(step) + " (" + String(percent) + " %) gesetzt.";
  diagLogf("LOCAL_CMD", "volume step=%d percent=%u from \"%s\"",
           step, percent, text.c_str());
  return true;
}

// Language models answer in Markdown, and a text-to-speech engine happily
// reads "Sternchen Sternchen" out loud. This turns the markup into something
// worth listening to: bullets become sentences, emphasis markers disappear.
String stripMarkdownForSpeech(const String &in) {
  String out;
  out.reserve(in.length());

  bool lineStart = true;

  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];

    if (c == '\r') continue;

    if (c == '\n') {
      // End the line like a sentence so the voice pauses instead of running
      // every list item together.
      while (out.length() > 0 && out[out.length() - 1] == ' ') {
        out.remove(out.length() - 1);
      }
      if (out.length() > 0) {
        char last = out[out.length() - 1];
        if (last != '.' && last != ':' && last != '!' && last != '?' && last != ',') {
          out += '.';
        }
        out += ' ';
      }
      lineStart = true;
      continue;
    }

    if (lineStart) {
      if (c == ' ' || c == '\t') continue;

      // Bullets, headings and quote markers at the start of a line.
      if (c == '*' || c == '-' || c == '+' || c == '#' || c == '>') {
        size_t j = i;
        while (j < in.length() && (in[j] == c || in[j] == ' ')) j++;
        i = j - 1;
        continue;
      }

      // Numbered lists: "1. " keeps the number, it carries meaning.
      lineStart = false;
    }

    // Emphasis and code markers anywhere in the line.
    if (c == '*' || c == '_' || c == '`' || c == '~') continue;

    // Collapse runs of spaces.
    if (c == ' ' && out.length() > 0 && out[out.length() - 1] == ' ') continue;

    out += c;
  }

  out.trim();
  return out;
}

String assistEventType(const String &msg) {
  int eventAt = msg.indexOf("\"event\"");
  if (eventAt >= 0) {
    String nested = jsonFindStringFrom(msg, "type", eventAt);
    if (nested.length() > 0 && nested != "event") return nested;
  }

  return jsonFindString(msg, "type");
}

void appendAssistantDelta(const String &text) {
  if (text.length() == 0) return;
  if (wakeAssistantText.endsWith(text)) return;
  wakeAssistantText += text;
}

bool wakeCanStart() {
  return !wakeBusy && (int32_t)(millis() - wakeIgnoreUntil) >= 0;
}

void updateAssistFromWsMessage(const String &msg) {
  wakeLastWsDetail = msg.substring(0, min(180, (int)msg.length()));

  String eventType = assistEventType(msg);
  if (eventType.length() > 0) {
    diagLog("WS_EVENT", eventType + " " + wakeLastWsDetail);
  } else if (msg.indexOf("\"type\":\"result\"") >= 0) {
    diagLog("WS_EVENT", String("result ") + wakeLastWsDetail);
  }
  if (eventType.length() > 0 && eventType != "event") {
    wakeLastWsStage = eventType;
  }

  String conversationId = jsonFindString(msg, "conversation_id");
  if (conversationId.length() > 0) {
    haConversationId = conversationId;
    haConversationSeenAt = millis();
  }

  if (eventType == "stt-end") {
    String text = jsonFindLastString(msg, "text");
    if (text.length() > 0) {
      wakeTranscript = text;
      if (handleLocalVoiceCommand(text)) localCommandHandled = true;
    }
  } else if (eventType == "intent-progress") {
    appendAssistantDelta(jsonFindLastString(msg, "content"));
  } else if (eventType == "intent-end") {
    String speech = jsonFindLastString(msg, "speech");
    if (speech.length() > 0) wakeAssistantText = speech;
    else appendAssistantDelta(jsonFindLastString(msg, "content"));
    haContinueConversation = jsonFindBool(msg, "continue_conversation", false);
  } else if (eventType == "tts-start") {
    // The pipeline decides engine, language and voice. Our own tts_get_url
    // calls have to repeat all three, otherwise Home Assistant falls back to
    // the engine default - which for tts.google_translate_en_com is English.
    String engine = jsonFindString(msg, "engine");
    String language = jsonFindString(msg, "language");
    String voice = jsonFindString(msg, "voice");

    if ((engine.length() > 0 && engine != haTtsEngine) ||
        (language.length() > 0 && language != haTtsLanguage) ||
        voice != haTtsVoice) {
      if (engine.length() > 0) haTtsEngine = engine;
      if (language.length() > 0) haTtsLanguage = language;
      haTtsVoice = voice;

      // Learned mid-conversation, so persist right away.
      prefs.begin("voicedot", false);
      prefs.putString("ha_tts_eng", haTtsEngine);
      prefs.putString("ha_tts_lang", haTtsLanguage);
      prefs.putString("ha_tts_voice", haTtsVoice);
      prefs.end();
      diagLogf("ASSIST", "tts engine=%s lang=%s voice=%s",
               haTtsEngine.c_str(),
               haTtsLanguage.length() ? haTtsLanguage.c_str() : "-",
               haTtsVoice.length() ? haTtsVoice.c_str() : "-");
    }
  } else if (eventType == "tts-end") {
    String url = jsonFindLastString(msg, "url");
    if (url.length() > 0) wakeTtsUrl = url;
  } else {
    String url = jsonFindLastString(msg, "url");
    if (url.length() > 0) wakeTtsUrl = url;
  }
}

// mDNS/DHCP hostname derived from the configured device name, so several
// VoiceDots in one network do not fight over "voicedot.local".
String deviceHostname() {
  String out;
  String src = cfg.deviceName;
  src.trim();

  for (size_t i = 0; i < src.length() && out.length() < 24; i++) {
    char c = src[i];
    if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      out += c;
    } else if (out.length() > 0 && !out.endsWith("-")) {
      out += '-';
    }
  }

  while (out.endsWith("-")) out.remove(out.length() - 1);
  if (out.length() < 2) out = DEFAULT_HOSTNAME;
  return out;
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
  cfg.haPipeline = prefs.getString("ha_pipeline", "");
  cfg.volume = prefs.getUChar("volume", 60);
  cfg.ledBrightness = prefs.getUChar("led_bri", 32);
  cfg.vadEnabled = prefs.getBool("vad", true);
  cfg.followUp = prefs.getBool("follow_up", true);
  cfg.wakeWord = prefs.getBool("wakeword", true);
  cfg.wakeWordModel = prefs.getString("ww_model", "");
  cfg.ackEnabled = prefs.getBool("ack_on", true);
  cfg.listenColor = prefs.getUInt("listen_col", 0x0096FF);  // blue
  cfg.speakColor = prefs.getUInt("speak_col", 0xFF7000);  // orange
  cfg.ttsSpeed = prefs.getUChar("tts_speed", 100);
  cfg.ttsSpeed = constrain(cfg.ttsSpeed, TTS_SPEED_MIN, TTS_SPEED_MAX);
  cfg.vadReleaseDb = prefs.getUChar("vad_rel_db", 8);
  cfg.vadReleaseDb = constrain(cfg.vadReleaseDb, 3, 18);
  cfg.vadSilenceMs = prefs.getUShort("vad_sil_ms", (uint16_t)WAKE_SILENCE_MS);
  cfg.vadSilenceMs = constrain(cfg.vadSilenceMs, 600, 3000);

  cfg.scheduleEnabled = prefs.getBool("sched_on", false);
  cfg.dayStartMin = prefs.getUShort("day_start", 6 * 60);
  cfg.nightStartMin = prefs.getUShort("night_start", 19 * 60);
  cfg.dayVolumeStep = constrain(prefs.getUChar("day_vol", 8), 0, 10);
  cfg.dayBrightness = constrain(prefs.getUChar("day_bri", 80), 0, LED_BRIGHTNESS_MAX);
  cfg.nightVolumeStep = constrain(prefs.getUChar("night_vol", 3), 0, 10);
  cfg.nightBrightness = constrain(prefs.getUChar("night_bri", 30), 0, LED_BRIGHTNESS_MAX);
  cfg.haPublish = prefs.getBool("ha_publish", false);
  cfg.cleanMarkdown = prefs.getBool("clean_md", true);
  cfg.alarmMinutes = prefs.getShort("alrm_min", -1);
  cfg.alarmDaily = prefs.getBool("alrm_day", false);
  cfg.alarmSound = prefs.getString("alrm_snd", "");
  cfg.alarmBriefing = prefs.getString("alrm_brf", "");
  cfg.timerSound = prefs.getString("tmr_snd", "");
  cfg.micGainDb = constrain(prefs.getUChar("mic_gain", 30), 0, 36);
  cfg.sttHaVad = prefs.getBool("stt_havad", false);
  cfg.sttNoiseSuppression = constrain(prefs.getUChar("stt_ns", 2), 0, 4);
  cfg.sttAutoGainDb = constrain(prefs.getUChar("stt_agc", 24), 0, 31);
  cfg.sttVolumePercent = constrain(prefs.getUShort("stt_vol", 100), 100, 400);
  cfg.volumeStep = constrain(prefs.getUChar("vol_step", 10), 5, 25);
  cfg.updateCheckEnabled = prefs.getBool("upd_chk", true);
  cfg.autoVolumeEnabled = prefs.getBool("avol_on", true);
  cfg.autoVolumeMaxDb = constrain(prefs.getUChar("avol_max", 10), 0, 18);
  cfg.multiEnabled = prefs.getBool("multi_on", true);
  cfg.multiWindowMs = constrain(prefs.getUShort("multi_win", 220), 80, 600);
  cfg.timezone = prefs.getString("tz", DEFAULT_TIMEZONE);
  if (cfg.timezone.isEmpty()) cfg.timezone = DEFAULT_TIMEZONE;
  cfg.ackPhrases = prefs.getString("ack_phrases", ACK_DEFAULT_PHRASES);
  haTtsEngine = prefs.getString("ha_tts_eng", "");
  haTtsLanguage = prefs.getString("ha_tts_lang", "");
  haTtsVoice = prefs.getString("ha_tts_voice", "");

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
  prefs.putString("ha_pipeline", cfg.haPipeline);
  prefs.putUChar("volume", cfg.volume);
  prefs.putUChar("led_bri", cfg.ledBrightness);
  prefs.putBool("vad", cfg.vadEnabled);
  prefs.putBool("follow_up", cfg.followUp);
  prefs.putBool("wakeword", cfg.wakeWord);
  prefs.putString("ww_model", cfg.wakeWordModel);
  prefs.putBool("ack_on", cfg.ackEnabled);
  prefs.putUInt("listen_col", cfg.listenColor);
  prefs.putUInt("speak_col", cfg.speakColor);
  prefs.putUChar("tts_speed", cfg.ttsSpeed);
  prefs.putUChar("vad_rel_db", cfg.vadReleaseDb);
  prefs.putUShort("vad_sil_ms", cfg.vadSilenceMs);
  prefs.putBool("sched_on", cfg.scheduleEnabled);
  prefs.putUShort("day_start", cfg.dayStartMin);
  prefs.putUShort("night_start", cfg.nightStartMin);
  prefs.putUChar("day_vol", cfg.dayVolumeStep);
  prefs.putUChar("day_bri", cfg.dayBrightness);
  prefs.putUChar("night_vol", cfg.nightVolumeStep);
  prefs.putUChar("night_bri", cfg.nightBrightness);
  prefs.putBool("ha_publish", cfg.haPublish);
  prefs.putBool("clean_md", cfg.cleanMarkdown);
  prefs.putShort("alrm_min", cfg.alarmMinutes);
  prefs.putBool("alrm_day", cfg.alarmDaily);
  prefs.putString("alrm_snd", cfg.alarmSound);
  prefs.putString("alrm_brf", cfg.alarmBriefing);
  prefs.putString("tmr_snd", cfg.timerSound);
  prefs.putUChar("mic_gain", cfg.micGainDb);
  prefs.putBool("stt_havad", cfg.sttHaVad);
  prefs.putUChar("stt_ns", cfg.sttNoiseSuppression);
  prefs.putUChar("stt_agc", cfg.sttAutoGainDb);
  prefs.putUShort("stt_vol", cfg.sttVolumePercent);
  prefs.putUChar("vol_step", cfg.volumeStep);
  prefs.putBool("upd_chk", cfg.updateCheckEnabled);
  prefs.putBool("avol_on", cfg.autoVolumeEnabled);
  prefs.putUChar("avol_max", cfg.autoVolumeMaxDb);
  prefs.putBool("multi_on", cfg.multiEnabled);
  prefs.putUShort("multi_win", cfg.multiWindowMs);
  prefs.putString("tz", cfg.timezone);
  prefs.putString("ack_phrases", cfg.ackPhrases);
  prefs.putString("ha_tts_eng", haTtsEngine);
  prefs.putString("ha_tts_lang", haTtsLanguage);
  prefs.putString("ha_tts_voice", haTtsVoice);

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
  prefs.putUInt("listen_col", cfg.listenColor);
  prefs.putUInt("speak_col", cfg.speakColor);
  prefs.putUChar("tts_speed", cfg.ttsSpeed);
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

  logPrintf("Selected profile: %s (SDA=%d SCL=%d)",
            activeProfile.name, activeProfile.i2cSda, activeProfile.i2cScl);
  logPrintf("I2C: ES8311=%s ES7210=%s TCA9555=%s RTC=%s",
            es8311Present ? "ok" : "-",
            es7210Present ? "ok" : "-",
            tca9555Present ? "ok" : "-",
            rtcPresent ? "ok" : "-");
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

// The amplifier gets uncomfortably loud well before the codec's register runs
// out, so the top of the scale is capped: what the interface calls 100 % is
// four fifths of what the chip could do, and nothing goes above it - the
// ambient boost included.
static constexpr uint8_t ES8311_REG_CEILING = 209;  // 80 % of the full 32..255

uint8_t es8311VolumeReg(uint8_t percent) {
  if (percent == 0) return 0x00;
  return map(percent, 1, 100, 32, ES8311_REG_CEILING);
}

// A boost in decibels is just an offset on the volume register - going through
// the percent scale would quantise it twice for nothing.
void es8311SetVolumeBoosted(uint8_t percent, float boostDb) {
  if (!codecPlaybackReady) return;

  int reg = es8311VolumeReg(percent);
  if (percent > 0 && boostDb > 0.0f) {
    reg += (int)lroundf(boostDb / ES8311_DB_PER_STEP);
    if (reg > ES8311_REG_CEILING) reg = ES8311_REG_CEILING;
  }

  codecWrite(ADDR_ES8311, 0x32, (uint8_t)reg);
  es8311SetMute(percent == 0 || muted);
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
  if (audioTxChan || audioRxChan) {
    if (audioTxChan) {
      i2s_channel_disable(audioTxChan);
      i2s_del_channel(audioTxChan);
      audioTxChan = nullptr;
    }
    if (audioRxChan) {
      i2s_channel_disable(audioRxChan);
      i2s_del_channel(audioRxChan);
      audioRxChan = nullptr;
    }
    audioI2sReady = false;
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 6;
  chanCfg.dma_frame_num = 256;
  chanCfg.auto_clear_after_cb = true;

  esp_err_t err = i2s_new_channel(&chanCfg, &audioTxChan, &audioRxChan);
  if (err != ESP_OK) {
    Serial.printf("I2S new channel failed: %d\n", (int)err);
    return false;
  }

  i2s_std_config_t stdCfg = {};
  stdCfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);
  stdCfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_768;
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = (gpio_num_t)activeProfile.i2sMclk;
  stdCfg.gpio_cfg.bclk = (gpio_num_t)activeProfile.i2sBclk;
  stdCfg.gpio_cfg.ws = (gpio_num_t)activeProfile.i2sLrclk;
  stdCfg.gpio_cfg.dout = (gpio_num_t)activeProfile.speakerData;
  stdCfg.gpio_cfg.din = (gpio_num_t)activeProfile.micData;
  stdCfg.gpio_cfg.invert_flags.mclk_inv = false;
  stdCfg.gpio_cfg.invert_flags.bclk_inv = false;
  stdCfg.gpio_cfg.invert_flags.ws_inv = false;

  err = i2s_channel_init_std_mode(audioTxChan, &stdCfg);
  if (err != ESP_OK) {
    Serial.printf("I2S TX init failed: %d\n", (int)err);
    i2s_del_channel(audioTxChan);
    i2s_del_channel(audioRxChan);
    audioTxChan = nullptr;
    audioRxChan = nullptr;
    return false;
  }

  err = i2s_channel_init_std_mode(audioRxChan, &stdCfg);
  if (err != ESP_OK) {
    Serial.printf("I2S RX init failed: %d\n", (int)err);
    i2s_del_channel(audioTxChan);
    i2s_del_channel(audioRxChan);
    audioTxChan = nullptr;
    audioRxChan = nullptr;
    return false;
  }

  err = i2s_channel_enable(audioRxChan);
  if (err != ESP_OK) {
    Serial.printf("I2S RX enable failed: %d\n", (int)err);
    i2s_del_channel(audioTxChan);
    i2s_del_channel(audioRxChan);
    audioTxChan = nullptr;
    audioRxChan = nullptr;
    return false;
  }

  err = i2s_channel_enable(audioTxChan);
  if (err != ESP_OK) {
    Serial.printf("I2S TX enable failed: %d\n", (int)err);
    i2s_channel_disable(audioRxChan);
    i2s_del_channel(audioTxChan);
    i2s_del_channel(audioRxChan);
    audioTxChan = nullptr;
    audioRxChan = nullptr;
    return false;
  }

  audioI2sReady = true;
  Serial.println("I2S: new STD driver, 16 kHz, 16-bit stereo, shared RX/TX ready");
  return true;
}

bool audioSetSampleRate(uint32_t sampleRate) {
  if (!audioI2sReady || !audioTxChan || !audioRxChan) return false;

  i2s_channel_disable(audioTxChan);
  i2s_channel_disable(audioRxChan);

  i2s_std_clk_config_t clkCfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
  clkCfg.mclk_multiple = I2S_MCLK_MULTIPLE_768;

  esp_err_t errTx = i2s_channel_reconfig_std_clock(audioTxChan, &clkCfg);
  esp_err_t errRx = i2s_channel_reconfig_std_clock(audioRxChan, &clkCfg);
  esp_err_t enRx = i2s_channel_enable(audioRxChan);
  esp_err_t enTx = i2s_channel_enable(audioTxChan);

  return errTx == ESP_OK && errRx == ESP_OK && enRx == ESP_OK && enTx == ESP_OK;
}

esp_err_t audioRead(void *dest, size_t size, size_t *bytesRead, uint32_t timeoutMs) {
  if (!audioRxChan) {
    if (bytesRead) *bytesRead = 0;
    return ESP_ERR_INVALID_STATE;
  }
  return i2s_channel_read(audioRxChan, dest, size, bytesRead, timeoutMs);
}

esp_err_t audioWrite(const void *src, size_t size, size_t *bytesWritten, uint32_t timeoutMs) {
  if (!audioTxChan) {
    if (bytesWritten) *bytesWritten = 0;
    diagLog("I2S_WRITE", "TX channel fehlt");
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = i2s_channel_write(audioTxChan, src, size, bytesWritten, timeoutMs);
  if (err != ESP_OK || (bytesWritten && *bytesWritten != size)) {
    diagLogf("I2S_WRITE", "err=%d requested=%lu written=%lu timeout=%lu",
             (int)err,
             (unsigned long)size,
             (unsigned long)(bytesWritten ? *bytesWritten : 0),
             (unsigned long)timeoutMs);
  }
  return err;
}

void audioClearTx() {
  if (!audioI2sReady || !audioTxChan) return;

  int16_t silence[128 * 2] = {0};
  size_t bytesWritten = 0;
  for (uint8_t i = 0; i < 4; i++) {
    audioWrite(silence, sizeof(silence), &bytesWritten, 20);
  }
}

void audioWriteSilenceBlocks(uint8_t blocks, uint32_t timeoutMs) {
  if (!audioI2sReady || !audioTxChan) return;

  int16_t silence[128 * 2] = {0};
  for (uint8_t i = 0; i < blocks; i++) {
    size_t bytesWritten = 0;
    audioWrite(silence, sizeof(silence), &bytesWritten, timeoutMs);
    ledTick();
    pumpServices();
    yield();
  }
}

// Clocking the I2S faster than the stream was encoded plays it back faster.
uint32_t scaledPlaybackRate(uint32_t nominal) {
  uint32_t scaled = (uint32_t)((uint64_t)nominal * cfg.ttsSpeed / 100ULL);
  if (scaled < 8000) scaled = 8000;
  if (scaled > 48000) scaled = 48000;
  return scaled;
}

// How much louder the voice has to be to stay as intelligible as it was in a
// quiet room. One decibel of room noise costs one decibel of speech, so the
// mapping is 1:1 - only the ceiling is a matter of taste.
float autoVolumeBoostDb() {
  if (!cfg.autoVolumeEnabled || cfg.autoVolumeMaxDb == 0) return 0.0f;
  if (!autoVolumeNoiseValid) return 0.0f;

  float over = autoVolumeNoiseDb - AUTO_VOLUME_QUIET_DB;
  if (over <= 0.0f) return 0.0f;
  if (over > (float)cfg.autoVolumeMaxDb) over = (float)cfg.autoVolumeMaxDb;
  return over;
}

bool beginTtsPlayback(uint32_t sampleRate, const char *tag) {
  if (!audioI2sReady || !codecPlaybackReady) return false;

  ttsPlaybackActive = true;
  setAmplifier(true);
  es8311SetMute(false);

  // Speech only. Music gets no boost: it is not the thing that has to stay
  // understandable, and a stream that measures the room it is filling would
  // chase its own tail.
  autoVolumeLastBoost = autoVolumeBoostDb();
  es8311SetVolumeBoosted(cfg.volume > 0 ? cfg.volume : 60, autoVolumeLastBoost);
  if (autoVolumeLastBoost > 0.0f) {
    diagLogf(tag, "auto volume +%.1f dB (Rauschboden %.1f dBFS)",
             autoVolumeLastBoost, autoVolumeNoiseDb);
  }
  bool rateOk = audioSetSampleRate(scaledPlaybackRate(sampleRate));
  audioClearTx();
  delay(TTS_AMP_WARMUP_MS);
  audioWriteSilenceBlocks(TTS_PREROLL_BLOCKS, 80);
  diagLogf(tag, "playback prepared rate=%lu(x%u%%) amp=%u preroll=%u rateOk=%u",
           (unsigned long)scaledPlaybackRate(sampleRate),
           (unsigned)cfg.ttsSpeed,
           amplifierEnabled ? 1 : 0,
           TTS_PREROLL_BLOCKS,
           rateOk ? 1 : 0);
  return rateOk;
}

void finishTtsPlayback(const char *tag) {
  audioWriteSilenceBlocks(TTS_TAIL_BLOCKS, 80);
  audioClearTx();
  delay(TTS_AMP_HOLD_MS);
  audioSetSampleRate(AUDIO_SAMPLE_RATE);
  es8311SetVolume(cfg.volume);
  if (cfg.volume == 0 || muted) es8311SetMute(true);
  delay(40);
  setAmplifier(false);
  ttsPlaybackActive = false;
  wakeIgnoreUntil = millis() + WAKE_COOLDOWN_MS;
  diagLogf(tag, "playback finished cooldown=%u amp=%u",
           WAKE_COOLDOWN_MS,
           amplifierEnabled ? 1 : 0);
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

// The two microphone channels share one setting. Writable while running, so a
// gain that turns out too low does not need a reboot to correct.
bool es7210ApplyGain(uint8_t db) {
  if (!es7210Present) return false;

  uint8_t reg = es7210GainReg((float)db);
  bool ok = codecWrite(ADDR_ES7210, 0x43, 0x10 | reg);
  ok &= codecWrite(ADDR_ES7210, 0x44, 0x10 | reg);
  diagLogf("MIC", "gain set to %u dB (register 0x%02X) ok=%u", db, 0x10 | reg, ok ? 1 : 0);
  return ok;
}

bool setupEs7210() {
  if (!es7210Present) return false;

  const uint8_t gain = es7210GainReg((float)cfg.micGainDb);
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

  logPrintf("Audio: I2S=%s playback=%s record=%s",
            i2sOk ? "OK" : "FAIL",
            outOk ? "OK" : "FAIL",
            inOk ? "OK" : "FAIL");
  return i2sOk && (outOk || inOk);
}

void pollMicLevel() {
  if (!audioI2sReady || !codecRecordReady || speakerTestActive || ttsPlaybackActive) return;

  // While WakeNet is running it owns the RX channel and updates the meter
  // itself; reading here would steal frames from the detector.
  if (srRunning && !srPaused) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastMicPoll) < 45) return;
  lastMicPoll = now;

  int16_t samples[AUDIO_FRAME_SAMPLES * 2];
  size_t bytesRead = 0;
  esp_err_t err = audioRead(samples, sizeof(samples), &bytesRead, 1);
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

  // Same asymmetric tracker as in the detector: drop quickly towards a quiet
  // room, creep upwards in a noisy one, so speech barely moves it.
  if (!srRoomNoiseValid) {
    srRoomNoiseDb = micDb;
    srRoomNoiseValid = true;
  } else if (micDb < srRoomNoiseDb) {
    srRoomNoiseDb += (micDb - srRoomNoiseDb) * 0.20f;
  } else {
    srRoomNoiseDb += (micDb - srRoomNoiseDb) * 0.005f;
  }

  if (srRoomNoiseDb > VAD_NOISE_FLOOR_MAX_DB) srRoomNoiseDb = VAD_NOISE_FLOOR_MAX_DB;
  srRoomNoiseAt = millis();

  // Rises instantly, falls about 30 dB in a second and a half, so the peak of
  // the wake word is still standing when the detector reports it.
  if (micDb > srRecentPeakDb) srRecentPeakDb = micDb;
  else srRecentPeakDb -= 1.2f;
  if (srRecentPeakDb < -96.0f) srRecentPeakDb = -96.0f;
}

// -----------------------------------------------------------------------------
// Local wake word (Espressif WakeNet)
//
// WakeNet reads the same I2S RX channel as the recorder, so only one of the two
// may own the microphone at a time. Everything that records or plays calls
// srPauseDetection() first and srResumeDetection() afterwards.
//
// The detector runs in its own FreeRTOS tasks. The only thing its callback does
// is raise srWakeFlag; loop() turns that into a normal wake request, so an
// Assist turn still starts from exactly one place.
// -----------------------------------------------------------------------------

#if VOICEDOT_WAKEWORD

static srmodel_list_t *srModels = nullptr;
static const esp_afe_sr_iface_t *srAfe = nullptr;
static esp_afe_sr_data_t *srAfeData = nullptr;
static volatile bool srRunTasks = false;
static volatile bool srFeedRunning = false;
static volatile bool srDetectRunning = false;
static volatile bool srFeedIdle = false;
static volatile uint32_t srFeedCount = 0;
static volatile uint32_t srShortReads = 0;
static volatile uint32_t srMaxReadMs = 0;
static volatile uint32_t srMaxFeedMs = 0;
static volatile uint32_t srMaxMeterMs = 0;
static volatile uint32_t srFetchCount = 0;
static volatile uint32_t srFetchFail = 0;
static volatile int srLastWakeupState = 0;
static int srFeedChunk = 0;
static int srFeedChannels = 0;
static int srFetchChunk = 0;

// Level meter, fed from the detector because it owns the microphone while it
// runs. Also keeps the long term room noise estimate the recorder starts from.
static void srUpdateMeter(const int16_t *samples, size_t count) {
  if (count == 0) return;

  int32_t peak = 0;
  uint64_t sumSquares = 0;
  for (size_t i = 0; i < count; i++) {
    int32_t v = samples[i];
    int32_t a = v < 0 ? -v : v;
    if (a > peak) peak = a;
    sumSquares += (uint64_t)(v * v);
  }

  float rms = sqrtf((float)sumSquares / (float)count);
  micPeak = (int16_t)constrain(peak, 0, 32767);
  micDb = rms > 1.0f ? 20.0f * log10f(rms / 32768.0f) : -96.0f;
  micLevel = constrain((int)map(micPeak, 0, 4000, 0, 100), 0, 100);
  micReadCount++;
  micLastBytes = count * sizeof(int16_t);
  if (micPeak < 4) micSilentFrames++;

  // Follow a quiet room quickly and a loud one slowly, so passing speech
  // barely moves the estimate.
  if (!srRoomNoiseValid) {
    srRoomNoiseDb = micDb;
    srRoomNoiseValid = true;
  } else if (micDb < srRoomNoiseDb) {
    srRoomNoiseDb = srRoomNoiseDb * 0.8f + micDb * 0.2f;
  } else {
    srRoomNoiseDb = srRoomNoiseDb * 0.995f + micDb * 0.005f;
  }

  if (srRoomNoiseDb > VAD_NOISE_FLOOR_MAX_DB) srRoomNoiseDb = VAD_NOISE_FLOOR_MAX_DB;
  srRoomNoiseAt = millis();

  // Rises instantly, falls about 30 dB in a second and a half, so the peak of
  // the wake word is still standing when the detector reports it.
  if (micDb > srRecentPeakDb) srRecentPeakDb = micDb;
  else srRecentPeakDb -= 1.2f;
  if (srRecentPeakDb < -96.0f) srRecentPeakDb = -96.0f;
}

// Reads the microphone and hands frames to the AFE.
static void srFeedTask(void *arg) {
  (void)arg;
  srFeedRunning = true;

  int chunk = srAfe->get_feed_chunksize(srAfeData);
  int feedChannels = srAfe->get_feed_channel_num(srAfeData);
  srFeedChunk = chunk;
  srFeedChannels = feedChannels;
  size_t rxBytes = (size_t)chunk * SR_RX_CHANNELS * sizeof(int16_t);
  size_t bufSamples = (size_t)chunk * (size_t)feedChannels;

  int16_t *buf = (int16_t*)heap_caps_malloc(bufSamples * sizeof(int16_t),
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (buf == nullptr) {
    diagLog("SR", "feed task has no memory for the audio buffer");
    srFeedRunning = false;
    vTaskDelete(NULL);
    return;
  }

  while (srRunTasks) {
    if (srPaused || ttsPlaybackActive || speakerTestActive ||
        !audioI2sReady || !codecRecordReady) {
      srFeedIdle = true;
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    srFeedIdle = false;
    size_t got = 0;
    uint32_t t0 = millis();
    if (audioRead(buf, rxBytes, &got, 120) != ESP_OK) got = 0;
    uint32_t t1 = millis();
    if (t1 - t0 > srMaxReadMs) srMaxReadMs = t1 - t0;
    if (got < rxBytes) {
      memset((uint8_t*)buf + got, 0, rxBytes - got);
      srShortReads++;
    }

    // The meter is only for the UI, it does not need every single chunk.
    if ((srFeedCount & 0x03) == 0) {
      srUpdateMeter(buf, got / sizeof(int16_t));
    }
    uint32_t t2 = millis();
    if (t2 - t1 > srMaxMeterMs) srMaxMeterMs = t2 - t1;

    // Spread our channels out if the AFE wants more than the microphone gives.
    if (feedChannels > SR_RX_CHANNELS) {
      for (int i = chunk - 1; i >= 0; i--) {
        for (int f = feedChannels - 1; f >= 0; f--) {
          buf[i * feedChannels + f] = (f >= SR_RX_CHANNELS)
                                      ? 0
                                      : buf[i * SR_RX_CHANNELS + f];
        }
      }
    }

    srAfe->feed(srAfeData, buf);
    uint32_t t3 = millis();
    if (t3 - t2 > srMaxFeedMs) srMaxFeedMs = t3 - t2;
    srFeedCount++;
  }

  heap_caps_free(buf);
  srFeedIdle = true;
  srFeedRunning = false;
  vTaskDelete(NULL);
}

// Pulls AFE results and raises the flag loop() acts on. Unlike the Arduino
// wrapper this never disables WakeNet, so there is nothing to re-arm.
static void srDetectTask(void *arg) {
  (void)arg;
  srDetectRunning = true;

  while (srRunTasks) {
    if (srPaused) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    afe_fetch_result_t *res = srAfe->fetch_with_delay(srAfeData, pdMS_TO_TICKS(200));
    if (res == nullptr || res->ret_value == ESP_FAIL) {
      srFetchFail++;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    srFetchCount++;
    if (res->wakeup_state != 0) srLastWakeupState = (int)res->wakeup_state;

    if (res->wakeup_state == WAKENET_DETECTED) {
      srWakeFlag = true;
    }
  }

  srDetectRunning = false;
  vTaskDelete(NULL);
}

// Reads the model index out of the partition once and remembers every WakeNet
// model it holds, together with its human readable wake word.
static bool srScanModels() {
  srModelCount = 0;

  const esp_partition_t *part = esp_partition_find_first(
    ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "model");
  if (part == nullptr) {
    srAvailable = false;
    srStatus = "Partition \"model\" fehlt, bitte partitions.csv verwenden und srmodels.bin flashen";
    Serial.println("WakeWord: no \"model\" partition found");
    return false;
  }

  if (srModels == nullptr) {
    srModels = esp_srmodel_init("model");
  }

  if (srModels == nullptr || srModels->num <= 0) {
    srAvailable = false;
    srStatus = "keine Modelle in der Partition";
    Serial.println("WakeWord: model partition holds no models");
    return false;
  }

  for (int i = 0; i < srModels->num && srModelCount < SR_MAX_MODELS; i++) {
    const char *name = srModels->model_name[i];
    if (name == nullptr || strncmp(name, ESP_WN_PREFIX, strlen(ESP_WN_PREFIX)) != 0) continue;

    srModelId[srModelCount] = name;

    char *words = esp_srmodel_get_wake_words(srModels, (char*)name);
    srModelWords[srModelCount] = (words != nullptr && words[0] != '\0') ? String(words) : String(name);
    logPrintf("WakeWord: model %s -> \"%s\"",
              name, srModelWords[srModelCount].c_str());
    srModelCount++;
  }

  srAvailable = srModelCount > 0;
  if (!srAvailable) srStatus = "kein WakeNet-Modell in der Partition";
  return srAvailable;
}

// The configured model if it is actually present, otherwise the first one.
static int srPickModelIndex() {
  if (srModelCount == 0) return -1;

  if (cfg.wakeWordModel.length() > 0) {
    for (uint8_t i = 0; i < srModelCount; i++) {
      if (srModelId[i] == cfg.wakeWordModel) return (int)i;
    }
    diagLogf("SR", "configured model %s is not in the partition, using the first one",
             cfg.wakeWordModel.c_str());
  }

  return 0;
}

static void srStopEngine() {
  if (srAfeData == nullptr && !srRunTasks) return;

  srRunTasks = false;
  uint32_t started = millis();
  while ((srFeedRunning || srDetectRunning) &&
         (uint32_t)(millis() - started) < SR_TASK_STOP_TIMEOUT_MS) {
    delay(10);
  }

  if (srFeedRunning || srDetectRunning) {
    diagLog("SR", "detector tasks did not stop in time, leaving the AFE alive");
    return;
  }

  if (srAfe != nullptr && srAfeData != nullptr) {
    srAfe->destroy(srAfeData);
  }

  srAfe = nullptr;
  srAfeData = nullptr;
  srRunning = false;
  srPaused = false;
  srActiveModel = "";
  srActiveWords = "";
  srRoomNoiseValid = false;
}

static bool srStartEngine() {
  int idx = srPickModelIndex();
  if (idx < 0) return false;

  String wanted = srModelId[idx];

  afe_config_t *afeCfg = afe_config_init(WAKE_WORD_INPUT_FORMAT, srModels,
                                         AFE_TYPE_SR, AFE_MODE_LOW_COST);
  if (afeCfg == nullptr) {
    srStatus = "AFE-Konfiguration fehlgeschlagen";
    return false;
  }

  // This is the whole point of driving esp-sr directly: afe_config_init picks
  // the first WakeNet it finds, here we override it with the chosen one.
  char *selected = esp_srmodel_filter(srModels, ESP_WN_PREFIX, (char*)wanted.c_str());
  if (selected != nullptr) afeCfg->wakenet_model_name = selected;

  // AFE can run a second wake word in parallel, and afe_config_init fills that
  // slot with another model from the partition. Left alone it would keep
  // listening for a word nobody selected.
  afeCfg->wakenet_model_name_2 = nullptr;
  afeCfg->wakenet_init = true;

  srAfe = esp_afe_handle_from_config(afeCfg);
  if (srAfe == nullptr) {
    afe_config_free(afeCfg);
    srStatus = "kein AFE-Handle";
    return false;
  }

  srAfeData = srAfe->create_from_config(afeCfg);
  afe_config_free(afeCfg);

  if (srAfeData == nullptr) {
    srAfe = nullptr;
    srStatus = "AFE konnte nicht erzeugt werden (zu wenig Speicher?)";
    Serial.println("WakeWord: AFE creation failed");
    return false;
  }

  srRunTasks = true;
  srPaused = false;
  srWakeFlag = false;

  BaseType_t okFeed = xTaskCreatePinnedToCore(srFeedTask, "sr_feed", 4096, nullptr, 5, nullptr, 0);
  BaseType_t okDetect = xTaskCreatePinnedToCore(srDetectTask, "sr_detect", 8192, nullptr, 5, nullptr, 1);

  if (okFeed != pdPASS || okDetect != pdPASS) {
    srStatus = "Detektor-Tasks konnten nicht starten";
    Serial.println("WakeWord: could not create detector tasks");
    srStopEngine();
    return false;
  }

  srFetchChunk = srAfe->get_fetch_chunksize(srAfeData);
  logPrintf("WakeWord: feed chunk=%d ch=%d, fetch chunk=%d",
            srFeedChunk, srFeedChannels, srFetchChunk);

  srRunning = true;
  srActiveModel = srModelId[idx];
  srActiveWords = srModelWords[idx];
  srStatus = String("aktiv, Stichwort \"") + srActiveWords + "\"";
  logPrintf("WakeWord: listening for \"%s\" (%s)",
            srActiveWords.c_str(), srActiveModel.c_str());
  return true;
}

bool srBegin() {
  srRunning = false;
  srPaused = false;

  if (!audioI2sReady || !codecRecordReady) {
    srStatus = "Mikrofon ist nicht bereit";
    Serial.println("WakeWord: microphone not ready, detector stays off");
    return false;
  }

  // Scan even when the detector is switched off, so the web UI can still show
  // which wake words this board could offer.
  if (!srScanModels()) return false;

  if (!cfg.wakeWord) {
    srStatus = "deaktiviert";
    return false;
  }

  return srStartEngine();
}

void srPauseDetection() {
  if (!srRunning || srPaused) return;

  srPaused = true;
  srPausedAt = millis();

  uint32_t waitStart = millis();
  while (!srFeedIdle && (uint32_t)(millis() - waitStart) < SR_PAUSE_SETTLE_MS) {
    delay(2);
  }

  diagLogf("SR", "detection paused after %lums%s",
           (unsigned long)(millis() - waitStart),
           srFeedIdle ? "" : " (feed task did not confirm)");
}

void srResumeDetection() {
  if (!srRunning || !srPaused) return;

  srWakeFlag = false;
  srPaused = false;
  diagLog("SR", "detection resumed");
}

void srApplyConfig() {
  bool wantRunning = cfg.wakeWord && srAvailable;
  bool modelChanged = srRunning && cfg.wakeWordModel.length() > 0 &&
                      srActiveModel != cfg.wakeWordModel;

  if (srRunning && (!wantRunning || modelChanged)) {
    srStopEngine();
    Serial.println("WakeWord: detector stopped");
  }

  if (wantRunning && !srRunning) {
    if (srModelCount == 0) srScanModels();
    srStartEngine();
  } else if (!cfg.wakeWord) {
    srStatus = "deaktiviert";
  }
}

#endif  // VOICEDOT_WAKEWORD

// -----------------------------------------------------------------------------
// Making room for a handshake
//
// mbedTLS holds two 16 kB record buffers plus the handshake state, and the wake
// word detector holds roughly 73 kB of internal RAM. Both together do not fit,
// which is why an update could only be fetched at boot, where the detector has
// not started yet. For the few seconds of a transfer the detector therefore
// steps aside - the same stop and start the interface already performs when the
// wake word is changed.
// -----------------------------------------------------------------------------

static bool netDetectorYielded = false;

static bool netMakeRoom(uint32_t needed) {
  if (ESP.getFreeHeap() >= needed) return false;
#ifdef VOICEDOT_WAKEWORD
  if (!srRunning) return false;
  uint32_t before = ESP.getFreeHeap();
  srStopEngine();
  netDetectorYielded = true;
  diagLogf("NET", "detector stopped for TLS, heap %lu -> %lu",
           (unsigned long)before, (unsigned long)ESP.getFreeHeap());
  return true;
#else
  return false;
#endif
}

static void netReleaseRoom() {
  if (!netDetectorYielded) return;
  netDetectorYielded = false;
#ifdef VOICEDOT_WAKEWORD
  if (cfg.wakeWord && srAvailable) srStartEngine();
  diagLogf("NET", "detector back, heap %lu", (unsigned long)ESP.getFreeHeap());
#endif
}

void playSpeakerTestTone() {
  if (!audioI2sReady || !codecPlaybackReady) return;

  srPauseDetection();
  speakerTestActive = true;
  setAmplifier(true);
  es8311SetMute(false);
  es8311SetVolume(cfg.volume > 0 ? cfg.volume : 60);
  audioClearTx();

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
    audioWrite(buffer, framesThisChunk * 2 * sizeof(int16_t), &bytesWritten, 100);
    frame += framesThisChunk;
    pumpServices();
    yield();
  }

  audioClearTx();
  es8311SetVolume(cfg.volume);
  if (cfg.volume == 0 || muted) es8311SetMute(true);
  delay(40);
  setAmplifier(false);
  speakerTestActive = false;
  srResumeDetection();
}

String absoluteHaMediaUrl(const String &url) {
  if (url.startsWith("http://") || url.startsWith("https://")) return url;

  String base = normalizedHaUrl(cfg.haUrl);
  while (base.endsWith("/")) base.remove(base.length() - 1);

  if (url.startsWith("/")) return base + url;
  return base + "/" + url;
}

bool streamReadExact(Stream &stream, uint8_t *buf, size_t len, uint32_t timeoutMs = 5000) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len) {
    int available = stream.available();
    if (available > 0) {
      size_t want = min<size_t>((size_t)available, len - got);
      size_t n = stream.readBytes((char*)buf + got, want);
      if (n > 0) {
        got += (size_t)n;
        start = millis();
      }
    } else {
      if ((uint32_t)(millis() - start) > timeoutMs) return false;
      ledTick();
      delay(1);
      yield();
    }
  }
  return true;
}

bool streamSkipBytes(Stream &stream, uint32_t len) {
  uint8_t discard[96];
  while (len > 0) {
    size_t chunk = min<uint32_t>(sizeof(discard), len);
    if (!streamReadExact(stream, discard, chunk)) return false;
    len -= chunk;
  }
  return true;
}

uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

bool playWavPcm16(Stream &stream) {
  diagLog("TTS_WAV", "start");
  uint8_t riff[12];
  if (!streamReadExact(stream, riff, sizeof(riff))) {
    wakeTtsStatus = "WAV-Header fehlt.";
    diagLog("TTS_WAV", wakeTtsStatus);
    return false;
  }

  if (memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
    wakeTtsStatus = "TTS ist kein WAV/RIFF-Stream.";
    diagLog("TTS_WAV", wakeTtsStatus);
    return false;
  }

  bool haveFmt = false;
  uint16_t audioFormat = 0;
  uint16_t channels = 0;
  uint32_t sampleRate = 0;
  uint16_t bitsPerSample = 0;
  uint32_t dataSize = 0;

  while (true) {
    uint8_t chunk[8];
    if (!streamReadExact(stream, chunk, sizeof(chunk))) {
      wakeTtsStatus = "WAV data chunk fehlt.";
      return false;
    }

    uint32_t chunkSize = le32(chunk + 4);
    if (memcmp(chunk, "fmt ", 4) == 0) {
      uint8_t fmt[40];
      size_t readNow = min<uint32_t>(sizeof(fmt), chunkSize);
      if (!streamReadExact(stream, fmt, readNow)) return false;
      if (chunkSize > readNow && !streamSkipBytes(stream, chunkSize - readNow)) return false;

      if (readNow >= 16) {
        audioFormat = le16(fmt);
        channels = le16(fmt + 2);
        sampleRate = le32(fmt + 4);
        bitsPerSample = le16(fmt + 14);
        haveFmt = true;
      }
    } else if (memcmp(chunk, "data", 4) == 0) {
      dataSize = chunkSize;
      break;
    } else {
      if (!streamSkipBytes(stream, chunkSize)) return false;
    }

    if (chunkSize & 1) {
      uint8_t pad = 0;
      if (!streamReadExact(stream, &pad, 1)) return false;
    }
  }

  if (!haveFmt || audioFormat != 1 || bitsPerSample != 16 ||
      (channels != 1 && channels != 2) ||
      sampleRate < 8000 || sampleRate > 48000 || dataSize == 0) {
    wakeTtsStatus = "WAV nicht unterstützt: PCM16 mono/stereo erwartet.";
    diagLogf("TTS_WAV", "unsupported fmt=%u ch=%u rate=%lu bits=%u data=%lu",
             audioFormat,
             channels,
             (unsigned long)sampleRate,
             bitsPerSample,
             (unsigned long)dataSize);
    return false;
  }

  diagLogf("TTS_WAV", "format ch=%u rate=%lu bits=%u data=%lu",
           channels,
           (unsigned long)sampleRate,
           bitsPerSample,
           (unsigned long)dataSize);
  if (!beginTtsPlayback(sampleRate, "TTS_WAV")) {
    wakeTtsStatus = "WAV Playback-Start fehlgeschlagen.";
    diagLog("TTS_WAV", wakeTtsStatus);
    return false;
  }

  uint8_t inBuf[512];
  int16_t stereoBuf[512];
  uint32_t remaining = dataSize;

  while (remaining > 0) {
    size_t chunk = min<uint32_t>(sizeof(inBuf), remaining);
    if (!streamReadExact(stream, inBuf, chunk, 3000)) break;
    remaining -= chunk;

    if (channels == 1) {
      size_t samples = chunk / sizeof(int16_t);
      for (size_t i = 0; i < samples; i++) {
        int16_t s = (int16_t)le16(inBuf + i * 2);
        stereoBuf[i * 2] = s;
        stereoBuf[i * 2 + 1] = s;
      }

      size_t bytesWritten = 0;
      audioWrite(stereoBuf, samples * 2 * sizeof(int16_t), &bytesWritten, 250);
    } else {
      size_t bytesWritten = 0;
      audioWrite(inBuf, chunk, &bytesWritten, 250);
    }

    pumpServices();
    yield();
  }

  finishTtsPlayback("TTS_WAV");

  bool complete = remaining == 0;
  wakeTtsStatus = complete
                  ? "WAV abgespielt: " + String(sampleRate) + " Hz, " + String(channels) + " ch"
                  : "WAV-Wiedergabe abgebrochen.";
  diagLog("TTS_WAV", wakeTtsStatus);
  return complete;
}

bool playMp3TtsUrl(const String &url) {
  diagLog("TTS_MP3", String("start ") + url);
  if (!audioI2sReady || !codecPlaybackReady) {
    wakeTtsStatus = "Playback ist nicht bereit.";
    diagLog("TTS_MP3", wakeTtsStatus);
    return false;
  }

  wakeTtsStatus = "MP3 wird dekodiert.";
  ttsOutputBytes = 0;
  ttsWriteFailures = 0;

  HTTPClient http;
  WiFiClientSecure secureClient;
  bool secure = url.startsWith("https://");
  if (secure) {
    secureClient.setInsecure();
    if (!http.begin(secureClient, url)) {
      wakeTtsStatus = "HTTPS MP3 begin fehlgeschlagen.";
      diagLog("TTS_MP3", wakeTtsStatus);
      return false;
    }
  } else if (!http.begin(url)) {
    wakeTtsStatus = "HTTP MP3 begin fehlgeschlagen.";
    diagLog("TTS_MP3", wakeTtsStatus);
    return false;
  }

  http.useHTTP10(true);  // no chunked framing in the raw stream we read from
  http.setTimeout(12000);
  http.setReuse(false);
  http.addHeader("Authorization", "Bearer " + cfg.haToken);
  http.addHeader("Accept", "audio/mpeg,audio/mp3,*/*");
  http.addHeader("Connection", "close");
  int code = http.GET();
  String contentType = http.header("Content-Type");
  contentType.toLowerCase();
  int totalSize = http.getSize();
  diagLogf("TTS_MP3", "GET code=%d type=%s size=%d", code, contentType.c_str(), totalSize);
  if (code != HTTP_CODE_OK) {
    wakeTtsStatus = "MP3 HTTP " + String(code);
    http.end();
    diagLog("TTS_MP3", wakeTtsStatus);
    return false;
  }

  if (!MP3Decoder_AllocateBuffers()) {
    wakeTtsStatus = "MP3 Decoder Speicher fehlt.";
    http.end();
    diagLog("TTS_MP3", wakeTtsStatus);
    return false;
  }

  uint8_t *inBuf = (uint8_t*)ps_malloc(MP3_INPUT_BUFFER_SIZE);
  int16_t *pcm = (int16_t*)ps_malloc(MP3_PCM_MAX_SAMPLES * sizeof(int16_t));
  int16_t *stereo = (int16_t*)ps_malloc(MP3_PCM_MAX_SAMPLES * 2 * sizeof(int16_t));
  if (!inBuf || !pcm || !stereo) {
    if (inBuf) free(inBuf);
    if (pcm) free(pcm);
    if (stereo) free(stereo);
    MP3Decoder_FreeBuffers();
    http.end();
    wakeTtsStatus = "MP3 Puffer Speicher fehlt.";
    diagLog("TTS_MP3", wakeTtsStatus);
    return false;
  }

  Client *stream = http.getStreamPtr();
  size_t bytesIn = 0;
  uint32_t lastReadMs = millis();
  uint32_t lastLogMs = millis();
  uint32_t lastFrameMs = millis();
  uint32_t lastStallLogMs = millis();
  uint32_t started = millis();
  uint32_t frames = 0;
  uint32_t decodeErrors = 0;
  uint32_t underflows = 0;
  int currentRate = 0;

  // Per-second worst cases, so a stall can be attributed instead of guessed at.
  uint32_t maxWriteMs = 0;
  uint32_t maxPumpMs = 0;
  uint32_t maxIterMs = 0;
  uint32_t secStreamBytes = 0;
  uint32_t iterations = 0;
  uint32_t noSyncSpins = 0;
  uint32_t iterStart = 0;

  bool playbackStarted = false;
  bool playbackPrepared = false;
  bool timedOut = false;
  bool streamEnded = false;
  bool idleDone = false;

  ttsPlaybackActive = true;
  diagLog("TTS_MP3", "decoder loop active; playback waits for first valid frame");

  while ((uint32_t)(millis() - started) < TTS_MP3_TIMEOUT_MS) {
    uint32_t iterNow = millis();
    if (iterStart != 0) {
      uint32_t d = iterNow - iterStart;
      if (d > maxIterMs) maxIterMs = d;
    }
    iterStart = iterNow;
    iterations++;

    while (bytesIn < MP3_INPUT_BUFFER_SIZE - 1024 && stream && stream->available() > 0) {
      size_t room = MP3_INPUT_BUFFER_SIZE - bytesIn;
      size_t available = (size_t)stream->available();
      size_t want = min<size_t>(room, min<size_t>(available, TTS_MP3_READ_CHUNK_MAX));
      if (want == 0) break;
      int n = stream->read(inBuf + bytesIn, want);
      if (n <= 0) break;
      bytesIn += (size_t)n;
      secStreamBytes += (size_t)n;
      lastReadMs = millis();
    }

    int sync = MP3FindSyncWord(inBuf, (int)bytesIn);
    if (sync < 0) {
      noSyncSpins++;
      if (bytesIn > 2048) {
        memmove(inBuf, inBuf + bytesIn - 1024, 1024);
        bytesIn = 1024;
      }

      if (stream && !stream->connected() && stream->available() == 0) {
        streamEnded = true;
        break;
      }
      if (playbackStarted && (uint32_t)(millis() - lastFrameMs) > TTS_MP3_IDLE_DONE_MS) {
        idleDone = true;
        diagLogf("TTS_MP3", "idle done no-sync frames=%lu in=%lu readIdle=%lu",
                 (unsigned long)frames,
                 (unsigned long)bytesIn,
                 (unsigned long)(millis() - lastReadMs));
        break;
      }
      if ((uint32_t)(millis() - lastReadMs) > TTS_MP3_IDLE_DONE_MS) {
        idleDone = playbackStarted;
        diagLogf("TTS_MP3", "idle break no-sync playback=%u in=%lu",
                 playbackStarted ? 1 : 0,
                 (unsigned long)bytesIn);
        break;
      }
      ledTick();
      pumpServices();
      delay(2);
      yield();
      continue;
    }

    if (sync > 0) {
      memmove(inBuf, inBuf + sync, bytesIn - sync);
      bytesIn -= sync;
    }

    int before = (int)bytesIn;
    int left = before;
    int err = MP3Decode(inBuf, &left, pcm, 0);
    int consumed = before - left;
    if (consumed > 0 && left >= 0) {
      memmove(inBuf, inBuf + consumed, left);
      bytesIn = (size_t)left;
    }

    if (err == ERR_MP3_INDATA_UNDERFLOW) {
      underflows++;
      if (playbackStarted && (uint32_t)(millis() - lastReadMs) > TTS_MP3_IDLE_DONE_MS &&
          (uint32_t)(millis() - lastFrameMs) > TTS_MP3_IDLE_DONE_MS) {
        idleDone = true;
        diagLogf("TTS_MP3", "idle done underflow frames=%lu in=%lu under=%lu",
                 (unsigned long)frames,
                 (unsigned long)bytesIn,
                 (unsigned long)underflows);
        break;
      }
      if ((uint32_t)(millis() - lastStallLogMs) > TTS_MP3_STALL_LOG_MS) {
        lastStallLogMs = millis();
        diagLogf("TTS_MP3", "underflow wait frames=%lu in=%lu readIdle=%lu frameIdle=%lu under=%lu",
                 (unsigned long)frames,
                 (unsigned long)bytesIn,
                 (unsigned long)(millis() - lastReadMs),
                 (unsigned long)(millis() - lastFrameMs),
                 (unsigned long)underflows);
      }
      if (bytesIn > 0 && bytesIn < MP3_INPUT_BUFFER_SIZE - 1024) {
        ledTick();
        pumpServices();
        delay(2);
        yield();
        continue;
      }
    }

    if (err != ERR_MP3_NONE && err != ERR_MP3_MAINDATA_UNDERFLOW) {
      decodeErrors++;
      if (consumed <= 0 && bytesIn > 1) {
        memmove(inBuf, inBuf + 1, bytesIn - 1);
        bytesIn--;
      }
      if (decodeErrors < 8 || (decodeErrors % 20) == 0) {
        diagLogf("TTS_MP3", "decode err=%d errors=%lu bytesIn=%lu",
                 err,
                 (unsigned long)decodeErrors,
                 (unsigned long)bytesIn);
      }
      ledTick();
      pumpServices();
      yield();
      continue;
    }

    int rate = MP3GetSampRate();
    int channels = MP3GetChannels();
    int samples = MP3GetOutputSamps();
    if (rate < 8000 || rate > 48000 || (channels != 1 && channels != 2) ||
        samples <= 0 || samples > (int)MP3_PCM_MAX_SAMPLES) {
      decodeErrors++;
      continue;
    }

    if (!playbackPrepared) {
      currentRate = rate;
      playbackPrepared = beginTtsPlayback((uint32_t)currentRate, "TTS_MP3");
      if (!playbackPrepared) {
        ttsWriteFailures++;
        diagLog("TTS_MP3", "playback prepare failed");
      }
      diagLogf("TTS_MP3", "rate=%d channels=%d prepared=%u",
               currentRate,
               channels,
               playbackPrepared ? 1 : 0);
    } else if (rate != currentRate) {
      currentRate = rate;
      bool rateOk = audioSetSampleRate(scaledPlaybackRate((uint32_t)currentRate));
      audioClearTx();
      audioWriteSilenceBlocks(2, 80);
      diagLogf("TTS_MP3", "rate switch=%d channels=%d ok=%u",
               currentRate,
               channels,
               rateOk ? 1 : 0);
    }

    size_t bytesToWrite = 0;
    const void *writeBuf = nullptr;
    if (channels == 1) {
      for (int i = 0; i < samples; i++) {
        stereo[i * 2] = pcm[i];
        stereo[i * 2 + 1] = pcm[i];
      }
      bytesToWrite = (size_t)samples * 2 * sizeof(int16_t);
      writeBuf = stereo;
    } else {
      bytesToWrite = (size_t)samples * sizeof(int16_t);
      writeBuf = pcm;
    }

    size_t bytesWritten = 0;
    uint32_t writeStart = millis();
    bool writeOk = playbackPrepared &&
                   audioWrite(writeBuf, bytesToWrite, &bytesWritten, 250) == ESP_OK &&
                   bytesWritten == bytesToWrite;
    uint32_t writeMs = millis() - writeStart;
    if (writeMs > maxWriteMs) maxWriteMs = writeMs;

    if (writeOk) {
      playbackStarted = true;
      ttsOutputBytes += bytesWritten;
      frames++;
      lastFrameMs = millis();
    } else {
      ttsWriteFailures++;
    }

    if ((uint32_t)(millis() - lastLogMs) > 1000) {
      lastLogMs = millis();
      diagLogf("TTS_MP3", "frames=%lu out=%lu in=%lu net=%lu it=%lu wMax=%lu pMax=%lu iMax=%lu err=%lu und=%lu wf=%lu",
               (unsigned long)frames,
               (unsigned long)ttsOutputBytes,
               (unsigned long)bytesIn,
               (unsigned long)secStreamBytes,
               (unsigned long)iterations,
               (unsigned long)maxWriteMs,
               (unsigned long)maxPumpMs,
               (unsigned long)maxIterMs,
               (unsigned long)decodeErrors,
               (unsigned long)underflows,
               (unsigned long)ttsWriteFailures);
      diagLogf("TTS_MP3", "  spins noSync=%lu conn=%u avail=%d",
               (unsigned long)noSyncSpins,
               (stream && stream->connected()) ? 1 : 0,
               stream ? stream->available() : -1);
      noSyncSpins = 0;
      maxWriteMs = 0;
      maxPumpMs = 0;
      maxIterMs = 0;
      secStreamBytes = 0;
      iterations = 0;
    }

    ledTick();
    if ((frames & 0x07) == 0) {
      uint32_t pumpStart = millis();
      pumpServices();
      uint32_t pumpMs = millis() - pumpStart;
      if (pumpMs > maxPumpMs) maxPumpMs = pumpMs;
    }
    yield();

    if (stream && !stream->connected() && stream->available() == 0 && bytesIn < 512) {
      streamEnded = true;
      break;
    }
  }

  if ((uint32_t)(millis() - started) >= TTS_MP3_TIMEOUT_MS) timedOut = true;

  if (playbackPrepared) {
    finishTtsPlayback("TTS_MP3");
  } else {
    audioSetSampleRate(AUDIO_SAMPLE_RATE);
    setAmplifier(false);
    ttsPlaybackActive = false;
    wakeIgnoreUntil = millis() + WAKE_COOLDOWN_MS;
    diagLog("TTS_MP3", "playback never prepared; cooldown set");
  }

  free(inBuf);
  free(pcm);
  free(stereo);
  MP3Decoder_FreeBuffers();
  http.end();

  bool finished = playbackStarted && (!timedOut || idleDone || streamEnded);
  wakeTtsStatus = finished
                  ? "MP3 abgespielt: " + String(frames) + " Frames"
                  : (timedOut ? "MP3 Timeout." : "MP3-Wiedergabe abgebrochen.");
  diagLogf("TTS_MP3", "end finished=%u timedOut=%u streamEnd=%u idleDone=%u frames=%lu out=%lu errors=%lu under=%lu failures=%lu status=%s",
           finished ? 1 : 0,
           timedOut ? 1 : 0,
           streamEnded ? 1 : 0,
           idleDone ? 1 : 0,
           (unsigned long)frames,
           (unsigned long)ttsOutputBytes,
           (unsigned long)decodeErrors,
           (unsigned long)underflows,
           (unsigned long)ttsWriteFailures,
           wakeTtsStatus.c_str());
  return finished;
}

bool fetchAndPlayTtsUrl(const String &ttsUrl) {
  diagLog("TTS_FETCH", String("input=") + ttsUrl);
  if (ttsUrl.length() == 0) {
    wakeTtsStatus = "Keine TTS-URL von Home Assistant.";
    diagLog("TTS_FETCH", wakeTtsStatus);
    return false;
  }

  if (!audioI2sReady || !codecPlaybackReady) {
    wakeTtsStatus = "Playback ist nicht bereit.";
    diagLog("TTS_FETCH", wakeTtsStatus);
    return false;
  }

  String url = absoluteHaMediaUrl(ttsUrl);
  diagLog("TTS_FETCH", String("absolute=") + url);
  String lowerUrl = url;
  lowerUrl.toLowerCase();
  if (lowerUrl.endsWith(".mp3")) {
    return playMp3TtsUrl(url);
  }

  wakeTtsStatus = "TTS wird abgerufen.";

  HTTPClient http;
  WiFiClientSecure secureClient;
  bool secure = url.startsWith("https://");
  if (secure) {
    secureClient.setInsecure();
    if (!http.begin(secureClient, url)) {
      wakeTtsStatus = "HTTPS TTS begin fehlgeschlagen.";
      diagLog("TTS_FETCH", wakeTtsStatus);
      return false;
    }
  } else if (!http.begin(url)) {
    wakeTtsStatus = "HTTP TTS begin fehlgeschlagen.";
    diagLog("TTS_FETCH", wakeTtsStatus);
    return false;
  }

  http.useHTTP10(true);  // no chunked framing in the raw stream we read from
  http.setTimeout(15000);
  http.addHeader("Authorization", "Bearer " + cfg.haToken);
  int code = http.GET();
  String contentType = http.header("Content-Type");
  contentType.toLowerCase();
  diagLogf("TTS_FETCH", "probe code=%d type=%s size=%d",
           code,
           contentType.c_str(),
           http.getSize());

  if (code != HTTP_CODE_OK) {
    wakeTtsStatus = "TTS HTTP " + String(code);
    diagLog("TTS_FETCH", wakeTtsStatus);
    http.end();
    return false;
  }

  if (contentType.indexOf("mpeg") >= 0 ||
      contentType.indexOf("mp3") >= 0) {
    http.end();
    return playMp3TtsUrl(url);
  }

  if (contentType.indexOf("ogg") >= 0 ||
      contentType.indexOf("opus") >= 0) {
    wakeTtsStatus = "TTS Format " + contentType + " braucht noch Decoder.";
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();
  bool ok = stream && playWavPcm16(*stream);
  http.end();
  return ok;
}

// -----------------------------------------------------------------------------
// Assist pipeline list
//
// The voice is a property of the pipeline in Home Assistant, not something the
// device can override per request. Fetching the list lets the user pick a
// pipeline by name instead of pasting an id, which is the practical way to
// switch voices from here.
// -----------------------------------------------------------------------------

// Returns the substring of the object starting at `from`, brace balanced.
static String jsonObjectAt(const String &json, int from, int &endOut) {
  int depth = 0;
  bool inString = false;
  bool esc = false;

  for (int i = from; i < (int)json.length(); i++) {
    char c = json[i];
    if (esc) { esc = false; continue; }
    if (c == '\\') { esc = true; continue; }
    if (c == '"') { inString = !inString; continue; }
    if (inString) continue;

    if (c == '{') depth++;
    else if (c == '}') {
      depth--;
      if (depth == 0) {
        endOut = i + 1;
        return json.substring(from, i + 1);
      }
    }
  }

  endOut = json.length();
  return "";
}

static void parsePipelineList(const String &msg) {
  pipelineCount = 0;

  int arrayAt = msg.indexOf("\"pipelines\":[");
  if (arrayAt < 0) return;

  pipelinePreferred = jsonFindString(msg, "preferred_pipeline");

  int pos = arrayAt;
  while (pipelineCount < PIPELINE_MAX) {
    int objAt = msg.indexOf('{', pos);
    if (objAt < 0) break;

    int objEnd = 0;
    String obj = jsonObjectAt(msg, objAt, objEnd);
    if (obj.length() == 0) break;

    String id = jsonFindString(obj, "id");
    String name = jsonFindString(obj, "name");
    if (id.length() > 0) {
      pipelineId[pipelineCount] = id;
      pipelineName[pipelineCount] = name.length() > 0 ? name : id;
      String voice = jsonFindString(obj, "tts_voice");
      String engine = jsonFindString(obj, "tts_engine");
      pipelineVoice[pipelineCount] = voice.length() > 0
                                     ? (engine + " / " + voice)
                                     : engine;
      pipelineCount++;
    }

    pos = objEnd;
    if (msg.indexOf(']', pos) >= 0 && msg.indexOf('{', pos) > msg.indexOf(']', pos)) break;
  }
}

// Opens a short WebSocket session just to list the pipelines. Runs from loop().
static void fetchPipelineList() {
  if (WiFi.status() != WL_CONNECTED || cfg.haUrl.isEmpty() || cfg.haToken.isEmpty()) {
    pipelineListStatus = "Home Assistant nicht konfiguriert";
    return;
  }

  String host;
  uint16_t port = 0;
  bool secure = false;
  if (!parseHaUrl(cfg.haUrl, host, port, secure)) {
    pipelineListStatus = "Home-Assistant-URL ist ungültig";
    return;
  }

  WiFiClient plain;
  WiFiClientSecure tls;
  Client *client = nullptr;
  if (secure) {
    tls.setInsecure();
    client = &tls;
  } else {
    client = &plain;
  }

  if (!wsConnectHa(*client, host, port, cfg.haToken)) {
    client->stop();
    pipelineListStatus = "WS/Auth fehlgeschlagen";
    diagLog("PIPELINE", pipelineListStatus);
    return;
  }

  wsSendText(*client, "{\"id\":1,\"type\":\"assist_pipeline/pipeline/list\"}");

  String msg;
  uint8_t opcode = 0;
  bool got = false;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < 6000) {
    if (!wsReadFrame(*client, msg, opcode, 6000)) break;
    if (opcode == 0x8) break;
    if (msg.indexOf("\"pipelines\"") >= 0) {
      parsePipelineList(msg);
      got = true;
      break;
    }
    pumpServices();
  }

  client->stop();

  pipelineListStatus = got
                       ? String(pipelineCount) + " Pipelines gefunden"
                       : "keine Antwort von Home Assistant";
  diagLogf("PIPELINE", "%s", pipelineListStatus.c_str());

  for (uint8_t i = 0; i < pipelineCount; i++) {
    diagLogf("PIPELINE", "  %s = %s (%s)%s",
             pipelineId[i].c_str(),
             pipelineName[i].c_str(),
             pipelineVoice[i].c_str(),
             pipelineId[i] == pipelinePreferred ? " [Standard]" : "");
  }
}

// -----------------------------------------------------------------------------
// Wake acknowledgement
//
// Short spoken clips ("Ja bitte", "Was gibt's?", ...) played straight after the
// wake word. They are rendered once by the user's own Home Assistant TTS and
// cached in LittleFS, so the confirmation uses the same voice as the answers
// and needs no network once it is stored.
// -----------------------------------------------------------------------------

// Asks Home Assistant to render a phrase and returns a URL we can actually
// reach. Shared by the cached acknowledgements and by /api/announce.
static bool haRenderTts(const String &phrase, String &urlOut, String &errOut) {
  if (WiFi.status() != WL_CONNECTED || cfg.haUrl.isEmpty() || cfg.haToken.isEmpty()) {
    errOut = "Home Assistant nicht konfiguriert";
    return false;
  }
  if (haTtsEngine.isEmpty()) {
    errOut = "TTS-Engine noch unbekannt - bitte erst einmal eine Frage stellen";
    return false;
  }

  HTTPClient http;
  if (!http.begin(normalizedHaUrl(cfg.haUrl) + "/api/tts_get_url")) {
    errOut = "HTTP-Client konnte nicht geöffnet werden";
    return false;
  }

  http.useHTTP10(true);
  http.setTimeout(15000);
  http.addHeader("Authorization", "Bearer " + cfg.haToken);
  http.addHeader("Content-Type", "application/json");

  // Engine, language and voice all have to be repeated - without them HA falls
  // back to the engine default, which is not what the pipeline uses.
  String body = "{\"engine_id\":\"" + jsonEscape(haTtsEngine) + "\"";
  body += ",\"message\":\"" + jsonEscape(phrase) + "\"";
  if (haTtsLanguage.length() > 0) {
    body += ",\"language\":\"" + jsonEscape(haTtsLanguage) + "\"";
  }
  if (haTtsVoice.length() > 0) {
    body += ",\"options\":{\"voice\":\"" + jsonEscape(haTtsVoice) + "\"}";
  }
  body += "}";

  int code = http.POST(body);
  String reply = code == HTTP_CODE_OK ? http.getString() : String("");
  http.end();

  if (code != HTTP_CODE_OK) {
    errOut = "HA antwortet mit HTTP " + String(code);
    return false;
  }

  // Prefer "path": the absolute "url" is built from HA's own internal/external
  // URL setting, which the board often cannot resolve.
  String url = jsonFindString(reply, "path");
  if (url.length() == 0) url = jsonFindString(reply, "url");
  if (url.length() == 0) {
    errOut = "keine URL in der Antwort";
    return false;
  }

  urlOut = absoluteHaMediaUrl(url);
  return true;
}

static String ackClipPathExt(uint8_t index, const char *ext) {
  return "/ack" + String(index) + "." + ext;
}

// Google delivers MP3, Piper delivers WAV. Store whatever came and remember it
// by extension instead of forcing one container.
static String ackClipPath(uint8_t index) {
  String mp3 = ackClipPathExt(index, "mp3");
  if (LittleFS.exists(mp3)) return mp3;

  String wav = ackClipPathExt(index, "wav");
  if (LittleFS.exists(wav)) return wav;

  return "";
}

uint8_t ackScanClips() {
  ackClipCount = 0;
  if (!ackFsReady) return 0;

  for (uint8_t i = 0; i < ACK_MAX_CLIPS; i++) {
    if (ackClipPath(i).length() == 0) break;
    ackClipCount++;
  }
  return ackClipCount;
}

// Splits the configured pipe separated phrase list.
static uint8_t ackPhraseAt(const String &list, uint8_t index, String &out) {
  int start = 0;
  uint8_t seen = 0;
  while (start <= (int)list.length()) {
    int sep = list.indexOf('|', start);
    String part = sep < 0 ? list.substring(start) : list.substring(start, sep);
    part.trim();
    if (part.length() > 0) {
      if (seen == index) {
        out = part;
        return 1;
      }
      seen++;
    }
    if (sep < 0) break;
    start = sep + 1;
  }
  return 0;
}

static uint8_t ackPhraseCount(const String &list) {
  uint8_t n = 0;
  String tmp;
  while (n < ACK_MAX_CLIPS && ackPhraseAt(list, n, tmp)) n++;
  return n;
}

// Fallback when no clip is cached: a short two tone chime, always available.
static void playAckChime() {
  if (!beginTtsPlayback(AUDIO_SAMPLE_RATE, "ACK")) return;

  const uint16_t toneMs[2] = {90, 120};
  const float toneHz[2] = {880.0f, 1320.0f};
  int16_t buf[128 * 2];

  for (uint8_t t = 0; t < 2; t++) {
    uint32_t total = (AUDIO_SAMPLE_RATE * toneMs[t]) / 1000;
    float step = 2.0f * PI * toneHz[t] / (float)AUDIO_SAMPLE_RATE;
    uint32_t done = 0;

    while (done < total) {
      uint32_t n = min<uint32_t>(128, total - done);
      for (uint32_t i = 0; i < n; i++) {
        uint32_t pos = done + i;
        float env = 1.0f;
        if (pos < 200) env = (float)pos / 200.0f;
        if (total - pos < 400) env = (float)(total - pos) / 400.0f;
        int16_t v = (int16_t)(sinf((float)pos * step) * 5000.0f * env);
        buf[i * 2] = v;
        buf[i * 2 + 1] = v;
      }
      size_t written = 0;
      audioWrite(buf, n * 2 * sizeof(int16_t), &written, 120);
      done += n;
      ledTick();
      yield();
    }
  }

  finishTtsPlayback("ACK");
}

// Decodes a cached MP3 straight off the filesystem. Much simpler than the
// network path: a file never stalls, so there is no jitter handling here.
static bool playMp3File(const String &path) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    diagLogf("ACK", "clip %s not readable", path.c_str());
    return false;
  }

  if (!MP3Decoder_AllocateBuffers()) {
    f.close();
    diagLog("ACK", "no memory for the MP3 decoder");
    return false;
  }

  uint8_t *inBuf = (uint8_t*)ps_malloc(MP3_INPUT_BUFFER_SIZE);
  int16_t *pcm = (int16_t*)ps_malloc(MP3_PCM_MAX_SAMPLES * sizeof(int16_t));
  int16_t *stereo = (int16_t*)ps_malloc(MP3_PCM_MAX_SAMPLES * 2 * sizeof(int16_t));
  if (!inBuf || !pcm || !stereo) {
    if (inBuf) free(inBuf);
    if (pcm) free(pcm);
    if (stereo) free(stereo);
    MP3Decoder_FreeBuffers();
    f.close();
    diagLog("ACK", "no memory for the MP3 buffers");
    return false;
  }

  size_t bytesIn = 0;
  uint32_t frames = 0;
  int currentRate = 0;
  bool prepared = false;

  ttsPlaybackActive = true;

  while (true) {
    while (bytesIn < MP3_INPUT_BUFFER_SIZE - 1024 && f.available()) {
      size_t room = MP3_INPUT_BUFFER_SIZE - bytesIn;
      int n = f.read(inBuf + bytesIn, min<size_t>(room, TTS_MP3_READ_CHUNK_MAX));
      if (n <= 0) break;
      bytesIn += (size_t)n;
    }

    if (bytesIn == 0) break;

    int sync = MP3FindSyncWord(inBuf, (int)bytesIn);
    if (sync < 0) {
      if (!f.available()) break;
      if (bytesIn > 2048) {
        memmove(inBuf, inBuf + bytesIn - 1024, 1024);
        bytesIn = 1024;
      }
      continue;
    }
    if (sync > 0) {
      memmove(inBuf, inBuf + sync, bytesIn - sync);
      bytesIn -= sync;
    }

    int before = (int)bytesIn;
    int left = before;
    int err = MP3Decode(inBuf, &left, pcm, 0);
    int consumed = before - left;
    if (consumed > 0 && left >= 0) {
      memmove(inBuf, inBuf + consumed, left);
      bytesIn = (size_t)left;
    }

    if (err == ERR_MP3_INDATA_UNDERFLOW && !f.available()) break;
    if (err != ERR_MP3_NONE && err != ERR_MP3_MAINDATA_UNDERFLOW) {
      if (consumed <= 0 && bytesIn > 1) {
        memmove(inBuf, inBuf + 1, bytesIn - 1);
        bytesIn--;
      }
      continue;
    }

    int rate = MP3GetSampRate();
    int channels = MP3GetChannels();
    int samples = MP3GetOutputSamps();
    if (rate < 8000 || rate > 48000 || (channels != 1 && channels != 2) ||
        samples <= 0 || samples > (int)MP3_PCM_MAX_SAMPLES) {
      continue;
    }

    if (!prepared) {
      currentRate = rate;
      prepared = beginTtsPlayback((uint32_t)currentRate, "ACK");
      if (!prepared) break;
    } else if (rate != currentRate) {
      currentRate = rate;
      audioSetSampleRate(scaledPlaybackRate((uint32_t)currentRate));
    }

    size_t bytesToWrite;
    const void *writeBuf;
    if (channels == 1) {
      for (int i = 0; i < samples; i++) {
        stereo[i * 2] = pcm[i];
        stereo[i * 2 + 1] = pcm[i];
      }
      bytesToWrite = (size_t)samples * 2 * sizeof(int16_t);
      writeBuf = stereo;
    } else {
      bytesToWrite = (size_t)samples * sizeof(int16_t);
      writeBuf = pcm;
    }

    size_t written = 0;
    if (audioWrite(writeBuf, bytesToWrite, &written, 250) == ESP_OK) frames++;

    ledTick();
    yield();
  }

  if (prepared) finishTtsPlayback("ACK");
  else ttsPlaybackActive = false;

  free(inBuf);
  free(pcm);
  free(stereo);
  MP3Decoder_FreeBuffers();
  f.close();

  diagLogf("ACK", "played %s frames=%lu", path.c_str(), (unsigned long)frames);
  return frames > 0;
}

// Picks one of the cached clips at random, or falls back to the chime.
bool playAckSound() {
  if (!cfg.ackEnabled) return false;
  if (!audioI2sReady || !codecPlaybackReady) return false;

  setLedPhase(LED_PHASE_SPEAK);

  bool ok = false;
  if (ackFsReady && ackClipCount > 0) {
    uint8_t pick = (uint8_t)(esp_random() % ackClipCount);
    String path = ackClipPath(pick);

    if (path.endsWith(".wav")) {
      // playWavPcm16 takes any Stream, and File is one.
      File f = LittleFS.open(path, "r");
      if (f) {
        ttsPlaybackActive = true;
        ok = playWavPcm16(f);
        f.close();
        diagLogf("ACK", "played %s (wav)", path.c_str());
      }
    } else if (path.length() > 0) {
      ok = playMp3File(path);
    }
  }

  if (!ok) playAckChime();

  // The confirmation is ours, it must not push the recording into a cooldown.
  wakeIgnoreUntil = millis();
  return true;
}

// One-shot announcement: render, play, done. Nothing is cached, the text comes
// from whoever called the API - a doorbell automation, for instance.
static bool playAnnouncement(const String &rawText) {
  String url;
  String err;

  // Leading "[name.mp3]" tokens are played first, in the order given, and
  // then whatever text is left gets spoken.
  String text = rawText;
  text.trim();

  bool playedSound = false;
  while (text.startsWith("[")) {
    int close = text.indexOf(']');
    if (close < 0) break;

    String name = text.substring(1, close);
    name.trim();
    text = text.substring(close + 1);
    text.trim();

    setLedPhase(LED_PHASE_SPEAK);
    if (playSoundFile(name)) playedSound = true;
    else diagLogf("ANNOUNCE", "sound \"%s\" skipped", name.c_str());
  }

  if (text.isEmpty()) {
    // Sound only, nothing to say.
    wakeIgnoreUntil = millis();
    announceStatus = playedSound ? "Klang abgespielt" : "nichts abzuspielen";
    return playedSound;
  }

  if (!haRenderTts(text, url, err)) {
    announceStatus = err;
    diagLogf("ANNOUNCE", "failed: %s", err.c_str());
    return false;
  }

  diagLogf("ANNOUNCE", "playing \"%s\"", text.c_str());
  setLedPhase(LED_PHASE_SPEAK);
  bool ok = fetchAndPlayTtsUrl(url);

  // An announcement is not a conversation, so it must not start a cooldown.
  wakeIgnoreUntil = millis();
  announceStatus = ok ? "abgespielt: " + text : "Wiedergabe fehlgeschlagen: " + wakeTtsStatus;
  return ok;
}

// Renders every configured phrase through Home Assistant's TTS and stores the
// result. Runs from loop(), it needs several seconds and a lot of network.
static void buildAckCache() {
  if (!ackFsReady) {
    ackBuildStatus = "LittleFS nicht bereit";
    return;
  }
  if (WiFi.status() != WL_CONNECTED || cfg.haUrl.isEmpty() || cfg.haToken.isEmpty()) {
    ackBuildStatus = "Home Assistant nicht konfiguriert";
    return;
  }
  if (haTtsEngine.isEmpty()) {
    ackBuildStatus = "TTS-Engine noch unbekannt - bitte erst einmal eine Frage stellen";
    return;
  }

  uint8_t want = ackPhraseCount(cfg.ackPhrases);
  if (want == 0) {
    ackBuildStatus = "keine Phrasen konfiguriert";
    return;
  }

  for (uint8_t i = 0; i < ACK_MAX_CLIPS; i++) {
    if (LittleFS.exists(ackClipPathExt(i, "mp3"))) LittleFS.remove(ackClipPathExt(i, "mp3"));
    if (LittleFS.exists(ackClipPathExt(i, "wav"))) LittleFS.remove(ackClipPathExt(i, "wav"));
  }
  ackClipCount = 0;

  uint8_t done = 0;

  for (uint8_t i = 0; i < want; i++) {
    String phrase;
    if (!ackPhraseAt(cfg.ackPhrases, i, phrase)) break;

    String url;
    String err;
    if (!haRenderTts(phrase, url, err)) {
      diagLogf("ACK", "render failed for \"%s\": %s", phrase.c_str(), err.c_str());
      ackBuildStatus = err;
      pumpServices();
      continue;
    }
    diagLogf("ACK", "download %s", url.c_str());

    HTTPClient dl;
    WiFiClientSecure dlSecure;
    bool dlOk;
    if (url.startsWith("https://")) {
      dlSecure.setInsecure();
      dlOk = dl.begin(dlSecure, url);
    } else {
      dlOk = dl.begin(url);
    }

    if (!dlOk) {
      diagLog("ACK", "download client could not be opened");
      continue;
    }

    dl.useHTTP10(true);
    dl.setTimeout(15000);
    dl.addHeader("Authorization", "Bearer " + cfg.haToken);
    int dlCode = dl.GET();

    if (dlCode != HTTP_CODE_OK) {
      diagLogf("ACK", "clip download failed http=%d url=%s", dlCode, url.c_str());
      ackBuildStatus = "Download fehlgeschlagen (HTTP " + String(dlCode) + ")";
      dl.end();
      continue;
    }

    String lower = url;
    lower.toLowerCase();
    const char *ext = lower.indexOf(".wav") >= 0 ? "wav" : "mp3";
    String outPath = ackClipPathExt(done, ext);

    File out = LittleFS.open(outPath, "w");
    if (!out) {
      dl.end();
      continue;
    }

    Client *stream = dl.getStreamPtr();
    uint8_t buf[1024];
    size_t total = 0;
    uint32_t lastData = millis();

    while (total < ACK_MAX_CLIP_BYTES) {
      int avail = stream ? stream->available() : 0;
      if (avail > 0) {
        int n = stream->read(buf, min<size_t>(sizeof(buf), (size_t)avail));
        if (n <= 0) break;
        out.write(buf, (size_t)n);
        total += (size_t)n;
        lastData = millis();
      } else {
        if (!stream || (!stream->connected() && stream->available() == 0)) break;
        if ((uint32_t)(millis() - lastData) > 6000) break;
        delay(2);
        pumpServices();
      }
    }

    out.close();
    dl.end();

    if (total < 512) {
      LittleFS.remove(outPath);
      diagLogf("ACK", "clip \"%s\" too small (%lu bytes)", phrase.c_str(), (unsigned long)total);
      continue;
    }

    diagLogf("ACK", "cached \"%s\" -> %s (%lu bytes)",
             phrase.c_str(), outPath.c_str(), (unsigned long)total);
    done++;
    pumpServices();
  }

  ackClipCount = done;
  ackBuildStatus = done > 0
                   ? String(done) + " von " + String(want) + " Ansagen gespeichert"
                   : "keine Ansage konnte erzeugt werden";
  diagLog("ACK", ackBuildStatus);
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

// Blocks until at least `want` bytes are buffered, or the deadline passes.
static bool wsWaitBytes(Client &client, int want, uint32_t start, uint32_t timeoutMs) {
  while (client.available() < want) {
    if (!client.connected() && client.available() < want) return false;
    if ((uint32_t)(millis() - start) > timeoutMs) return false;
    ledTick();
    delay(1);
    yield();
  }
  return true;
}

bool wsReadFrame(Client &client, String &text, uint8_t &opcode, uint32_t timeoutMs) {
  uint32_t start = millis();
  text = "";
  opcode = 0;

  while (client.connected() && client.available() < 2) {
    if ((uint32_t)(millis() - start) > timeoutMs) {
      diagLogf("WS_READ", "timeout waiting header timeout=%lu connected=%u",
               (unsigned long)timeoutMs,
               client.connected() ? 1 : 0);
      return false;
    }
    ledTick();
    delay(2);
    yield();
  }

  if (client.available() < 2) {
    diagLogf("WS_READ", "no header available connected=%u", client.connected() ? 1 : 0);
    return false;
  }

  uint8_t b0 = client.read();
  uint8_t b1 = client.read();
  opcode = b0 & 0x0F;
  bool masked = (b1 & 0x80) != 0;
  uint64_t len = b1 & 0x7F;

  if (len == 126) {
    if (!wsWaitBytes(client, 2, start, timeoutMs)) {
      diagLog("WS_READ", "timeout reading 16 bit length");
      return false;
    }
    len = ((uint16_t)client.read() << 8) | client.read();
  } else if (len == 127) {
    len = 0;
    if (!wsWaitBytes(client, 8, start, timeoutMs)) {
      diagLog("WS_READ", "timeout reading 64 bit length");
      return false;
    }
    for (int i = 0; i < 8; i++) len = (len << 8) | client.read();
  }

  uint8_t mask[4] = {0, 0, 0, 0};
  if (masked) {
    if (!wsWaitBytes(client, 4, start, timeoutMs)) {
      diagLog("WS_READ", "timeout reading mask");
      return false;
    }
    for (uint8_t i = 0; i < 4; i++) mask[i] = client.read();
  }

  // Read every byte so the stream stays in sync, but only keep what we can
  // reasonably hold - and keep the *beginning*, which is where the event type
  // and the interesting fields sit.
  size_t keep = (size_t)min<uint64_t>(len, (uint64_t)WS_FRAME_KEEP_MAX);
  if (len > WS_FRAME_KEEP_MAX) {
    diagLogf("WS_READ", "frame truncated opcode=%u len=%lu keep=%lu",
             opcode,
             (unsigned long)len,
             (unsigned long)keep);
  }

  text.reserve(keep + 1);
  for (uint64_t i = 0; i < len; i++) {
    while (!client.available()) {
      if ((uint32_t)(millis() - start) > timeoutMs) {
        diagLogf("WS_READ", "timeout payload opcode=%u len=%lu read=%lu",
                 opcode,
                 (unsigned long)len,
                 (unsigned long)i);
        return false;
      }
      ledTick();
      delay(1);
      yield();
    }
    char c = (char)client.read();
    if (masked) c ^= mask[i & 3];
    if (i < keep) text += c;
  }

  return true;
}

bool wsConnectHa(Client &client, const String &host, uint16_t port, const String &token) {
  wakeLastWsStage = "connect";
  wakeLastWsDetail = host + ":" + String(port);
  wakeLastHttpCode = 0;
  client.setTimeout(5000);
  diagLogf("WS", "connect %s:%u", host.c_str(), port);

  if (!client.connect(host.c_str(), port)) {
    wakeLastWsDetail = "TCP connect failed: " + host + ":" + String(port);
    diagLog("WS", wakeLastWsDetail);
    return false;
  }

  String req;
  req.reserve(420 + token.length());
  req += "GET /api/websocket HTTP/1.1\r\n";
  req += "Host: " + host + ":" + String(port) + "\r\n";
  req += "Upgrade: websocket\r\n";
  req += "Connection: Upgrade\r\n";
  req += "Sec-WebSocket-Key: Vm9pY2VEb3RWMDQyVGVzdA==\r\n";
  req += "Sec-WebSocket-Version: 13\r\n";
  req += "User-Agent: VoiceDot/" + String(FW_VERSION) + "\r\n\r\n";

  wakeLastWsStage = "http-upgrade";
  diagLog("WS", "http-upgrade send");
  client.print(req);

  String line;
  String firstLine;
  uint32_t start = millis();
  bool upgraded = false;
  while ((uint32_t)(millis() - start) < 5000) {
    line = client.readStringUntil('\n');
    line.trim();
    if (firstLine.isEmpty() && line.startsWith("HTTP/")) {
      firstLine = line;
      int sp = line.indexOf(' ');
      if (sp > 0) wakeLastHttpCode = line.substring(sp + 1).toInt();
    }
    if (line.startsWith("HTTP/1.1 101") || line.startsWith("HTTP/1.0 101")) upgraded = true;
    if (line.length() == 0 && upgraded) break;
  }

  if (!upgraded) {
    wakeLastWsDetail = firstLine.length() > 0 ? firstLine : "No HTTP upgrade response";
    diagLog("WS", String("upgrade failed ") + wakeLastWsDetail);
    return false;
  }

  wakeLastWsDetail = firstLine.length() > 0 ? firstLine : "HTTP 101";
  diagLog("WS", String("upgrade ok ") + wakeLastWsDetail);

  String msg;
  uint8_t opcode = 0;
  wakeLastWsStage = "auth-required";
  if (!wsReadFrame(client, msg, opcode, 5000)) {
    wakeLastWsDetail = "No auth_required frame";
    diagLog("WS", wakeLastWsDetail);
    return false;
  }
  if (msg.indexOf("auth_required") < 0) {
    wakeLastWsDetail = msg.substring(0, min(180, (int)msg.length()));
    diagLog("WS", String("unexpected auth frame ") + wakeLastWsDetail);
    return false;
  }

  wakeLastWsStage = "auth";
  diagLog("WS", "auth send");
  wsSendText(client, "{\"type\":\"auth\",\"access_token\":\"" + token + "\"}");
  if (!wsReadFrame(client, msg, opcode, 5000)) {
    wakeLastWsDetail = "No auth response frame";
    diagLog("WS", wakeLastWsDetail);
    return false;
  }

  wakeLastWsDetail = msg.substring(0, min(180, (int)msg.length()));
  if (msg.indexOf("auth_ok") < 0) {
    diagLog("WS", String("auth failed ") + wakeLastWsDetail);
    return false;
  }

  wakeLastWsStage = "auth-ok";
  diagLog("WS", "auth ok");
  return true;
}

// Mono RMS level of one frame, in dBFS.
static float frameLevelDb(const int16_t *mono, size_t samples, int16_t *peakOut) {
  if (samples == 0) return -96.0f;

  uint64_t sumSquares = 0;
  int32_t peak = 0;

  for (size_t i = 0; i < samples; i++) {
    int32_t v = mono[i];
    int32_t a = v < 0 ? -v : v;
    if (a > peak) peak = a;
    sumSquares += (uint64_t)(v * v);
  }

  if (peakOut) *peakOut = (int16_t)constrain(peak, 0, 32767);

  float rms = sqrtf((float)sumSquares / (float)samples);
  return rms > 1.0f ? 20.0f * log10f(rms / 32768.0f) : -96.0f;
}

// -----------------------------------------------------------------------------
// Assist turn, streamed
//
// The pipeline is opened *before* recording and every captured frame goes
// straight out over the WebSocket. Speech-to-text therefore runs while the user
// is still talking, instead of starting only once the recording is complete.
// Measured against the buffered version this removes the upload from the
// critical path and lets a slow cloud STT overlap with the speech itself.
//
// Home Assistant runs its own VAD on the incoming stream. If it decides the
// utterance is over before we do, stt-end arrives while we are still recording
// and we stop right there - which is faster than our own silence timer.
// -----------------------------------------------------------------------------

static bool assistFlush(AssistStream &st) {
  if (st.pending == 0 || st.failed) return !st.failed;

  if (!wsSendFrame(*st.client, 0x2, st.frame, st.pending + 1)) {
    st.failed = true;
    diagLog("ASSIST", "audio stream write failed");
    return false;
  }

  wakeLastBytes += st.pending;
  st.pending = 0;
  return true;
}

static bool assistWrite(AssistStream &st, const uint8_t *data, size_t len) {
  if (st.failed) return false;

  while (len > 0) {
    size_t room = sizeof(st.frame) - 1 - st.pending;
    size_t chunk = min<size_t>(room, len);
    memcpy(st.frame + 1 + st.pending, data, chunk);
    st.pending += chunk;
    data += chunk;
    len -= chunk;

    if (st.pending == sizeof(st.frame) - 1 && !assistFlush(st)) return false;
  }
  return true;
}

// Consumes whatever Home Assistant has already sent, without blocking the
// recording. Returns true when stt-end arrived, meaning HA is done listening.
static bool assistPollEvents(AssistStream &st, bool &errorOut) {
  bool sttEnded = false;
  String msg;
  uint8_t opcode = 0;

  while (st.client && st.client->available() >= 2) {
    if (!wsReadFrame(*st.client, msg, opcode, 500)) break;
    if (opcode == 0x8) {
      errorOut = true;
      break;
    }

    updateAssistFromWsMessage(msg);
    String eventType = assistEventType(msg);

    if (eventType == "stt-end") sttEnded = true;
    if (eventType == "error") {
      wakeLastMessage = "HA Assist Fehler: " + msg.substring(0, min(180, (int)msg.length()));
      diagLog("ASSIST", wakeLastMessage);
      errorOut = true;
      break;
    }
  }

  return sttEnded;
}

// Opens the WebSocket, authenticates and starts the pipeline. On success the
// stream is ready to take audio.
static bool assistOpen(AssistStream &st, Client *client) {
  st.client = client;
  st.handlerId = -1;
  st.pending = 0;
  st.failed = false;

  if (WiFi.status() != WL_CONNECTED) {
    wakeLastMessage = "WLAN ist nicht verbunden.";
    diagLog("ASSIST", wakeLastMessage);
    return false;
  }

  if (cfg.haUrl.isEmpty() || cfg.haToken.isEmpty()) {
    wakeLastMessage = "Home-Assistant-URL oder Token fehlt.";
    diagLog("ASSIST", wakeLastMessage);
    return false;
  }

  String host;
  uint16_t port = 0;
  bool secure = false;
  if (!parseHaUrl(cfg.haUrl, host, port, secure)) {
    wakeLastMessage = "Home-Assistant-URL ist ungültig.";
    diagLog("ASSIST", wakeLastMessage);
    return false;
  }

  wakeSending = true;
  wakeLastState = "connecting";
  wakeLastMessage = "Verbinde mit Home Assistant.";
  wakeLastHttpCode = 0;
  wakeLastWsStage = "init";
  wakeLastWsDetail = host + ":" + String(port) + (secure ? " https" : " http");
  wakeTranscript = "";
  wakeAssistantText = "";
  wakeTtsUrl = "";
  wakeTtsStatus = "-";
  wakeLastBytes = 0;
  localCommandHandled = false;
  wakeAnnounceText = "";
  setLedPhase(LED_PHASE_THINK);

  if (!wsConnectHa(*client, host, port, cfg.haToken)) {
    wakeLastMessage = "HA WS/Auth fehlgeschlagen bei " + wakeLastWsStage + ": " + wakeLastWsDetail;
    diagLog("ASSIST", wakeLastMessage);
    return false;
  }

  wakeLastHttpCode = 101;
  wakeLastWsStage = "pipeline-start";
  wakeLastWsDetail = "auth ok";

  if (haConversationId.length() > 0 &&
      (uint32_t)(millis() - haConversationSeenAt) > CONVERSATION_TTL_MS) {
    diagLog("ASSIST", "conversation expired, starting a new one");
    haConversationId = "";
  }

  String run = "{\"id\":1,\"type\":\"assist_pipeline/run\"";
  run += ",\"start_stage\":\"stt\",\"end_stage\":\"tts\"";
  // Home Assistant defaults every one of these to off, which leaves a weak
  // signal weak: its own voice-activity detection then ends the sentence the
  // moment the speaker's level dips into the room noise, and the recogniser
  // gets a fragment. Sending them is what its own voice satellites do.
  run += ",\"input\":{\"sample_rate\":16000";
  run += ",\"noise_suppression_level\":" + String(cfg.sttNoiseSuppression);
  run += ",\"auto_gain_dbfs\":" + String(cfg.sttAutoGainDb);
  run += ",\"volume_multiplier\":" + String(cfg.sttVolumePercent / 100.0f, 2);

  // This device already decides when a sentence has ended, with a pre-roll
  // buffer and a threshold measured against the speaker's own level. Home
  // Assistant's own detector runs on top of that and is the cruder of the two -
  // measured, it cut a three second question after 1.4 s and the recogniser was
  // handed a fragment. Ours stops the stream; HA transcribes what it gets.
  if (!cfg.sttHaVad) run += ",\"no_vad\":true";

  run += "}";
  run += ",\"timeout\":30";
  if (cfg.haPipeline.length() > 0) {
    run += ",\"pipeline\":\"" + jsonEscape(cfg.haPipeline) + "\"";
  }
  if (haConversationId.length() > 0) {
    run += ",\"conversation_id\":\"" + jsonEscape(haConversationId) + "\"";
  }
  run += "}";

  diagLogf("ASSIST", "pipeline/run send pipeline=%s conversation=%s ns=%u agc=%u vol=%u%% haVad=%u",
           cfg.haPipeline.length() > 0 ? cfg.haPipeline.c_str() : "default",
           haConversationId.length() > 0 ? haConversationId.c_str() : "new",
           cfg.sttNoiseSuppression, cfg.sttAutoGainDb, cfg.sttVolumePercent,
           cfg.sttHaVad ? 1 : 0);
  wsSendText(*client, run);

  String msg;
  uint8_t opcode = 0;
  uint32_t waitStart = millis();

  while ((uint32_t)(millis() - waitStart) < 8000) {
    if (!wsReadFrame(*client, msg, opcode, 8000)) break;
    if (opcode == 0x8) break;

    updateAssistFromWsMessage(msg);

    int id = jsonFindInt(msg, "stt_binary_handler_id", -1);
    if (id >= 0 && id <= 255) {
      st.handlerId = id;
      st.frame[0] = (uint8_t)id;
      wakeLastWsStage = "audio-stream";
      diagLogf("ASSIST", "handler=%d, ready to stream", id);
      return true;
    }

    if (assistEventType(msg) == "error") {
      wakeLastMessage = "HA meldet Fehler vor Audio: " + msg.substring(0, min(180, (int)msg.length()));
      diagLog("ASSIST", wakeLastMessage);
      return false;
    }
  }

  wakeLastMessage = "HA hat keinen STT-Audiokanal geliefert.";
  wakeLastWsStage = "stt-handler";
  diagLog("ASSIST", wakeLastMessage);
  return false;
}

// Records with VAD and streams every frame as it arrives. Returns false when
// nothing was spoken or the stream broke.
static bool assistRecordAndStream(AssistStream &st) {
  if (!audioI2sReady || !codecRecordReady) {
    wakeLastMessage = "Mikrofon ist nicht bereit.";
    diagLog("WAKE_REC", wakeLastMessage);
    return false;
  }

  uint8_t *preroll = nullptr;
  size_t prerollFill = 0;
  size_t prerollHead = 0;
  if (cfg.vadEnabled) {
    preroll = (uint8_t*)malloc(WAKE_PREROLL_BYTES);
    if (!preroll) diagLog("WAKE_REC", "pre-roll buffer unavailable, continuing without it");
  }

  wakeRecording = true;
  wakeLastState = cfg.vadEnabled ? "listening" : "recording";
  wakeLastMessage = cfg.vadEnabled ? "Ich höre zu." : "Aufnahme läuft.";
  wakeLastVadSilenceMs = 0;
  setLedPhase(LED_PHASE_LISTEN);

  int16_t stereo[AUDIO_FRAME_SAMPLES * 2];
  int16_t mono[AUDIO_FRAME_SAMPLES];
  size_t bytesRead = 0;
  for (uint8_t i = 0; i < 24; i++) {
    if (audioRead(stereo, sizeof(stereo), &bytesRead, 0) != ESP_OK || bytesRead == 0) break;
  }

  const uint32_t started = millis();
  uint32_t lastLog = started;
  uint32_t lastVoiceMs = started;
  uint32_t speechStartedAt = started;
  bool speechStarted = !cfg.vadEnabled;
  float noiseFloorDb = VAD_NOISE_FLOOR_START_DB;
  float speechPeakDb = -96.0f;
  float lastRelease = -96.0f;
  float calibrationSum = 0.0f;
  uint32_t calibrationFrames = 0;
  uint32_t voiceFrames = 0;
  bool haStopped = false;
  bool haError = false;

  bool needCalibration = true;
  if (srRoomNoiseValid && (uint32_t)(started - srRoomNoiseAt) < SR_ROOM_NOISE_TTL_MS) {
    noiseFloorDb = srRoomNoiseDb;
    needCalibration = false;
    diagLogf("WAKE_REC", "noise floor seeded from detector: %d dBFS", (int)noiseFloorDb);
  }

  while (true) {
    uint32_t now = millis();
    uint32_t elapsed = now - started;

    if (!speechStarted && elapsed > WAKE_SPEECH_TIMEOUT_MS) break;
    if (speechStarted && !cfg.vadEnabled && elapsed >= WAKE_RECORD_MS) break;
    if (speechStarted && cfg.vadEnabled &&
        (uint32_t)(now - speechStartedAt) >= WAKE_MAX_RECORD_MS) {
      diagLog("WAKE_REC", "max length reached");
      break;
    }
    if (st.failed || haError) break;
    if (haStopped) {
      diagLog("WAKE_REC", "Home Assistant ended the utterance first");
      break;
    }

    esp_err_t err = audioRead(stereo, sizeof(stereo), &bytesRead, 60);
    if (err == ESP_OK && bytesRead >= sizeof(int16_t) * 2) {
      size_t stereoSamples = bytesRead / sizeof(int16_t);
      size_t monoSamples = 0;
      for (size_t i = 0; i + 1 < stereoSamples; i += 2) {
        mono[monoSamples++] = (int16_t)(((int32_t)stereo[i] + (int32_t)stereo[i + 1]) / 2);
      }

      size_t monoBytes = monoSamples * sizeof(int16_t);
      int16_t peak = 0;
      float db = frameLevelDb(mono, monoSamples, &peak);

      micPeak = peak;
      micDb = db;
      micLevel = constrain((int)map(peak, 0, 4000, 0, 100), 0, 100);
      micReadCount++;
      micLastBytes = bytesRead;

      bool calibrating = needCalibration && !speechStarted &&
                         (elapsed < VAD_CALIBRATE_MS || calibrationFrames < 4);
      if (calibrating) {
        calibrationSum += db;
        calibrationFrames++;
        noiseFloorDb = calibrationSum / (float)calibrationFrames;
      } else if (db < noiseFloorDb) {
        noiseFloorDb += (db - noiseFloorDb) * 0.25f;
      } else {
        noiseFloorDb += (db - noiseFloorDb) * 0.002f;
      }

      if (noiseFloorDb > VAD_NOISE_FLOOR_MAX_DB) noiseFloorDb = VAD_NOISE_FLOOR_MAX_DB;
      wakeNoiseFloorDb = noiseFloorDb;

      if (!speechStarted) {
        float threshold = noiseFloorDb + VAD_SPEECH_MARGIN_DB;
        if (threshold < VAD_MIN_SPEECH_DB) threshold = VAD_MIN_SPEECH_DB;

        if (!calibrating && db > threshold) {
          speechStarted = true;
          speechStartedAt = now;
          lastVoiceMs = now;
          wakeLastState = "recording";
          wakeLastMessage = "Aufnahme läuft.";
          diagLogf("WAKE_REC", "speech start db=%d floor=%d after=%lu",
                   (int)db, (int)noiseFloorDb, (unsigned long)elapsed);

          // The pre-roll goes out first so the opening syllable is not lost.
          if (preroll && prerollFill > 0) {
            size_t oldest = prerollFill < WAKE_PREROLL_BYTES ? 0 : prerollHead;
            assistWrite(st, preroll + oldest, prerollFill - oldest);
            if (oldest > 0) assistWrite(st, preroll, oldest);
          }

          assistWrite(st, (const uint8_t*)mono, monoBytes);
        } else if (preroll) {
          size_t remaining = monoBytes;
          const uint8_t *src = (const uint8_t*)mono;
          while (remaining > 0) {
            size_t chunk = min<size_t>(remaining, WAKE_PREROLL_BYTES - prerollHead);
            memcpy(preroll + prerollHead, src, chunk);
            prerollHead = (prerollHead + chunk) % WAKE_PREROLL_BYTES;
            src += chunk;
            remaining -= chunk;
            if (prerollFill < WAKE_PREROLL_BYTES) {
              prerollFill = min<size_t>(WAKE_PREROLL_BYTES, prerollFill + chunk);
            }
          }
        }
      } else {
        assistWrite(st, (const uint8_t*)mono, monoBytes);

        if (db > speechPeakDb) speechPeakDb = db;
        else speechPeakDb = speechPeakDb * 0.999f + db * 0.001f;

        float release = speechPeakDb - VAD_SPEECH_DROP_DB;
        float floorGuard = noiseFloorDb + (float)cfg.vadReleaseDb;
        if (release < floorGuard) release = floorGuard;
        lastRelease = release;

        if (db > release) {
          voiceFrames++;

          // Move the silence clock back instead of zeroing it, so a single
          // noise spike cannot erase a second of accumulated quiet.
          uint32_t accumulated = now - lastVoiceMs;
          lastVoiceMs += min<uint32_t>(accumulated, VAD_BLIP_CREDIT_MS);
        }

        if (cfg.vadEnabled) {
          wakeLastVadSilenceMs = now - lastVoiceMs;
          uint32_t spoken = now - speechStartedAt;
          if (wakeLastVadSilenceMs >= cfg.vadSilenceMs && spoken >= WAKE_MIN_RECORD_MS) {
            diagLogf("WAKE_REC", "silence stop spoken=%lu silence=%lu voice=%lu peak=%d rel=%d",
                     (unsigned long)spoken,
                     (unsigned long)wakeLastVadSilenceMs,
                     (unsigned long)voiceFrames,
                     (int)speechPeakDb,
                     (int)release);
            break;
          }
        }
      }
    }

    // Keep the socket drained: Home Assistant reports stt-start, VAD events and
    // errors while we are still sending.
    if (speechStarted && assistPollEvents(st, haError)) haStopped = true;

    pumpServices();
    ledTick();

    if ((uint32_t)(millis() - lastLog) > 1000) {
      lastLog = millis();
      diagLogf("WAKE_REC", "streaming sent=%lu db=%d floor=%d rel=%d silence=%lu",
               (unsigned long)wakeLastBytes,
               (int)micDb,
               (int)noiseFloorDb,
               (int)lastRelease,
               (unsigned long)wakeLastVadSilenceMs);
    }
  }

  if (preroll) free(preroll);

  assistFlush(st);

  wakeRecording = false;
  wakeLastDurationMs = millis() - started;

  if (!speechStarted) {
    wakeLastMessage = "Nichts gehört. Bitte direkt nach dem Tastendruck sprechen.";
    diagLogf("WAKE_REC", "no speech floor=%d duration=%lu",
             (int)noiseFloorDb, (unsigned long)wakeLastDurationMs);
    return false;
  }

  if (st.failed) {
    wakeLastMessage = "Audio-Upload zu HA abgebrochen.";
    return false;
  }

  diagLogf("WAKE_REC", "done sent=%lu duration=%lu voice=%lu floor=%d rel=%d",
           (unsigned long)wakeLastBytes,
           (unsigned long)wakeLastDurationMs,
           (unsigned long)voiceFrames,
           (int)noiseFloorDb,
           (int)lastRelease);
  return true;
}

// Closes the audio channel and waits for the answer, then plays it.
static bool assistFinish(AssistStream &st) {
  uint8_t endFrame[1] = {(uint8_t)st.handlerId};
  wsSendFrame(*st.client, 0x2, endFrame, 1);
  diagLog("ASSIST", "audio end frame sent");

  wakeSending = true;
  wakeLastState = "sending";
  wakeLastMessage = "Warte auf Home Assistant.";
  setLedPhase(LED_PHASE_THINK);

  bool ok = false;
  String msg;
  uint8_t opcode = 0;
  uint32_t resultStart = millis();
  uint32_t lastWaitLog = resultStart;

  while ((uint32_t)(millis() - resultStart) < 30000) {
    if (!wsReadFrame(*st.client, msg, opcode, 30000)) break;
    if (opcode == 0x8) break;

    updateAssistFromWsMessage(msg);
    String eventType = assistEventType(msg);

    if (eventType == "error") {
      wakeLastMessage = "HA Assist Fehler: " + msg.substring(0, min(180, (int)msg.length()));
      diagLog("ASSIST", wakeLastMessage);
      break;
    }

    if (eventType == "run-end" || msg.indexOf("\"type\":\"result\"") >= 0) {
      wakeLastWsStage = "done";
      ok = true;
      diagLog("ASSIST", "run done");
      break;
    }

    pumpServices();
    ledTick();

    if ((uint32_t)(millis() - lastWaitLog) > 3000) {
      lastWaitLog = millis();
      diagLogf("ASSIST", "waiting result stage=%s transcript=%u assistant=%u ttsUrl=%u",
               wakeLastWsStage.c_str(),
               wakeTranscript.length(),
               wakeAssistantText.length(),
               wakeTtsUrl.length());
    }
    yield();
  }

  st.client->stop();
  wakeSending = false;

  diagLogf("ASSIST", "ws closed ok=%u stage=%s ttsUrl=%s",
           ok ? 1 : 0, wakeLastWsStage.c_str(), wakeTtsUrl.c_str());

  if (!ok) {
    if (wakeLastMessage.length() == 0 || wakeLastMessage == "Warte auf Home Assistant.") {
      wakeLastState = (wakeTranscript.length() > 0 || wakeAssistantText.length() > 0 ||
                       wakeTtsUrl.length() > 0) ? "partial" : "error";
      wakeLastMessage = "HA Assist hat nicht rechtzeitig geantwortet.";
    } else {
      wakeLastState = "error";
    }
    return false;
  }

  if (localCommandHandled) {
    wakeLastState = "done";
    setLedPhase(LED_PHASE_SPEAK);

    if (wakeAnnounceText.length() > 0) {
      String say = wakeAnnounceText;
      wakeAnnounceText = "";
      wakeAssistantText = say;
      wakeTtsStatus = "lokaler Befehl, eigene Ansage";
      playAnnouncement(say);
      diagLog("ASSIST", "local command answered locally");
      return true;
    }

    wakeTtsStatus = "lokaler Befehl, keine TTS";
    playAckChime();
    diagLog("ASSIST", "local command handled, HA answer skipped");
    return true;
  }

  if (wakeTtsUrl.length() > 0) {
    wakeLastState = "tts";
    wakeLastWsStage = "tts-playback";
    wakeLastMessage = "Spiele HA TTS-Antwort.";
    setLedPhase(LED_PHASE_SPEAK);

    String playUrl = wakeTtsUrl;

    if (cfg.cleanMarkdown && wakeAssistantText.length() > 0) {
      String clean = stripMarkdownForSpeech(wakeAssistantText);
      if (clean.length() > 0 && clean != wakeAssistantText) {
        String freshUrl;
        String err;
        if (haRenderTts(clean, freshUrl, err)) {
          playUrl = freshUrl;
          diagLogf("TTS_CLEAN", "re-rendered without markup (%u -> %u Zeichen)",
                   wakeAssistantText.length(), clean.length());
        } else {
          diagLogf("TTS_CLEAN", "re-render failed (%s), playing HA audio", err.c_str());
        }
      }
    }

    bool ttsOk = fetchAndPlayTtsUrl(playUrl);
    wakeLastState = ttsOk ? "done" : "partial";
    wakeLastMessage = ttsOk
                      ? "HA Assist fertig, TTS abgespielt."
                      : "HA Assist fertig, TTS nicht abgespielt: " + wakeTtsStatus;
    return true;
  }

  wakeLastState = "done";
  wakeTtsStatus = "Keine TTS-URL von Home Assistant.";
  if (wakeAssistantText.length() > 0) wakeLastMessage = "HA Assist fertig: " + wakeAssistantText;
  else if (wakeTranscript.length() > 0) wakeLastMessage = "HA Assist fertig: " + wakeTranscript;
  else wakeLastMessage = "HA Assist fertig.";
  return true;
}

// One complete turn: open, stream, answer.
static bool runAssistStreamingTurn(bool playAck) {
  WiFiClient plainClient;
  WiFiClientSecure secureClient;

  bool secure = normalizedHaUrl(cfg.haUrl).startsWith("https://");
  Client *client;
  if (secure) {
    secureClient.setInsecure();
    client = &secureClient;
  } else {
    client = &plainClient;
  }

  AssistStream st;

  // Opening the pipeline first costs about half a second. Doing it before the
  // acknowledgement hides that behind a sound the user is listening to anyway.
  if (!assistOpen(st, client)) {
    client->stop();
    wakeSending = false;
    return false;
  }

  wakeSending = false;

  if (playAck) playAckSound();

  if (!assistRecordAndStream(st)) {
    uint8_t endFrame[1] = {(uint8_t)st.handlerId};
    if (!st.failed) wsSendFrame(*st.client, 0x2, endFrame, 1);
    client->stop();
    return false;
  }

  return assistFinish(st);
}

// Queues an Assist turn. The turn itself always runs from loop(), never from
// inside a web handler or a button poll, so nothing re-enters the web server.
void requestWake(const char *source) {
  wakeRequested = true;
  wakeRequestSource = source;
  diagLogf("WAKE", "requested by %s", source);
}

void runWakeCaptureAndHa() {
  if (!wakeCanStart()) {
    diagLogf("WAKE", "ignored busy=%u cooldownLeft=%ld",
             wakeBusy ? 1 : 0,
             (long)((int32_t)(wakeIgnoreUntil - millis())));
    return;
  }

  diagLogf("WAKE", "start source=%s", wakeRequestSource);
  wakeBusy = true;
  srPauseDetection();
  radioPauseForTurn();
  wakeLastState = "wake";
  wakeLastMessage = "Wake erkannt.";
  wakeTranscript = "";
  wakeAssistantText = "";
  wakeTtsUrl = "";
  wakeTtsStatus = "-";

  // No confirmation on a follow-up turn: the assistant just asked something,
  // answering it with "Ja bitte" would talk over its own question.
  bool isFollowUpTurn = strcmp(wakeRequestSource, "continue_conversation") == 0;
  bool playAck = cfg.ackEnabled && !isFollowUpTurn;

  bool ok = runAssistStreamingTurn(playAck);

  if (!ok && wakeLastState != "error" && wakeLastState != "partial") wakeLastState = "error";

  // Home Assistant asks for another turn when the intent expects an answer.
  bool followUp = ok && cfg.followUp && haContinueConversation &&
                  !apMode && WiFi.status() == WL_CONNECTED;
  haContinueConversation = false;

  // A local command can have a question of its own - "which station?" - and
  // then wants the next utterance, exactly like a follow-up from Home
  // Assistant. Whether we actually listen respects the same setting.
  if (radioAskPending) {
    radioAskPending = false;
    wakeAssistantText = radioAskText;
    setLedPhase(LED_PHASE_SPEAK);
    playAnnouncement(radioAskText);

    if (radioAnswerUntil != 0 && cfg.followUp && !apMode &&
        WiFi.status() == WL_CONNECTED) {
      followUp = true;
    } else {
      radioAnswerUntil = 0;
    }
  }

  if (wakeLastState == "error") {
    setLedPhase(LED_PHASE_ERROR);
  } else if (!followUp) {
    setLedPhase(LED_PHASE_IDLE);
  }

  wakeIgnoreUntil = millis() + (followUp ? FOLLOW_UP_COOLDOWN_MS : WAKE_COOLDOWN_MS);
  k2RawPressed = false;
  k2StablePressed = false;
  k2LongHandled = true;
  k2ChangedAt = millis();
  wakeBusy = false;

  if (followUp) {
    wakeLastMessage = "Antwort erwartet, ich höre weiter zu.";
    requestWake("continue_conversation");
  } else {
    // Re-arm only once the answer has finished playing, so the speaker cannot
    // trigger the detector with our own TTS.
    srResumeDetection();
    radioResumeAfterTurn();
  }

  diagLogf("WAKE", "end ok=%u followUp=%u state=%s",
           ok ? 1 : 0,
           followUp ? 1 : 0,
           wakeLastState.c_str());
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
    LED_COLOR_ORDER
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

void waitRingTick(uint32_t now, bool force) {
  if (!pixels) return;
  if (!force && (uint32_t)(now - waitRingLastMs) < 95) return;

  waitRingLastMs = now;
  waitRingPosition = (waitRingPosition + 1) % 7;

  pixels->clear();
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t distance = (i + 7 - waitRingPosition) % 7;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    if (distance == 0) {
      r = 255; g = 96; b = 0;
    } else if (distance == 1) {
      r = 130; g = 42; b = 0;
    } else if (distance == 2) {
      r = 45; g = 12; b = 0;
    }

    pixels->setPixelColor(i, pixels->Color(r, g, b));
  }
  pixels->show();
}

// Level meter for the listening phase.
//
// The scale is logarithmic on purpose: micLevel is a linear peak mapped to
// 0..4000, and speech blows straight through that, so a linear bar only ever
// shows full or empty. Working in dB between the measured room noise and
// -14 dBFS spreads normal speech across all seven LEDs.
//
// Ballistics are the classic ones for a meter: fast attack, slow release, plus
// a peak marker that sinks back down slowly.
void listenRingTick(uint32_t now, bool force) {
  if (!pixels) return;
  if (!force && (uint32_t)(now - waitRingLastMs) < 40) return;  // 25 fps
  waitRingLastMs = now;

  float floorDb = wakeNoiseFloorDb;
  if (floorDb < -75.0f) floorDb = -75.0f;
  if (floorDb > -35.0f) floorDb = -35.0f;

  const float bottomDb = floorDb + 4.0f;   // just above the room
  const float topDb = -14.0f;              // loud, close speech

  float level = (micDb - bottomDb) / (topDb - bottomDb);
  if (level < 0.0f) level = 0.0f;
  if (level > 1.0f) level = 1.0f;

  if (level > eqLevel) eqLevel += (level - eqLevel) * 0.55f;
  else eqLevel += (level - eqLevel) * 0.16f;

  float r0 = (float)((cfg.listenColor >> 16) & 0xFF);
  float g0 = (float)((cfg.listenColor >> 8) & 0xFF);
  float b0 = (float)(cfg.listenColor & 0xFF);

  // The whole ring shares one brightness, so it pulses as a single light
  // instead of filling up segment by segment. The floor keeps it visible
  // during pauses, the hue stays put because only the level is scaled.
  float amount = 0.12f + 0.88f * eqLevel;

  uint8_t r = (uint8_t)(r0 * amount);
  uint8_t g = (uint8_t)(g0 * amount);
  uint8_t b = (uint8_t)(b0 * amount);

  for (uint8_t i = 0; i < 7; i++) {
    pixels->setPixelColor(i, pixels->Color(r, g, b));
  }
  pixels->show();
}

// Breathing ring while the assistant talks. The colour comes from the config,
// the pulse rides from a dim floor up to the full value so the hue stays put.
void speakRingTick(uint32_t now, bool force) {
  if (!pixels) return;
  if (!force && (uint32_t)(now - waitRingLastMs) < 60) return;
  waitRingLastMs = now;

  ledPulseStep = (ledPulseStep + 1) % 50;
  float phase = (1.0f - cosf((float)ledPulseStep * (2.0f * PI / 50.0f))) * 0.5f;
  float level = 0.30f + 0.70f * phase;

  uint8_t r = (uint8_t)(((cfg.speakColor >> 16) & 0xFF) * level);
  uint8_t g = (uint8_t)(((cfg.speakColor >> 8) & 0xFF) * level);
  uint8_t b = (uint8_t)((cfg.speakColor & 0xFF) * level);

  for (uint8_t i = 0; i < 7; i++) {
    pixels->setPixelColor(i, pixels->Color(r, g, b));
  }
  pixels->show();
}

// Radio is running: one dim dot walking slowly around the ring. Distinct from
// listening (whole ring pulsing) and speaking (whole ring breathing), and quiet
// enough to live with for an hour.
void radioRingTick(uint32_t now, bool force) {
  if (!pixels) return;
  if (!force && (uint32_t)(now - waitRingLastMs) < 220) return;
  waitRingLastMs = now;

  waitRingPosition = (waitRingPosition + 1) % 7;

  uint8_t r = (uint8_t)((((cfg.listenColor >> 16) & 0xFF) * 35) / 100);
  uint8_t g = (uint8_t)((((cfg.listenColor >> 8) & 0xFF) * 35) / 100);
  uint8_t b = (uint8_t)(((cfg.listenColor & 0xFF) * 35) / 100);

  pixels->clear();
  pixels->setPixelColor(waitRingPosition, pixels->Color(r, g, b));
  pixels->show();
}

// Two soft blinks in the listening colour, then the ring goes dark and hands
// back to the idle dot. Self-terminating, so it never blocks the loop while the
// other device takes over the conversation.
void yieldRingTick(uint32_t now, bool force) {
  if (!pixels) return;
  if (!force && (uint32_t)(now - waitRingLastMs) < 25) return;
  waitRingLastMs = now;

  uint32_t elapsed = now - ledPhaseSince;
  if (elapsed >= LED_YIELD_TOTAL_MS) {
    setLedPhase(LED_PHASE_IDLE);
    return;
  }

  // Sine envelope rather than a hard on/off - the ring fades away instead of
  // flashing, which reads as backing off rather than as an error.
  float level = 0.0f;
  const uint32_t cycle = LED_YIELD_BLINK_MS + LED_YIELD_GAP_MS;
  if (elapsed < 2 * cycle) {
    uint32_t inBlink = elapsed % cycle;
    if (inBlink < LED_YIELD_BLINK_MS) {
      level = sinf((float)inBlink * PI / (float)LED_YIELD_BLINK_MS);
    }
  }

  uint8_t r = (uint8_t)(((cfg.listenColor >> 16) & 0xFF) * level);
  uint8_t g = (uint8_t)(((cfg.listenColor >> 8) & 0xFF) * level);
  uint8_t b = (uint8_t)((cfg.listenColor & 0xFF) * level);

  for (uint8_t i = 0; i < 7; i++) {
    pixels->setPixelColor(i, pixels->Color(r, g, b));
  }
  pixels->show();
}

void ledsStatusIdle() {
  if (apMode) ledsStatusSetup();
  else ledsStatusReady();
}

// Why the device last restarted. Without this a crash and a deliberate reboot
// look exactly alike from the outside, and guessing which one happened has
// already cost enough time.
static String resetReasonName() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON:   return "Einschalten";
    case ESP_RST_EXT:       return "externer Reset";
    case ESP_RST_SW:        return "Neustart per Software";
    case ESP_RST_PANIC:     return "Absturz";
    case ESP_RST_INT_WDT:   return "Interrupt-Watchdog";
    case ESP_RST_TASK_WDT:  return "Task-Watchdog";
    case ESP_RST_WDT:       return "Watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep";
    case ESP_RST_BROWNOUT:  return "Unterspannung";
    case ESP_RST_SDIO:      return "SDIO";
    default:
      switch ((int)reason) {
        case 11: return "USB-Reset";
        case 12: return "JTAG-Reset";
        case 13: return "eFuse-Fehler";
        case 14: return "Spannungseinbruch";
        case 15: return "CPU-Blockade";
        default: return "sonstiger Grund (" + String((int)reason) + ")";
      }
  }
}

// Name of the current ring animation, for the status page and for checking
// from outside that a self-terminating phase really did terminate.
static const char *ledPhaseName() {
  switch (ledPhase) {
    case LED_PHASE_LISTEN: return "listen";
    case LED_PHASE_THINK:  return "think";
    case LED_PHASE_SPEAK:  return "speak";
    case LED_PHASE_RADIO:  return "radio";
    case LED_PHASE_YIELD:  return "yield";
    case LED_PHASE_ERROR:  return "error";
    default:               return "idle";
  }
}

void setLedPhase(LedPhase phase) {
  ledPhase = phase;
  ledPhaseSince = millis();
  ledPulseStep = 0;
  waitRingLastMs = 0;

  if (phase == LED_PHASE_LISTEN) {
    eqLevel = 0.0f;
  }

  switch (phase) {
    case LED_PHASE_LISTEN: listenRingTick(millis(), true); break;
    case LED_PHASE_THINK:  waitRingTick(millis(), true); break;
    case LED_PHASE_SPEAK:  speakRingTick(millis(), true); break;
    case LED_PHASE_RADIO:  radioRingTick(millis(), true); break;
    case LED_PHASE_YIELD:  yieldRingTick(millis(), true); break;
    case LED_PHASE_ERROR:  ledsStatusError(); break;
    default:               ledsStatusIdle(); break;
  }
}

// Called from every long running loop so the ring keeps animating while the
// main loop is busy with audio or HTTP work.
void ledTick(bool force) {
  uint32_t now = millis();

  switch (ledPhase) {
    case LED_PHASE_LISTEN: listenRingTick(now, force); break;
    case LED_PHASE_THINK:  waitRingTick(now, force); break;
    case LED_PHASE_SPEAK:  speakRingTick(now, force); break;
    case LED_PHASE_RADIO:  radioRingTick(now, force); break;
    case LED_PHASE_YIELD:  yieldRingTick(now, force); break;
    case LED_PHASE_ERROR:
      if ((uint32_t)(now - ledPhaseSince) > 1600) setLedPhase(LED_PHASE_IDLE);
      break;
    default:
      break;
  }
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

// Idle is a standby indicator, not a lamp: a single dim LED is enough and does
// not light up the room.
void ledsStatusReady() {
  if (!pixels) return;

  pixels->clear();
  pixels->setPixelColor(0, pixels->Color(0, 90, 0));
  pixels->show();
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

  if (!wakeCanStart()) {
    static uint32_t lastK2BlockLog = 0;
    if ((uint32_t)(now - lastK2BlockLog) > 1000) {
      lastK2BlockLog = now;
      diagLogf("K2", "blocked pressed=%u busy=%u cooldownLeft=%ld raw=%u stable=%u",
               pressedNow ? 1 : 0,
               wakeBusy ? 1 : 0,
               (long)((int32_t)(wakeIgnoreUntil - now)),
               k2RawPressed ? 1 : 0,
               k2StablePressed ? 1 : 0);
    }
    k2RawPressed = pressedNow;
    k2StablePressed = pressedNow;
    k2LongHandled = true;
    k2ChangedAt = now;
    if (pressedNow) k2PressedAt = now;
    return;
  }

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
      Serial.printf("K2 short press: wake/record held=%lu\n", (unsigned long)held);
      diagLogf("K2", "short held=%lu", (unsigned long)held);
      requestWake("K2");
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
  WiFi.setSleep(false);
  WiFi.setHostname(deviceHostname().c_str());
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());

  uint32_t started = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - started < 15000) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    logPrintf("WiFi: connected to %s, IP %s, %d dBm",
              cfg.wifiSsid.c_str(),
              WiFi.localIP().toString().c_str(),
              (int)WiFi.RSSI());
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

  logPrintf("Setup AP: %s", ssid.c_str());
  Serial.printf("Password: %s\n", AP_PASSWORD);
  Serial.println("URL: http://192.168.4.1");
}

void startMdns() {
  if (apMode) return;

  String host = deviceHostname();
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "device", cfg.deviceName.c_str());
    MDNS.addServiceTxt("http", "tcp", "fw", FW_VERSION);

    // A dedicated service type so Home Assistant can discover VoiceDots
    // specifically, instead of sifting through every HTTP device in the network.
    char id[13];
    uint64_t chip = ESP.getEfuseMac();
    snprintf(id, sizeof(id), "%012llx", (unsigned long long)chip);

    MDNS.addService("voicedot", "tcp", 80);
    MDNS.addServiceTxt("voicedot", "tcp", "id", (const char *)id);
    MDNS.addServiceTxt("voicedot", "tcp", "device", cfg.deviceName.c_str());
    MDNS.addServiceTxt("voicedot", "tcp", "fw", FW_VERSION);
    MDNS.addServiceTxt("voicedot", "tcp", "api", "1");

    logPrintf("mDNS: http://%s.local (_voicedot._tcp, id=%s)", host.c_str(), id);
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
 --text:#f4f7fb;--muted:#95a3b5;--accent:#79a6ff;--side:#10151d;
 --green:#47d885;--red:#ff6b75;--orange:#ffb454;--blue:#79a6ff
}
:root[data-theme="light"]{
 --bg:#eef1f6;--card:#ffffff;--card2:#f4f6fa;--line:#d5dce6;
 --text:#16202c;--muted:#5f6f81;--accent:#2563eb;--side:#ffffff;
 --green:#1f9d55;--red:#d64550;--orange:#b8710c;--blue:#2563eb
}
*{box-sizing:border-box}
body{
 margin:0;background:var(--bg);color:var(--text);
 font:14px/1.55 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
 display:flex;min-height:100vh
}
a{color:var(--accent)}

aside{
 width:236px;flex:0 0 236px;background:var(--side);border-right:1px solid var(--line);
 padding:18px 12px;display:flex;flex-direction:column;gap:6px;
 position:sticky;top:0;height:100vh;overflow:auto
}
.brand{display:flex;align-items:center;gap:10px;padding:2px 8px 14px}
.logo{width:26px;height:26px;border-radius:8px;flex:0 0 26px;
 background:linear-gradient(140deg,var(--accent),#b98cff)}
.brand h1{margin:0;font-size:15px;letter-spacing:.02em}
.brand .sub{font-size:11px;color:var(--muted)}
.navgroup{margin-top:12px;padding:0 8px 4px;font-size:10px;letter-spacing:.12em;
 color:var(--muted);text-transform:uppercase}
.navitem{display:flex;align-items:center;gap:9px;padding:8px 10px;border-radius:9px;
 cursor:pointer;color:var(--muted);font-size:13px;border:1px solid transparent}
.navitem:hover{background:var(--card2);color:var(--text)}
.navitem.active{background:var(--card2);color:var(--text);border-color:var(--line)}
.navitem i{font-style:normal;width:16px;text-align:center;opacity:.8}
.sidefoot{margin-top:auto;padding-top:14px;border-top:1px solid var(--line);
 display:flex;flex-direction:column;gap:8px}
.themebtn{display:flex;align-items:center;justify-content:space-between;
 padding:8px 10px;border-radius:9px;cursor:pointer;background:var(--card2);
 border:1px solid var(--line);font-size:12px;color:var(--text)}

main{flex:1;min-width:0;padding:22px 26px 96px;max-width:1440px}
.pagehead{display:flex;align-items:baseline;justify-content:space-between;
 gap:14px;margin:0 0 16px}
.pagehead h2{margin:0;font-size:19px;letter-spacing:.01em}
.tag{font-size:11px;color:var(--muted);border:1px solid var(--line);
 border-radius:999px;padding:4px 11px;white-space:nowrap}
.page{display:none}
.page.active{display:block}

.card{
 background:var(--card);border:1px solid var(--line);border-radius:14px;
 padding:18px;margin-bottom:16px
}
.card h2{margin:0 0 14px;font-size:12px;letter-spacing:.11em;color:var(--muted)}
h3{color:var(--text)}
.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.row>*{min-width:0}
.actions{display:flex;flex-wrap:wrap;gap:9px;margin-top:14px}
button{
 background:var(--accent);color:#fff;border:0;border-radius:9px;
 padding:9px 15px;font-size:13px;cursor:pointer
}
button:hover{filter:brightness(1.08)}
button.secondary{background:var(--card2);color:var(--text);border:1px solid var(--line)}
button.danger{background:var(--red)}
label{display:block;margin:12px 0 5px;font-size:12px;color:var(--muted)}
input,select,textarea{
 width:100%;background:var(--card2);color:var(--text);
 border:1px solid var(--line);border-radius:9px;padding:9px 11px;font:inherit
}
input[type=range]{padding:0}
input[type=checkbox]{width:auto}
.toggle{display:flex;align-items:center;gap:9px;margin-top:12px}
.help{display:block;margin-top:8px;color:var(--muted);font-size:12px;line-height:1.5}
.stat{display:flex;justify-content:space-between;gap:10px;padding:7px 0;
 border-bottom:1px dashed var(--line);font-size:13px}
.stat:last-child{border-bottom:0}
.stat .value{color:var(--text);font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:7px}
.dot.ok{background:var(--green)}.dot.bad{background:var(--red)}
.dot.warn{background:var(--orange)}
.pinbox{
 background:var(--card2);border:1px solid var(--line);border-radius:10px;
 padding:11px 13px;margin-top:11px;font:12px/1.6 ui-monospace,Menlo,Consolas,monospace;
 color:var(--muted);white-space:pre-wrap;overflow:auto
}
.hw{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.hwitem{background:var(--card2);border:1px solid var(--line);border-radius:10px;padding:11px}
.hwtitle{font-size:12px}
.hwstate{font-size:12px;color:var(--muted);margin-top:4px}

.savebar{
 position:fixed;left:236px;right:0;bottom:0;z-index:40;
 padding:12px 26px;background:var(--card);border-top:1px solid var(--line);
 display:flex;gap:12px;align-items:center
}
.savebar .note{font-size:12px;color:var(--muted)}
@media(max-width:860px){.savebar{left:0;padding:10px 14px}}

.modal{position:fixed;inset:0;background:rgba(6,9,14,.75);display:flex;
 align-items:center;justify-content:center;padding:18px;z-index:50}
.modal[hidden]{display:none}
.modalCard{background:var(--card);border:1px solid var(--line);border-radius:14px;
 padding:20px;width:min(1180px,100%);max-height:88vh;overflow:auto}
.modalCard h3{margin:0 0 14px;font-size:15px;letter-spacing:.04em}
.chip{display:flex;align-items:center;gap:8px;background:var(--card2);
 border:1px solid var(--line);border-radius:9px;padding:7px 9px;margin:5px 0}
.chip span{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:12px}
.chip button{padding:2px 9px;margin:0;font-size:13px;line-height:1.2}
.hit{display:flex;align-items:center;gap:8px;padding:5px 2px;cursor:pointer}
.hit span{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:12px}

#toast{
 display:none;position:fixed;right:18px;bottom:18px;background:var(--card2);
 border:1px solid var(--line);border-radius:10px;padding:11px 15px;
 font-size:13px;z-index:60;max-width:420px
}
@media(max-width:860px){
 body{flex-direction:column}
 aside{width:auto;flex:none;height:auto;position:static;
  flex-direction:row;flex-wrap:wrap;border-right:0;border-bottom:1px solid var(--line)}
 .sidefoot{margin:0;border:0;padding:0}
 main{padding:16px 14px 80px}
 .row,.hw{grid-template-columns:1fr}
}
</style>
</head>
<body>
<aside>
 <div class="brand">
  <div class="logo"></div>
  <div>
   <h1 id="title">VoiceDot</h1>
   <div class="sub">Waveshare ESP32-S3</div>
  </div>
 </div>
<div class="navitem" data-page="uebersicht" onclick="showPage('uebersicht')"><i>◧</i>Übersicht</div>
<div class="navgroup">Sprachassistent</div>
<div class="navitem" data-page="voicedot" onclick="showPage('voicedot')"><i>◉</i>VoiceDot</div>
<div class="navitem" data-page="hass" onclick="showPage('hass')"><i>⌂</i>Home Assistant</div>
<div class="navitem" data-page="audio" onclick="showPage('audio')"><i>≋</i>Audio Pipeline</div>
<div class="navitem" data-page="ansage" onclick="showPage('ansage')"><i>☼</i>Ansage & Licht</div>
<div class="navgroup">Funktionen</div>
<div class="navitem" data-page="wecker" onclick="showPage('wecker')"><i>◷</i>Wecker & Timer</div>
<div class="navitem" data-page="gruppen" onclick="showPage('gruppen')"><i>⊞</i>Gruppen</div>
<div class="navitem" data-page="radio" onclick="showPage('radio')"><i>♪</i>Webradio</div>
<div class="navitem" data-page="klaenge" onclick="showPage('klaenge')"><i>♫</i>Klänge</div>
<div class="navgroup">System</div>
<div class="navitem" data-page="netzwerk" onclick="showPage('netzwerk')"><i>⇄</i>Netzwerk</div>
<div class="navitem" data-page="tagnacht" onclick="showPage('tagnacht')"><i>◐</i>Tag & Nacht</div>
<div class="navitem" data-page="multi" onclick="showPage('multi')"><i>⁙</i>Mehrere VoiceDots</div>
<div class="navitem" data-page="firmware" onclick="showPage('firmware')"><i>⤒</i>Firmware</div>
 <div class="sidefoot">
  <div class="themebtn" onclick="toggleTheme()">
   <span id="themeLabel">Dunkel</span><span id="themeIcon">◐</span>
  </div>
 </div>
</aside>

<main>
<div class="page" id="page-uebersicht">
<div class="pagehead"><h2>Übersicht</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
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
<section class="card full">
<h2>SERIAL LOG</h2>
<div class="pinbox" id="logBox" style="height:280px;white-space:pre-wrap">Warte auf Logzeilen ...</div>
<div class="actions">
 <button class="secondary" onclick="clearLog()">Ansicht leeren</button>
 <label class="toggle" style="margin:0"><input id="logFollow" type="checkbox" checked> automatisch scrollen</label>
</div>
<small class="help">Dieselben Zeilen, die über USB auf der seriellen Konsole erscheinen. Das Board hält die letzten 200.</small>
</section>
<section class="card full">
<h2>KONFIGURATION</h2>
<div class="actions">
 <button onclick="saveConfig()">Speichern & neu starten</button>
 <button class="secondary" onclick="reboot()">Neu starten</button>
</div>
</section>
</div>

<div class="page" id="page-voicedot">
<div class="pagehead"><h2>VoiceDot</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card">
<h2>VOICE DOT</h2>
<label>Gerätename</label>
<input id="device_name" placeholder="VoiceDot Wohnzimmer">

<label>Lautstärke</label>
<input id="volume" type="range" min="0" max="100" value="60"
 oninput="volLabel.textContent=this.value+' %'" onchange="applyRuntime()">
<small id="volLabel" class="help">60 %</small>
<small class="help">
100 % entspricht 80 % dessen, was der Codec könnte — darüber wird abgeriegelt,
auch für die Anhebung nach Umgebungslärm.
</small>

<label>Schrittweite für „leiser" und „lauter"</label>
<input id="volume_step" type="range" min="5" max="25" step="5" value="10"
 oninput="volStepLabel.textContent=this.value+' %'">
<small id="volStepLabel" class="help">10 %</small>

<label>Sprechtempo</label>
<input id="tts_speed" type="range" min="75" max="135" step="5" value="100"
 oninput="spdLabel.textContent=this.value+' %'" onchange="applyRuntime()">
<small id="spdLabel" class="help">100 %</small>
<small class="help">Über 100 % spricht der Assistent schneller, die Stimme wird dabei etwas höher.</small>

<label>LED-Helligkeit</label>
<input id="led_bri" type="range" min="0" max="100" value="30"
 oninput="briLabel.textContent=this.value" onchange="applyRuntime()">
<small id="briLabel" class="help">30</small>
</section>
</div>

<div class="page" id="page-hass">
<div class="pagehead"><h2>Home Assistant</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
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
<label>Assist Pipeline <span style="opacity:.7">(bestimmt Sprache, Stimme und LLM)</span></label>
<select id="ha_pipeline_sel" onchange="$('ha_pipeline').value=this.value"></select>
<input id="ha_pipeline" placeholder="oder ID direkt eintragen, leer = HA-Standard" style="margin-top:8px">
<div class="actions"><button class="secondary" onclick="loadPipelines()">Pipelines aus HA laden</button></div>
<small class="help" id="pipeHint">Noch nicht abgerufen.</small>

<div class="toggle">
 <input id="wake_word" type="checkbox" checked>
 <label for="wake_word" style="margin:0">Lokales Wake-Word (WakeNet, läuft auf dem Board)</label>
</div>
<label>Stichwort</label>
<select id="wake_model"></select>
<small class="help" id="wwHint">Die Liste kommt aus der Modell-Partition des Boards.</small>
<div class="toggle">
 <input id="vad" type="checkbox" checked>
 <label for="vad" style="margin:0">Aufnahme automatisch beenden, wenn ich aufhöre zu sprechen</label>
</div>

<div class="row">
 <div>
  <label>Abstand zum Störgeräusch</label>
  <input id="vad_release_db" type="range" min="3" max="18" step="1" value="8"
   oninput="relLabel.textContent=this.value+' dB'" onchange="applyVad()">
  <small id="relLabel" class="help">8 dB</small>
 </div>
 <div>
  <label>Stille bis Satzende</label>
  <input id="vad_silence_ms" type="range" min="600" max="3000" step="100" value="1400"
   oninput="silLabel.textContent=(this.value/1000).toFixed(1)+' s'" onchange="applyVad()">
  <small id="silLabel" class="help">1.4 s</small>
 </div>
</div>
<small class="help">
Läuft im Raum eine Maschine, muss der Abstand größer sein als deren Pegelschwankung —
sonst gilt jede Schwankung als Sprache und die Aufnahme endet nie. Wird dir dagegen
mitten im Satz abgeschnitten, den Abstand verkleinern oder die Stille verlängern.
Der aktuelle Rauschboden steht in der Assist-Diagnose.
</small>
<div class="toggle">
 <input id="follow_up" type="checkbox" checked>
 <label for="follow_up" style="margin:0">Rückfragen automatisch fortsetzen (continue_conversation)</label>
</div>
<div class="toggle">
 <input id="clean_markdown" type="checkbox" checked>
 <label for="clean_markdown" style="margin:0">Markdown aus Antworten entfernen (sonst werden Sternchen vorgelesen)</label>
</div>

<div class="actions"><button class="secondary" onclick="testHA()">Verbindung testen</button></div>
<small class="help" id="haResult">Noch nicht getestet.</small>
</section>
</div>

<div class="page" id="page-audio">
<div class="pagehead"><h2>Audio Pipeline</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
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

  <div style="height:14px;background:#202735;border:1px solid #334052;border-radius:8px;overflow:hidden;margin-top:10px;position:relative">
   <div id="noiseBar" style="height:100%;width:0%;background:#5a6b82"></div>
   <div id="threshMark" style="position:absolute;top:-2px;bottom:-2px;width:2px;background:#ffb454;left:0%"></div>
  </div>
  <small id="noiseText" class="help">Rauschboden ...</small>

   <small id="autoVolText" class="help">-</small>
  </div>
 </div>

<div class="row">
 <div>
  <h3 style="margin:4px 0 8px;font-size:12px;letter-spacing:.08em;opacity:.65">AM GERÄT</h3>

  <label>Mikrofonverstärkung</label>
  <input id="mic_gain_db" type="range" min="0" max="36" step="3" value="30"
   oninput="micGainLabel.textContent=this.value+' dB'">
  <small id="micGainLabel" class="help">30 dB</small>

  <div class="toggle" style="margin-top:10px">
   <input id="auto_volume" type="checkbox" checked>
   <label for="auto_volume" style="margin:0">Lautstärke an das Umfeld anpassen</label>
  </div>
  <label>Maximale Anhebung</label>
  <input id="auto_volume_max_db" type="range" min="0" max="18" step="1" value="10"
   oninput="autoVolLabel.textContent=this.value+' dB'">
  <small id="autoVolLabel" class="help">10 dB</small>

  <div class="toggle" style="margin-top:10px">
   <input id="stt_ha_vad" type="checkbox">
   <label for="stt_ha_vad" style="margin:0">HA darf das Satzende mitbestimmen</label>
  </div>
 </div>
 <div>
  <h3 style="margin:4px 0 8px;font-size:12px;letter-spacing:.08em;opacity:.65">IN HOME ASSISTANT</h3>

  <label>Pegel vor der Erkennung</label>
  <input id="stt_volume_percent" type="range" min="100" max="400" step="25" value="100"
   oninput="volMulLabel.textContent=(this.value/100).toFixed(2)+' ×'">
  <small id="volMulLabel" class="help">1.00 ×</small>

  <label>Automatische Verstärkung</label>
  <input id="stt_auto_gain_db" type="range" min="0" max="31" step="1" value="24"
   oninput="agcLabel.textContent=this.value==0?'aus':this.value+' dBFS'">
  <small id="agcLabel" class="help">24 dBFS</small>

  <label>Rauschunterdrückung</label>
  <input id="stt_noise_suppression" type="range" min="0" max="4" step="1" value="2"
   oninput="nsLabel.textContent=this.value==0?'aus':this.value">
  <small id="nsLabel" class="help">2</small>
 </div>
</div>

<small class="help">
<b>Am Gerät:</b> Die Mikrofonverstärkung hebt Stimme und Raumgeräusch gleichermaßen
an — gegen ein schlechtes Verhältnis der beiden hilft sie nicht, wohl aber gegen
einen insgesamt zu leisen Eingang. Die Anpassung ans Umfeld hebt umgekehrt nur die
<i>Ausgabe</i> an, wenn der Raum laut wird; gemessen wird dafür nur, solange der
eigene Lautsprecher still ist.<br>
<b>Das Satzende</b> erkennt dieses Gerät selbst, mit Pre-Roll-Puffer und einer
Schwelle relativ zum Pegel des Sprechers. Home Assistant zusätzlich mitreden zu
lassen ist die gröbere Variante — sie hat hier eine dreisekündige Frage nach
1,4 Sekunden abgeschnitten. Aus lassen.<br>
<b>In Home Assistant:</b> Diese drei gehen bei jeder Runde mit. Der Pegel
multipliziert die Abtastwerte direkt und wirkt am stärksten, die automatische
Verstärkung arbeitet danach. Rauschunterdrückung hilft bei konstanten
Störquellen, zu viel davon schadet der Erkennungsrate.
</small>

<div class="pinbox" id="pinbox">Lade Pinbelegung ...</div>
<div class="pinbox" id="assistBox">Assist bereit ...</div>
<div class="pinbox" id="wakeBox">Wake-Word ...</div>
<div class="actions">
 <button onclick="wakeTest()">Wake testen</button>
 <button class="secondary" onclick="speakerTest()">Lautsprecher testen</button>
</div>
<small class="help">
Sag das Wake-Word oder drücke K2 kurz, um eine Assist-Runde zu starten. K2 lang schaltet Mute.
Der Ring zeigt blau beim Zuhören, orange beim Denken und grün bei der Antwort.
</small>
</section>
</div>

<div class="page" id="page-ansage">
<div class="pagehead"><h2>Ansage & Licht</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>ANSAGE &amp; LICHT</h2>
<div class="toggle">
 <input id="ack_enabled" type="checkbox" checked>
 <label for="ack_enabled" style="margin:0">Nach dem Wake-Word kurz antworten</label>
</div>

<label>Phrasen <span style="opacity:.7">(mit | getrennt, eine wird zufällig gewählt)</span></label>
<input id="ack_phrases" placeholder="Ja bitte|Was gibt's?|Jawohl">

<div class="row">
 <div>
  <label>Farbe beim Zuhören (Pegelanzeige)</label>
  <input id="listen_color" type="color" value="#0096ff" onchange="applyRuntime()" style="height:46px;padding:4px">
 </div>
 <div>
  <label>Farbe beim Sprechen</label>
  <input id="speak_color" type="color" value="#ff7000" onchange="applyRuntime()" style="height:46px;padding:4px">
 </div>
</div>
<label>Status</label>
<div class="pinbox" id="ackBox" style="min-height:46px">...</div>

<div class="actions">
 <button class="secondary" onclick="ackBuild()">Ansagen erzeugen</button>
 <button class="secondary" onclick="ackTest()">Ansage anhören</button>
</div>
<small class="help">
Die Ansagen werden einmalig von deiner Home-Assistant-TTS erzeugt und auf dem Board gespeichert -
danach klingen sie wie der Assistent und brauchen kein Netz mehr. Ohne gespeicherte Ansage
spielt VoiceDot einen kurzen Zweiklang.
</small>
</section>
</div>

<div class="page" id="page-wecker">
<div class="pagehead"><h2>Wecker & Timer</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>WECKER &amp; TIMER</h2>
<div class="pinbox" id="alarmBox" style="min-height:40px">Lade ...</div>

<div class="row">
 <div>
  <h3 style="margin:4px 0 8px;font-size:12px;letter-spacing:.08em;opacity:.65">WECKER</h3>
  <label>Weckzeit</label>
  <div style="display:flex;gap:8px">
   <input id="alarmTime" type="time" style="flex:1">
   <button type="button" onclick="alarmSave()">Stellen</button>
   <button type="button" class="danger" onclick="alarmClear()">Löschen</button>
  </div>
  <div class="toggle" style="margin-top:10px">
   <input id="alarm_daily" type="checkbox">
   <label for="alarm_daily" style="margin:0">Jeden Tag wiederholen</label>
  </div>
  <label>Weckton</label>
  <select id="alarm_sound"><option value="">— keiner —</option></select>
 </div>
 <div>
  <h3 style="margin:4px 0 8px;font-size:12px;letter-spacing:.08em;opacity:.65">TIMER</h3>
  <label>Laufzeit in Minuten</label>
  <div style="display:flex;gap:8px">
   <input id="timerMinutes" type="number" min="1" max="720" value="10" style="flex:1">
   <button type="button" onclick="timerStart()">Starten</button>
   <button type="button" class="danger" onclick="timerStop()">Stoppen</button>
  </div>
  <label>Ton bei Ablauf</label>
  <select id="timer_sound"><option value="">— Ansage —</option></select>
 </div>
</div>

<label>Morgen-Briefing — Anweisung an den Assistenten</label>
<textarea id="alarm_briefing" rows="5"
 placeholder="Stelle den Wetterbericht von heute früh vor und was wir anziehen sollen."></textarea>
<div class="actions">
 <button class="secondary" onclick="alarmSaveOptions()">Briefing und Töne speichern</button>
 <button class="secondary" onclick="briefingTest()">Jetzt ausprobieren</button>
</div>

<small class="help">
Gesprochen: <b>„Stelle den Wecker auf 7 Uhr 30"</b>, <b>„Wecker löschen"</b>,
<b>„Wann klingelt der Wecker"</b> — und <b>„Stelle den Timer auf 10 Minuten"</b>,
<b>„Wie lange noch"</b>, <b>„Timer abbrechen"</b>. Alles wertet das Gerät selbst
aus, ohne Umweg über den Assistenten; der Wecker klingelt deshalb auch, wenn
Home Assistant gerade nicht erreichbar ist.<br>
Zuerst spielt der Weckton, danach das Briefing. Das Briefing ist <b>keine feste
Ansage, sondern eine Anweisung an deinen Assistenten</b> — sie geht durch dieselbe
Pipeline wie ein gesprochener Befehl, das Sprachmodell darf also Wetter, Kalender
und Gerätezustände heranziehen. Vorgelesen wird seine Antwort.<br>
Das kann bis zu einer Minute dauern; „Jetzt ausprobieren" testet es, ohne bis zum
nächsten Morgen zu warten. Der Timer läuft nur im Arbeitsspeicher und ist nach
einem Neustart weg; der Wecker bleibt.
</small>
</section>
</div>

<div class="page" id="page-gruppen">
<div class="pagehead"><h2>Gruppen</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>GRUPPEN</h2>
<div class="pinbox" id="groupBox" style="min-height:46px">Lade ...</div>

<label>Neue Gruppe anlegen</label>
<div style="display:flex;gap:8px">
 <input id="groupName" placeholder="Obergeschoss Licht" style="flex:1"
  onkeydown="if(event.key==='Enter'){event.preventDefault();groupCreate();}">
 <button type="button" onclick="groupCreate()">Anlegen</button>
</div>

<small class="help">
Gesprochen: <b>„Schalte im Obergeschoss das Licht aus"</b> oder
<b>„Erdgeschoss Rollo öffnen"</b>. Das Gerät sucht die Gruppe, deren Name im
Satz vorkommt, und schickt <i>einen</i> Service-Aufruf an Home Assistant — ohne
Umweg über das Sprachmodell, das dafür Sekunden bräuchte und jedes Mal eine
andere Auswahl treffen könnte.<br>
Erkannte Aktionen: <b>an/ein</b>, <b>aus</b>, <b>öffnen/auf/hoch</b>,
<b>schließen/zu/runter</b>. Licht und Steckdosen in einer Gruppe zu mischen ist
in Ordnung — geschaltet wird über <code>homeassistant.turn_on/off</code>, das
über alle Domänen hinweg funktioniert. Rollos brauchen eine eigene Gruppe, weil
sie eigene Dienste haben.<br>
Entitäten mit Komma trennen. Der Gruppenname muss mindestens zur Hälfte im Satz
vorkommen, damit gilt — sonst passiert nichts, statt das Falsche zu schalten.
</small>
</section>
</div>

<div class="page" id="page-radio">
<div class="pagehead"><h2>Webradio</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>WEBRADIO</h2>
<div class="pinbox" id="radioNow" style="min-height:26px">-</div>
<div class="pinbox" id="radioBox" style="min-height:46px">Lade ...</div>
<div class="actions"><button class="danger" onclick="radioStop()">Radio aus</button></div>

<label>Sender suchen (radio-browser.info)</label>
<div style="display:flex;gap:8px">
 <input id="radioQuery" placeholder="Energy Wien" style="flex:1"
  onkeydown="if(event.key==='Enter'){event.preventDefault();radioSearch();}">
 <button type="button" class="secondary" onclick="radioSearch()">Suchen</button>
</div>
<div class="pinbox" id="radioHits" style="min-height:26px">Noch nicht gesucht.</div>

<label>Sender von Hand eintragen</label>
<div style="display:flex;gap:8px">
 <input id="radioName" placeholder="Name" style="flex:1">
 <input id="radioUrl" placeholder="http://..." style="flex:2">
 <button type="button" onclick="radioAdd()">Merken</button>
</div>
<small class="help">
Gesprochen mit <b>„Spiele Energy Wien"</b>, beendet mit <b>„Stopp"</b> — beides
wertet das Gerät selbst aus, ohne Umweg über den Assistenten. Läuft Radio und
das Stichwort fällt, pausiert die Musik für die Frage und geht danach weiter.
Nur MP3-Streams, kein AAC. Die Ausgabe läuft mit 16 kHz, damit das Mikrofon
weiter auf 16 kHz bleibt und die Stichworterkennung mithören kann.
</small>
</section>
</div>

<div class="page" id="page-klaenge">
<div class="pagehead"><h2>Klänge</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>KLÄNGE</h2>
<form id="soundForm">
 <input id="soundFile" type="file" accept=".mp3,.wav,audio/mpeg,audio/wav">
 <div class="actions"><button type="submit">Hochladen</button></div>
</form>
<div class="pinbox" id="soundBox" style="min-height:46px">Lade ...</div>
<small class="help">
Kleine MP3- oder WAV-Dateien, maximal 512 kB und 20 Stück. In einer Ansage
vorangestellt abspielen mit <b>[dingdong.mp3] Es hat geläutet.</b> —
mehrere Klänge hintereinander sind erlaubt, der Text danach ist optional.
</small>
</section>
</div>

<div class="page" id="page-netzwerk">
<div class="pagehead"><h2>Netzwerk</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card">
<h2>NETWORK</h2>
<div class="stat"><span>Modus</span><span id="mode" class="value">...</span></div>
<div class="stat"><span>SSID</span><span id="ssidNow" class="value">...</span></div>
<div class="stat"><span>Signal</span><span id="rssi" class="value">...</span></div>
<div class="stat"><span>Hostname</span><span id="hostname" class="value">...</span></div>

<label>WLAN SSID</label>
<input id="wifi_ssid" placeholder="Mein WLAN">

<label>WLAN Passwort</label>
<input id="wifi_pass" type="password" placeholder="Passwort nur eingeben wenn geändert">

<div class="actions"><button class="secondary" onclick="scanWifi()">WLAN suchen</button></div>
<small class="help" id="scanResult"></small>
</section>
</div>

<div class="page" id="page-tagnacht">
<div class="pagehead"><h2>Tag & Nacht</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>TAG &amp; NACHT</h2>
<div class="toggle">
 <input id="schedule_enabled" type="checkbox">
 <label for="schedule_enabled" style="margin:0">Lautstärke und Helligkeit nach Uhrzeit umschalten</label>
</div>

<div class="row">
 <div>
  <label>Tag ab</label>
  <input id="day_start" type="time" value="06:00">
  <label>Lautstärke Tag (0–10)</label>
  <input id="day_volume" type="range" min="0" max="10" step="1" value="8"
   oninput="dayVolLabel.textContent=this.value">
  <small id="dayVolLabel" class="help">8</small>
  <label>LED-Helligkeit Tag</label>
  <input id="day_brightness" type="range" min="0" max="100" step="5" value="80"
   oninput="dayBriLabel.textContent=this.value+' %'">
  <small id="dayBriLabel" class="help">80 %</small>
 </div>
 <div>
  <label>Nacht ab</label>
  <input id="night_start" type="time" value="19:00">
  <label>Lautstärke Nacht (0–10)</label>
  <input id="night_volume" type="range" min="0" max="10" step="1" value="3"
   oninput="nightVolLabel.textContent=this.value">
  <small id="nightVolLabel" class="help">3</small>
  <label>LED-Helligkeit Nacht</label>
  <input id="night_brightness" type="range" min="0" max="100" step="5" value="30"
   oninput="nightBriLabel.textContent=this.value+' %'">
  <small id="nightBriLabel" class="help">30 %</small>
 </div>
</div>

<label>Zeitzone</label>
<input id="timezone" placeholder="CET-1CEST,M3.5.0,M10.5.0/3">
<small class="help" id="schedBox">Uhrzeit ...</small>

<div class="toggle">
 <input id="ha_publish" type="checkbox">
 <label for="ha_publish" style="margin:0">Zustand an Home Assistant melden (eigene Entität)</label>
</div>
<small class="help" id="haStateBox">-</small>
<small class="help">
Die Uhrzeit kommt per NTP, die Zeitzone im POSIX-Format bringt die
Sommerzeitumstellung gleich mit. Das Profil wird nur beim Wechsel gesetzt —
eine Änderung von Hand bleibt also bis zum nächsten Umschalten bestehen.
</small>
</section>
</div>

<div class="page" id="page-multi">
<div class="pagehead"><h2>Mehrere VoiceDots</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>MEHRERE VOICEDOTS</h2>
<div class="toggle">
 <input id="multi_enabled" type="checkbox" checked>
 <label for="multi_enabled" style="margin:0">Bei mehreren Geräten aushandeln, wer antwortet</label>
</div>
<label>Wartezeit für Gegenangebote</label>
<input id="multi_window_ms" type="range" min="80" max="600" step="20" value="220"
 oninput="multiLabel.textContent=this.value+' ms'">
<small id="multiLabel" class="help">220 ms</small>
<div class="pinbox" id="multiBox" style="min-height:46px">...</div>
<button class="green" onclick="yieldPreview()">Schlafenlegen zeigen</button>
<small class="help">
Hören mehrere VoiceDots dasselbe Stichwort, meldet jeder per UDP-Rundruf, wie laut
er es aufgenommen hat. Der lauteste führt das Gespräch, die anderen legen sich
wieder schlafen. Ist kein zweites Gerät im Netz, entfällt die Wartezeit ganz.
</small>
</section>
</div>

<div class="page" id="page-firmware">
<div class="pagehead"><h2>Firmware</h2><div class="tag">Firmware <span class="fwtag">...</span></div></div>
<section class="card full">
<h2>FIRMWARE</h2>
<div class="pinbox" id="updBox" style="min-height:46px">Lade ...</div>
<div class="actions">
 <button class="secondary" onclick="updCheck()">Nach Updates suchen</button>
</div>
<div class="pinbox" id="updList" style="min-height:26px">-</div>
<div class="toggle">
 <input id="update_check" type="checkbox" checked>
 <label for="update_check" style="margin:0">Selbst bei GitHub nachsehen (alle 12 Stunden)</label>
</div>
<small class="help">
Die Firmware kommt aus den <b>Releases</b> des Projekts auf GitHub. Ausgewählt
wird hier oder in Home Assistant, geladen und geschrieben wird direkt vom
Gerät — kein USB-Kabel nötig. Geschrieben wird immer in die gerade nicht
laufende Partition; geht dabei etwas schief, bleibt die alte Version bestehen.
Einstellungen, Klänge und Sender überleben das Update, die Wake-Word-Modelle
liegen in einer eigenen Partition und werden nicht mit angefasst.
<br><br>
Waehrend der Suche und des Ladens hoert das Geraet <b>kurz nicht zu</b>: die
Verschluesselung zu GitHub braucht denselben Arbeitsspeicher wie die
Wake-Word-Erkennung, die deshalb so lange pausiert und danach von selbst
zurueckkommt.
</small>
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
</main>

<div id="groupModal" class="modal" hidden>
 <div class="modalCard">
  <h3 id="groupModalTitle">Gruppe bearbeiten</h3>
  <div class="row">
   <div>
    <label>Entität in Home Assistant suchen</label>
    <div style="display:flex;gap:8px">
     <input id="entQuery" placeholder="licht og" style="flex:1"
      onkeydown="if(event.key==='Enter'){event.preventDefault();entSearch();}">
     <button type="button" class="secondary" onclick="entSearch()">Suchen</button>
    </div>
    <div class="pinbox" id="entHits" style="min-height:180px">Suchbegriff eingeben.</div>
   </div>
   <div>
    <label>In dieser Gruppe</label>
    <div class="pinbox" id="entChosen" style="min-height:180px">Noch nichts ausgewählt.</div>
   </div>
  </div>
  <div class="actions">
   <button type="button" onclick="groupModalSave()">Speichern</button>
   <button type="button" class="secondary" onclick="groupModalClose()">Abbrechen</button>
  </div>
 </div>
</div>

<div class="savebar">
 <button onclick="saveAll()">Speichern</button>
 <span class="note">Sichert alle Einstellungen des Geräts, nicht nur diese Seite.</span>
</div>

<div id="toast"></div>

<script>
)HTML"
"\n"
"const $=id=>document.getElementById(id);\n"
"\n"
"// The chosen page and theme are remembered per browser, so a reload does\n"
"// not drop you back on the overview in the middle of setting something up.\n"
"function showPage(key){\n"
" document.querySelectorAll('.page').forEach(p=>\n"
"  p.classList.toggle('active',p.id==='page-'+key));\n"
" document.querySelectorAll('.navitem').forEach(n=>\n"
"  n.classList.toggle('active',n.dataset.page===key));\n"
" try{localStorage.setItem('vd_page',key);}catch(e){}\n"
" window.scrollTo(0,0);\n"
"}\n"
"\n"
"function applyTheme(t){\n"
" document.documentElement.setAttribute('data-theme',t);\n"
" const dark=t!=='light';\n"
" $('themeLabel').textContent=dark?'Dunkel':'Hell';\n"
" $('themeIcon').textContent=dark?'◐':'☀';\n"
" try{localStorage.setItem('vd_theme',t);}catch(e){}\n"
"}\n"
"\n"
"function toggleTheme(){\n"
" const now=document.documentElement.getAttribute('data-theme');\n"
" applyTheme(now==='light'?'dark':'light');\n"
"}\n"
"\n"
"// One button on every page saves the whole configuration: the settings are\n"
"// one form spread over several pages, and saving only what is visible would\n"
"// quietly drop the rest.\n"
"async function saveAll(){\n"
" await applyRuntime();\n"
" await saveConfig();\n"
"}\n"
"\n"
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
"  document.querySelectorAll('.fwtag').forEach(e=>e.textContent=j.firmware);\n"
"  $('uptime').textContent=uptime(j.uptime);\n"
"  $('ip').textContent=j.ip;\n"
"  $('heap').textContent=bytes(j.free_heap)+' (min '+bytes(j.min_free_heap)+')';\n"
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
"  $('hostname').textContent=(j.hostname||'voicedot')+'.local';\n"
"  if(j.device_name){$('title').textContent=j.device_name;document.title=j.device_name;}\n"
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
"  const dbPct=v=>Math.max(0,Math.min(100,(v+80)/80*100));\n"
"  const nf=j.audio.room_noise_db??-96;\n"
"  const sp=j.audio.speech_threshold_db??-96;\n"
"  const rl=j.audio.release_threshold_db??-96;\n"
"  $('noiseBar').style.width=dbPct(nf)+'%';\n"
"  $('threshMark').style.left=dbPct(sp)+'%';\n"
"  $('noiseText').textContent=\n"
"   'Rauschboden '+nf.toFixed(1)+' dBFS · Sprache ab '+sp.toFixed(1)+\n"
"   ' · Satzende unter '+rl.toFixed(1)+' dBFS';\n"
"\n"
"  const up=j.update||{};\n"
"  const rel=up.releases||[];\n"
"  $('updBox').textContent=\n"
"   'INSTALLIERT '+(up.installed||'-')+'\\n'+\n"
"   'VERFUEGBAR  '+(up.latest||'-')+(up.available?'   (neuer)':'')+'\\n'+\n"
"   'STATUS      '+(up.status||'-')+\n"
"   (up.checked?'   (vor '+up.age_s+' s geprueft)':'')+\n"
"   (up.in_progress?'\\nFORTSCHRITT '+up.progress+' %':'');\n"
"\n"
"  const ul=$('updList');\n"
"  ul.innerHTML='';\n"
"  if(!rel.length){ul.textContent='Noch keine Releases abgerufen.';}\n"
"  else rel.forEach(x=>{\n"
"   const row=document.createElement('div');\n"
"   row.style.cssText='display:flex;align-items:center;gap:8px;margin:3px 0';\n"
"   const label=document.createElement('span');\n"
"   label.style.cssText='flex:1;overflow:hidden;text-overflow:ellipsis';\n"
"   label.textContent=x.tag+(x.firmware?'':'   (ohne Firmware-Datei)')+\n"
"    (x.newer?'   neuer':'');\n"
"   row.appendChild(label);\n"
"   if(x.firmware){\n"
"    const b=document.createElement('button');\n"
"    b.type='button';b.className=x.newer?'':'secondary';\n"
"    b.textContent='Installieren';\n"
"    b.onclick=()=>updInstall(x.tag);\n"
"    row.appendChild(b);\n"
"   }\n"
"   ul.appendChild(row);\n"
"  });\n"
"\n"
"  const av=j.auto_volume||{};\n"
"  $('autoVolText').textContent=av.enabled\n"
"   ?('Umfeld '+(av.noise_db??0).toFixed(1)+' dBFS · ruhig bis '+\n"
"     (av.quiet_db??0).toFixed(1)+' dBFS · Anhebung jetzt +'+\n"
"     (av.boost_db??0).toFixed(1)+' dB · zuletzt gesprochen +'+\n"
"     (av.last_boost_db??0).toFixed(1)+' dB')\n"
"   :'Anpassung aus.';\n"
"\n"
"  $('assistBox').textContent=\n"
"   'STATE       '+(j.assist.state||'-')+'\\n'+\n"
"   'WS STAGE    '+(j.assist.ws_stage||'-')+'\\n'+\n"
"   'MESSAGE     '+(j.assist.message||'-')+'\\n'+\n"
"   'DETAIL      '+(j.assist.ws_detail||'-')+'\\n'+\n"
"   'REC BYTES   '+(j.assist.bytes||0)+'\\n'+\n"
"   'REC TIME    '+(j.assist.duration_ms||0)+' ms\\n'+\n"
"   'WS CODE     '+(j.assist.ws_code||0)+'\\n'+\n"
"   'TRANSCRIPT  '+(j.assist.transcript||'-')+'\\n'+\n"
"   'ASSISTANT   '+(j.assist.assistant_text||'-')+'\\n'+\n"
"   'TTS URL     '+(j.assist.tts_url||'-')+'\\n'+\n"
"   'TTS         '+(j.assist.tts_status||'-')+'\\n'+\n"
"   'CONV ID     '+(j.assist.conversation_id||'-')+'\\n'+\n"
"   'PIPELINE    '+(j.assist.pipeline||'HA Standard')+'\\n'+\n"
"   'VAD         '+(j.assist.vad?'an':'aus')+' · Rauschboden '+(j.assist.noise_floor_db??0).toFixed(1)+' dBFS'+'\\n'+\n"
"   'FOLLOW-UP   '+(j.assist.follow_up?'an':'aus')+(j.assist.queued?' · nächste Runde in der Warteschlange':'');\n"
"\n"
"  const w=j.wake_word||{};\n"
"  let wstate='aus';\n"
"  if(w.running)wstate=w.paused?'pausiert (Aufnahme oder Wiedergabe läuft)':'hört auf das Wake-Word';\n"
"  else if(w.enabled)wstate='nicht gestartet';\n"
"  $('wakeBox').textContent=\n"
"   'WAKE-WORD   '+(w.word||'-')+'\\n'+\n"
"   'STATUS      '+wstate+'\\n'+\n"
"   'DETAIL      '+(w.status||'-')+'\\n'+\n"
"   'MODELL      '+(w.available?'Partition gefunden':'keine Modell-Partition')+'\\n'+\n"
"   'ERKANNT     '+(w.detections||0)+'x'+((w.detections&&w.last_ms_ago!=null)?' · zuletzt vor '+Math.round(w.last_ms_ago/1000)+' s':'');\n"
"  const pl=j.pipelines||{};\n"
"  const psel=$('ha_pipeline_sel');\n"
"  const plist=pl.list||[];\n"
"  const psig=plist.map(x=>x.id).join('|');\n"
"  if(psel.dataset.sig!==psig){\n"
"   psel.dataset.sig=psig;\n"
"   psel.innerHTML='';\n"
"   const o0=document.createElement('option');\n"
"   o0.value='';o0.textContent='HA-Standard';\n"
"   psel.appendChild(o0);\n"
"   plist.forEach(x=>{\n"
"    const o=document.createElement('option');\n"
"    o.value=x.id;\n"
"    o.textContent=x.name+(x.voice?'  ('+x.voice+')':'')+(x.id===pl.preferred?'  ★':'');\n"
"    psel.appendChild(o);\n"
"   });\n"
"   psel.value=$('ha_pipeline').value||'';\n"
"  }\n"
"  $('pipeHint').textContent=pl.status||'-';\n"
"\n"
"  const sc=j.schedule||{};\n"
"  $('schedBox').textContent='Uhrzeit '+(sc.now||'--:--')+' '+(sc.tzname||'')+\n"
"   (sc.time_valid?'':' (noch keine NTP-Zeit)')+(sc.rtc?' · RTC vorhanden':'')+\n"
"   ' · aktuelles Profil: '+(sc.period||'-');\n"
"\n"
"  const mu=j.multi||{};\n"
"  const peers=mu.peers||[];\n"
"  $('multiBox').textContent=\n"
"   'EIGENE ID    '+(mu.id||'-')+'\\n'+\n"
"   'STATUS       '+(mu.enabled?(mu.ready?'aktiv':'UDP nicht bereit'):'aus')+'\\n'+\n"
"   'MEIN SCORE   '+(mu.score??0).toFixed(1)+' dB  (Spitze '+(mu.peak_db??0).toFixed(1)+' dBFS)\\n'+\n"
"   'LETZTE RUNDE '+(mu.decision||'-')+'\\n'+\n"
"   'NACHBARN     '+(peers.length?peers.map(x=>(x.name||x.id)+' ('+x.ip+', '+x.age_s+' s)').join(', '):'keine');\n"
"\n"
"  const hs=j.ha_state||{};\n"
"  $('haStateBox').textContent=hs.enabled\n"
"   ?('Entität: '+(hs.entity||'-')+' · '+(hs.status||'-'))\n"
"   :'aus - VoiceDot erscheint nicht in Home Assistant';\n"
"\n"
"  const a=j.ack||{};\n"
"  $('ackBox').textContent=\n"
"   (a.enabled?'aktiv':'aus')+' · '+(a.clips||0)+' Clip'+((a.clips||0)==1?'':'s')+'\\n'+\n"
"   (a.status||'-')+'\\n'+\n"
"   'TTS: '+(a.engine||'noch unbekannt')+(a.language?' · '+a.language:'')+(a.voice?' · '+a.voice:'');\n"
"\n"
"  const sel=$('wake_model');\n"
"  const list=w.models||[];\n"
"  const sig=list.map(m=>m.id).join('|');\n"
"  if(sel.dataset.sig!==sig){\n"
"   sel.dataset.sig=sig;\n"
"   const keep=sel.value||w.model||'';\n"
"   sel.innerHTML='';\n"
"   if(!list.length){\n"
"    const o=document.createElement('option');\n"
"    o.value='';o.textContent='kein Modell in der Partition';\n"
"    sel.appendChild(o);\n"
"   }\n"
"   list.forEach(m=>{\n"
"    const o=document.createElement('option');\n"
"    o.value=m.id;o.textContent=m.words+'  ('+m.id+')';\n"
"    sel.appendChild(o);\n"
"   });\n"
"   if(keep)sel.value=keep;\n"
"   if(!sel.value&&w.model)sel.value=w.model;\n"
"  }\n"
"  $('wwHint').textContent=list.length\n"
"   ?(list.length+' Stichwort'+(list.length>1?'e':'')+' in der Partition · aktiv: '+(w.word||'-'))\n"
"   :'Keine Modell-Partition gefunden - partitions.csv verwenden und srmodels.bin flashen.';\n"
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
" $('ha_pipeline').value=j.ha_pipeline||'';\n"
" $('vad').checked=j.vad!==false;\n"
" $('vad_release_db').value=j.vad_release_db??8;\n"
" $('relLabel').textContent=$('vad_release_db').value+' dB';\n"
" $('vad_silence_ms').value=j.vad_silence_ms??1400;\n"
" $('silLabel').textContent=($('vad_silence_ms').value/1000).toFixed(1)+' s';\n"
" $('follow_up').checked=j.follow_up!==false;\n"
" $('clean_markdown').checked=j.clean_markdown!==false;\n"
" $('mic_gain_db').value=j.mic_gain_db??30;\n"
" $('micGainLabel').textContent=$('mic_gain_db').value+' dB';\n"
" $('stt_ha_vad').checked=j.stt_ha_vad===true;\n"
" $('stt_noise_suppression').value=j.stt_noise_suppression??2;\n"
" $('nsLabel').textContent=$('stt_noise_suppression').value==0?'aus':$('stt_noise_suppression').value;\n"
" $('stt_auto_gain_db').value=j.stt_auto_gain_db??24;\n"
" $('agcLabel').textContent=$('stt_auto_gain_db').value==0?'aus':$('stt_auto_gain_db').value+' dBFS';\n"
" $('stt_volume_percent').value=j.stt_volume_percent??100;\n"
" $('volMulLabel').textContent=($('stt_volume_percent').value/100).toFixed(2)+' ×';\n"
" $('volume_step').value=j.volume_step??10;\n"
" $('volStepLabel').textContent=$('volume_step').value+' %';\n"
" $('update_check').checked=j.update_check!==false;\n"
" $('auto_volume').checked=j.auto_volume!==false;\n"
" $('auto_volume_max_db').value=j.auto_volume_max_db??10;\n"
" $('autoVolLabel').textContent=$('auto_volume_max_db').value+' dB';\n"
" $('multi_enabled').checked=j.multi_enabled!==false;\n"
" $('multi_window_ms').value=j.multi_window_ms??220;\n"
" $('multiLabel').textContent=$('multi_window_ms').value+' ms';\n"
" $('wake_word').checked=j.wake_word!==false;\n"
" $('ack_enabled').checked=j.ack_enabled!==false;\n"
" $('schedule_enabled').checked=!!j.schedule_enabled;\n"
" $('day_start').value=j.day_start||'06:00';\n"
" $('night_start').value=j.night_start||'19:00';\n"
" $('day_volume').value=j.day_volume??8;\n"
" $('dayVolLabel').textContent=$('day_volume').value;\n"
" $('day_brightness').value=j.day_brightness??80;\n"
" $('dayBriLabel').textContent=$('day_brightness').value+' %';\n"
" $('night_volume').value=j.night_volume??3;\n"
" $('nightVolLabel').textContent=$('night_volume').value;\n"
" $('night_brightness').value=j.night_brightness??30;\n"
" $('nightBriLabel').textContent=$('night_brightness').value+' %';\n"
" $('timezone').value=j.timezone||'';\n"
" $('ha_publish').checked=!!j.ha_publish;\n"
" $('ack_phrases').value=j.ack_phrases||'';\n"
" if(j.speak_color)$('speak_color').value=j.speak_color;\n"
" if(j.listen_color)$('listen_color').value=j.listen_color;\n"
" $('tts_speed').value=j.tts_speed??100;\n"
" $('spdLabel').textContent=$('tts_speed').value+' %';\n"
" if(j.wake_model)window.__wantModel=j.wake_model;\n"
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
" p.set('ha_pipeline',$('ha_pipeline').value);\n"
" p.set('vad',$('vad').checked?'1':'0');\n"
" p.set('vad_release_db',$('vad_release_db').value);\n"
" p.set('vad_silence_ms',$('vad_silence_ms').value);\n"
" p.set('follow_up',$('follow_up').checked?'1':'0');\n"
" p.set('clean_markdown',$('clean_markdown').checked?'1':'0');\n"
" p.set('mic_gain_db',$('mic_gain_db').value);\n"
" p.set('stt_ha_vad',$('stt_ha_vad').checked?'1':'0');\n"
" p.set('stt_noise_suppression',$('stt_noise_suppression').value);\n"
" p.set('stt_auto_gain_db',$('stt_auto_gain_db').value);\n"
" p.set('stt_volume_percent',$('stt_volume_percent').value);\n"
" p.set('volume_step',$('volume_step').value);\n"
" p.set('update_check',$('update_check').checked?'1':'0');\n"
" p.set('auto_volume',$('auto_volume').checked?'1':'0');\n"
" p.set('auto_volume_max_db',$('auto_volume_max_db').value);\n"
" p.set('multi_enabled',$('multi_enabled').checked?'1':'0');\n"
" p.set('multi_window_ms',$('multi_window_ms').value);\n"
" p.set('wake_word',$('wake_word').checked?'1':'0');\n"
" p.set('wake_model',$('wake_model').value);\n"
" p.set('ack_enabled',$('ack_enabled').checked?'1':'0');\n"
" p.set('schedule_enabled',$('schedule_enabled').checked?'1':'0');\n"
" p.set('day_start',$('day_start').value);\n"
" p.set('night_start',$('night_start').value);\n"
" p.set('day_volume',$('day_volume').value);\n"
" p.set('day_brightness',$('day_brightness').value);\n"
" p.set('night_volume',$('night_volume').value);\n"
" p.set('night_brightness',$('night_brightness').value);\n"
" p.set('timezone',$('timezone').value);\n"
" p.set('ha_publish',$('ha_publish').checked?'1':'0');\n"
" p.set('ack_phrases',$('ack_phrases').value);\n"
" p.set('speak_color',$('speak_color').value);\n"
" p.set('listen_color',$('listen_color').value);\n"
" p.set('tts_speed',$('tts_speed').value);\n"
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
"async function applyRuntime(){\n"
" const p=new URLSearchParams();\n"
" p.set('volume',$('volume').value);\n"
" p.set('led_brightness',$('led_bri').value);\n"
" p.set('speak_color',$('speak_color').value);\n"
" p.set('listen_color',$('listen_color').value);\n"
" p.set('tts_speed',$('tts_speed').value);\n"
" try{\n"
"  await fetch('/api/runtime',{\n"
"   method:'POST',\n"
"   headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},\n"
"   body:p.toString()\n"
"  });\n"
" }catch(e){}\n"
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
"async function yieldPreview(){\n"
" const r=await fetch('/api/hardware/led-test?phase=yield',{method:'POST'});\n"
" toast(await r.text());\n"
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
"let logSince=0;\n"
"let logBuf=[];\n"
"\n"
"async function refreshLog(){\n"
" try{\n"
"  const r=await fetch('/api/log?since='+logSince,{cache:'no-store'});\n"
"  const j=await r.json();\n"
"  if(j.total<logSince){logBuf=[];logSince=0;}\n"
"  logSince=j.total;\n"
"  if(j.lines&&j.lines.length){\n"
"   logBuf=logBuf.concat(j.lines);\n"
"   if(logBuf.length>500)logBuf=logBuf.slice(-500);\n"
"   const b=$('logBox');\n"
"   b.textContent=logBuf.join('\\n');\n"
"   if($('logFollow').checked)b.scrollTop=b.scrollHeight;\n"
"  }\n"
" }catch(e){}\n"
"}\n"
"\n"
"function clearLog(){logBuf=[];$('logBox').textContent='';}\n"
"\n"
"async function refreshGroups(){\n"
" try{\n"
"  const j=await (await fetch('/api/groups',{cache:'no-store'})).json();\n"
"  const box=$('groupBox');\n"
"  box.innerHTML='';\n"
"  const gs=j.groups||[];\n"
"  if(!gs.length){box.textContent='Noch keine Gruppen angelegt.';return;}\n"
"  gs.forEach(g=>{\n"
"   const row=document.createElement('div');\n"
"   row.style.cssText='display:flex;align-items:center;gap:8px;margin:3px 0';\n"
"   const label=document.createElement('span');\n"
"   label.style.cssText='flex:1;overflow:hidden;text-overflow:ellipsis';\n"
"   const n=g.entities.split(',').filter(x=>x.trim()).length;\n"
"   label.textContent=g.name+'   ('+(n?n+' Entitäten':'noch leer')+')';\n"
"   label.title=g.entities;\n"
"   const edit=document.createElement('button');\n"
"   edit.type='button';edit.className='secondary';edit.textContent='Bearbeiten';\n"
"   edit.onclick=()=>groupModalOpen(g.name,g.entities);\n"
"   const del=document.createElement('button');\n"
"   del.type='button';del.className='danger';del.textContent='Löschen';\n"
"   del.onclick=()=>groupDelete(g.name);\n"
"   row.appendChild(label);row.appendChild(edit);row.appendChild(del);\n"
"   box.appendChild(row);\n"
"  });\n"
" }catch(e){}\n"
"}\n"
"\n"
"let groupEditing='';\n"
"let groupSel=[];\n"
"const entNames={};\n"
"\n"
"function groupModalOpen(name,entities){\n"
" groupEditing=name;\n"
" groupSel=(entities||'').split(',').map(x=>x.trim()).filter(Boolean);\n"
" $('groupModalTitle').textContent='Gruppe: '+name;\n"
" $('entQuery').value='';\n"
" $('entHits').textContent='Suchbegriff eingeben.';\n"
" groupChips();\n"
" $('groupModal').hidden=false;\n"
"}\n"
"\n"
"function groupModalClose(){$('groupModal').hidden=true;groupEditing='';}\n"
"\n"
"function groupChips(){\n"
" const box=$('entChosen');\n"
" box.innerHTML='';\n"
" if(!groupSel.length){box.textContent='Noch nichts ausgewählt.';return;}\n"
" groupSel.forEach(id=>{\n"
"  const chip=document.createElement('div');\n"
"  chip.className='chip';\n"
"  const label=document.createElement('span');\n"
"  label.textContent=entNames[id]?entNames[id]+'  ·  '+id:id;\n"
"  label.title=id;\n"
"  const x=document.createElement('button');\n"
"  x.type='button';x.className='danger';x.textContent='×';\n"
"  x.title='Aus der Gruppe entfernen';\n"
"  x.onclick=()=>{groupSel=groupSel.filter(e=>e!==id);groupChips();};\n"
"  chip.appendChild(label);chip.appendChild(x);\n"
"  box.appendChild(chip);\n"
" });\n"
"}\n"
"\n"
"async function groupModalSave(){\n"
" const p=new URLSearchParams();\n"
" p.set('name',groupEditing);p.set('entities',groupSel.join(', '));\n"
" const r=await fetch('/api/groups',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});\n"
" toast(await r.text());groupModalClose();refreshGroups();\n"
"}\n"
"\n"
"async function entSearch(){\n"
" const q=$('entQuery').value.trim();\n"
" const box=$('entHits');\n"
" box.textContent='Frage Home Assistant ...';\n"
" try{\n"
"  const r=await fetch('/api/ha/entities?q='+encodeURIComponent(q),{cache:'no-store'});\n"
"  if(!r.ok){box.textContent=await r.text();return;}\n"
"  const j=await r.json();\n"
"  box.innerHTML='';\n"
"  if(!j.count){box.textContent='Nichts gefunden.';return;}\n"
"  j.entities.forEach(e=>{\n"
"   entNames[e.id]=e.name||'';\n"
"   const row=document.createElement('div');\n"
"   row.className='hit';\n"
"   const label=document.createElement('span');\n"
"   label.textContent=(e.name||e.id)+'  ·  '+e.id;\n"
"   label.title=e.id;\n"
"   const add=document.createElement('button');\n"
"   add.type='button';add.className='secondary';add.textContent='+';\n"
"   const put=()=>{\n"
"    if(groupSel.includes(e.id)){toast('Ist schon in der Gruppe.');return;}\n"
"    groupSel.push(e.id);groupChips();\n"
"   };\n"
"   row.onclick=put;add.onclick=ev=>{ev.stopPropagation();put();};\n"
"   row.appendChild(label);row.appendChild(add);\n"
"   box.appendChild(row);\n"
"  });\n"
"  if(j.truncated){\n"
"   const note=document.createElement('small');\n"
"   note.className='help';\n"
"   note.textContent='Nur die ersten '+j.count+' Treffer — Suche eingrenzen.';\n"
"   box.appendChild(note);\n"
"  }\n"
" }catch(e){box.textContent='Suche fehlgeschlagen: '+e;}\n"
"}\n"
"\n"
"async function groupCreate(){\n"
" const n=$('groupName').value.trim();\n"
" if(!n){toast('Bitte einen Namen eingeben.');return;}\n"
" const p=new URLSearchParams();p.set('name',n);p.set('entities','');\n"
" const r=await fetch('/api/groups',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});\n"
" toast(await r.text());\n"
" $('groupName').value='';\n"
" await refreshGroups();\n"
" groupModalOpen(n,'');\n"
"}\n"
"\n"
"async function groupDelete(name){\n"
" if(!confirm(name+' löschen?'))return;\n"
" const p=new URLSearchParams();p.set('name',name);p.set('delete','1');\n"
" const r=await fetch('/api/groups',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});\n"
" toast(await r.text());refreshGroups();\n"
"}\n"
"\n"
"let alarmLoaded=false;\n"
"let soundsLoaded=false;\n"
"\n"
"async function refreshAlarm(){\n"
" try{\n"
"  const a=await (await fetch('/api/alarm',{cache:'no-store'})).json();\n"
"  const t=await (await fetch('/api/timer',{cache:'no-store'})).json();\n"
"  const hhmm=x=>{const h=Math.floor(x/3600),m=Math.floor(x%3600/60),s=x%60;\n"
"   return (h?h+' h ':'')+(h||m?m+' min ':'')+s+' s';};\n"
"  $('alarmBox').textContent=\n"
"   'WECKER  '+(a.set?(a.time+' Uhr'+(a.daily?' , taeglich':'')+\n"
"     (a.seconds_until>0?'   in '+hhmm(a.seconds_until):'')):'nicht gestellt')+'\\n'+\n"
"   'TIMER   '+(t.active?('noch '+hhmm(t.remaining_s)+'   von '+hhmm(t.total_s)):'laeuft nicht');\n"
"  // Fields are filled once and after a save, never on the poll: it ran every\n"
"  // two seconds and wiped whatever was being typed the moment focus left.\n"
"  $('alarm_daily').checked=a.daily===true;\n"
"  if(!alarmLoaded){\n"
"   $('alarm_briefing').value=a.briefing||'';\n"
"   if(a.set)$('alarmTime').value=a.time;\n"
"   alarmLoaded=true;\n"
"  }\n"
"  await fillSoundSelects(a.sound||'',t.sound||'');\n"
" }catch(e){}\n"
"}\n"
"\n"
"async function fillSoundSelects(alarmSel,timerSel){\n"
" try{\n"
"  const j=await (await fetch('/api/sound/list',{cache:'no-store'})).json();\n"
"  const files=j.files||[];\n"
"  [['alarm_sound',alarmSel,'— keiner —'],['timer_sound',timerSel,'— Ansage —']]\n"
"   .forEach(([id,sel,none])=>{\n"
"   const el=$(id);\n"
"   // Rebuild the options only when the library itself changed.\n"
"   if(el.dataset.n!==String(files.length)){\n"
"    const keep=el.value;\n"
"    el.innerHTML='';\n"
"    const o0=document.createElement('option');o0.value='';o0.textContent=none;\n"
"    el.appendChild(o0);\n"
"    files.forEach(f=>{\n"
"     const o=document.createElement('option');o.value=f.name;o.textContent=f.name;\n"
"     el.appendChild(o);});\n"
"    el.dataset.n=String(files.length);\n"
"    el.value=keep;\n"
"   }\n"
"   // Applied once. After that the poll must not touch it, or a choice\n"
"   // would be undone two seconds after it was made.\n"
"   if(!soundsLoaded)el.value=sel;\n"
"  });\n"
"  if(files.length||alarmLoaded)soundsLoaded=true;\n"
" }catch(e){}\n"
"}\n"
"\n"
"async function alarmSave(){\n"
" const v=$('alarmTime').value;\n"
" if(!v){toast('Bitte eine Uhrzeit waehlen.');return;}\n"
" const p=new URLSearchParams();p.set('time',v);\n"
" p.set('daily',$('alarm_daily').checked?'1':'0');\n"
" const r=await fetch('/api/alarm',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});\n"
" toast(await r.text());refreshAlarm();\n"
"}\n"
"\n"
"async function alarmClear(){\n"
" const r=await fetch('/api/alarm?clear=1',{method:'POST'});\n"
" toast(await r.text());refreshAlarm();\n"
"}\n"
"\n"
"async function alarmSaveOptions(){\n"
" const p=new URLSearchParams();\n"
" p.set('sound',$('alarm_sound').value);\n"
" p.set('briefing',$('alarm_briefing').value);\n"
" p.set('daily',$('alarm_daily').checked?'1':'0');\n"
" const r=await fetch('/api/alarm',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});\n"
" toast(await r.text());\n"
" alarmLoaded=false;soundsLoaded=false;\n"
" const q=new URLSearchParams();q.set('sound',$('timer_sound').value);\n"
" await fetch('/api/timer',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q.toString()});\n"
" refreshAlarm();\n"
"}\n"
"\n"
"async function timerStart(){\n"
" const m=parseInt($('timerMinutes').value||'0',10);\n"
" if(!m){toast('Bitte Minuten angeben.');return;}\n"
" const r=await fetch('/api/timer?minutes='+m,{method:'POST'});\n"
" toast(await r.text());refreshAlarm();\n"
"}\n"
"\n"
"async function timerStop(){\n"
" const r=await fetch('/api/timer?clear=1',{method:'POST'});\n"
" toast(await r.text());refreshAlarm();\n"
"}\n"
"\n"
"async function refreshRadio(){\n"
" try{\n"
"  const r=await fetch('/api/radio/list',{cache:'no-store'});\n"
"  const j=await r.json();\n"
"  $('radioNow').textContent=j.active?('LAEUFT   '+j.station+'   ('+j.status+')'):'AUS';\n"
"  const box=$('radioBox');\n"
"  box.innerHTML='';\n"
"  const st=j.stations||[];\n"
"  if(!st.length){box.textContent='Noch keine Sender gespeichert.';return;}\n"
"  st.forEach(x=>{\n"
"   const row=document.createElement('div');\n"
"   row.style.cssText='display:flex;align-items:center;gap:8px;margin:3px 0';\n"
"   const label=document.createElement('span');\n"
"   label.style.cssText='flex:1;overflow:hidden;text-overflow:ellipsis';\n"
"   label.textContent=x.name;\n"
"   label.title=x.url;\n"
"   const play=document.createElement('button');\n"
"   play.type='button';play.className='secondary';play.textContent='Abspielen';\n"
"   play.onclick=()=>radioPlay(x.name);\n"
"   const del=document.createElement('button');\n"
"   del.type='button';del.className='danger';del.textContent='Löschen';\n"
"   del.onclick=()=>radioDelete(x.name);\n"
"   row.appendChild(label);row.appendChild(play);row.appendChild(del);\n"
"   box.appendChild(row);\n"
"  });\n"
" }catch(e){}\n"
"}\n"
"\n"
"async function updCheck(){\n"
" toast('Frage GitHub ...');\n"
" const r=await fetch('/api/update/check',{method:'POST'});\n"
" toast(await r.text());\n"
" refreshStatus();\n"
"}\n"
"\n"
"async function updInstall(tag){\n"
" if(!confirm(tag+' installieren? Das Gerät startet danach neu.'))return;\n"
" const r=await fetch('/api/update/install?tag='+encodeURIComponent(tag),{method:'POST'});\n"
" toast(await r.text());\n"
"}\n"
"\n"
"async function radioPlay(name){\n"
" const r=await fetch('/api/radio/play?name='+encodeURIComponent(name),{method:'POST'});\n"
" toast(await r.text());\n"
" setTimeout(refreshRadio,600);\n"
"}\n"
"\n"
"async function radioStop(){\n"
" const r=await fetch('/api/radio/stop',{method:'POST'});\n"
" toast(await r.text());\n"
" setTimeout(refreshRadio,300);\n"
"}\n"
"\n"
"async function radioDelete(name){\n"
" if(!confirm(name+' aus der Liste nehmen?'))return;\n"
" const r=await fetch('/api/radio/delete?name='+encodeURIComponent(name),{method:'POST'});\n"
" toast(await r.text());\n"
" refreshRadio();\n"
"}\n"
"\n"
"async function radioSave(name,url){\n"
" const p=new URLSearchParams();p.set('name',name);p.set('url',url);\n"
" const r=await fetch('/api/radio/save',{method:'POST',\n"
"  headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});\n"
" toast(await r.text());\n"
" refreshRadio();\n"
"}\n"
"\n"
"function radioAdd(){\n"
" const n=$('radioName').value.trim(),u=$('radioUrl').value.trim();\n"
" if(!n||!u){toast('Name und URL bitte ausfüllen.');return;}\n"
" radioSave(n,u);$('radioName').value='';$('radioUrl').value='';\n"
"}\n"
"\n"
"// The search runs in the browser, not on the board: radio-browser.info allows\n"
"// cross origin requests, so the device needs no third party service at all.\n"
"async function radioSearch(){\n"
" const q=$('radioQuery').value.trim();\n"
" const box=$('radioHits');\n"
" if(!q){box.textContent='Bitte einen Suchbegriff eingeben.';return;}\n"
" box.textContent='Suche ...';\n"
" try{\n"
"  const u='https://de1.api.radio-browser.info/json/stations/search?hidebroken=true&limit=12&order=votes&reverse=true&name='+encodeURIComponent(q);\n"
"  const r=await fetch(u,{headers:{'Accept':'application/json'}});\n"
"  const j=await r.json();\n"
"  const mp3=j.filter(x=>(x.codec||'').toUpperCase()==='MP3'&&x.url_resolved);\n"
"  box.innerHTML='';\n"
"  if(!mp3.length){box.textContent='Keine MP3-Sender gefunden. AAC kann das Gerät nicht dekodieren.';return;}\n"
"  mp3.forEach(x=>{\n"
"   const row=document.createElement('div');\n"
"   row.style.cssText='display:flex;align-items:center;gap:8px;margin:3px 0';\n"
"   const label=document.createElement('span');\n"
"   label.style.cssText='flex:1;overflow:hidden;text-overflow:ellipsis';\n"
"   label.textContent=x.name.trim()+'  ('+(x.bitrate||'?')+' kbps'+(x.country?', '+x.country:'')+')';\n"
"   label.title=x.url_resolved;\n"
"   const add=document.createElement('button');\n"
"   add.type='button';add.textContent='Merken';\n"
"   add.onclick=()=>radioSave(x.name.trim(),x.url_resolved);\n"
"   row.appendChild(label);row.appendChild(add);\n"
"   box.appendChild(row);\n"
"  });\n"
" }catch(e){box.textContent='Suche fehlgeschlagen: '+e;}\n"
"}\n"
"\n"
"async function refreshSounds(){\n"
" try{\n"
"  const r=await fetch('/api/sound/list',{cache:'no-store'});\n"
"  const j=await r.json();\n"
"  const box=$('soundBox');\n"
"  box.innerHTML='';\n"
"  const files=j.files||[];\n"
"  if(!files.length){\n"
"   box.textContent='Noch keine Klänge hochgeladen.';\n"
"   return;\n"
"  }\n"
"  files.forEach(f=>{\n"
"   const row=document.createElement('div');\n"
"   row.style.cssText='display:flex;align-items:center;gap:8px;margin:3px 0';\n"
"   const label=document.createElement('span');\n"
"   label.style.cssText='flex:1;overflow:hidden;text-overflow:ellipsis';\n"
"   label.textContent=f.name+'  ('+Math.round(f.size/1024)+' kB)';\n"
"   const play=document.createElement('button');\n"
"   play.type='button';play.className='secondary';play.textContent='Abspielen';\n"
"   play.onclick=()=>soundPlay(f.name);\n"
"   const del=document.createElement('button');\n"
"   del.type='button';del.className='danger';del.textContent='Löschen';\n"
"   del.onclick=()=>soundDelete(f.name);\n"
"   row.appendChild(label);row.appendChild(play);row.appendChild(del);\n"
"   box.appendChild(row);\n"
"  });\n"
" }catch(e){}\n"
"}\n"
"\n"
"async function soundPlay(name){\n"
" const r=await fetch('/api/sound/play?name='+encodeURIComponent(name),{method:'POST'});\n"
" toast(await r.text());\n"
"}\n"
"\n"
"async function soundDelete(name){\n"
" if(!confirm(name+' löschen?'))return;\n"
" const r=await fetch('/api/sound/delete?name='+encodeURIComponent(name),{method:'POST'});\n"
" toast(await r.text());\n"
" refreshSounds();\n"
"}\n"
"\n"
"async function applyVad(){\n"
" const p=new URLSearchParams();\n"
" p.set('vad_release_db',$('vad_release_db').value);\n"
" p.set('vad_silence_ms',$('vad_silence_ms').value);\n"
" try{\n"
"  await fetch('/api/runtime',{\n"
"   method:'POST',\n"
"   headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'},\n"
"   body:p.toString()\n"
"  });\n"
"  toast('Empfindlichkeit übernommen');\n"
" }catch(e){}\n"
"}\n"
"\n"
"async function loadPipelines(){\n"
" $('pipeHint').textContent='Frage Home Assistant ...';\n"
" const r=await fetch('/api/ha/pipelines',{method:'POST'});\n"
" toast(await r.text());\n"
" setTimeout(refreshStatus,2500);\n"
"}\n"
"\n"
"async function ackBuild(){\n"
" const r=await fetch('/api/ack/build',{method:'POST'});\n"
" toast(await r.text());\n"
"}\n"
"\n"
"async function ackTest(){\n"
" const r=await fetch('/api/ack/test',{method:'POST'});\n"
" toast(await r.text());\n"
"}\n"
"\n"
"async function reboot(){\n"
" if(!confirm('VoiceDot neu starten?'))return;\n"
" await fetch('/api/system/reboot',{method:'POST'});\n"
" toast('Neustart ...');\n"
"}\n"
"\n"
"\n"
"$('soundForm').addEventListener('submit',async e=>{\n"
" e.preventDefault();\n"
" const f=$('soundFile').files[0];\n"
" if(!f){toast('Bitte eine Datei auswählen.');return}\n"
" const fd=new FormData();\n"
" fd.append('sound',f);\n"
" toast('Upload läuft ...');\n"
" try{\n"
"  const r=await fetch('/api/sound/upload',{method:'POST',body:fd});\n"
"  toast(await r.text());\n"
"  $('soundFile').value='';\n"
"  refreshSounds();\n"
" }catch(e){toast('Upload fehlgeschlagen.')}\n"
"});\n"
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
"(function(){\n"
" let t='dark',pg='uebersicht';\n"
" try{t=localStorage.getItem('vd_theme')||'dark';\n"
"     pg=localStorage.getItem('vd_page')||'uebersicht';}catch(e){}\n"
" applyTheme(t);\n"
" if(!document.getElementById('page-'+pg))pg='uebersicht';\n"
" showPage(pg);\n"
"})();\n"
"\n"
"loadConfig();\n"
"refreshStatus();\n"
"refreshLog();\n"
"refreshSounds();\n"
"refreshRadio();\n"
"refreshAlarm();\n"
"refreshGroups();\n"
"setInterval(refreshRadio,5000);\n"
"setInterval(refreshAlarm,2000);\n"
"setInterval(refreshStatus,3000);\n"
"setInterval(refreshLog,1500);\n"
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
  // Refresh device presence dynamically, but not while the audio path is busy:
  // this handler runs from pumpServices() inside the playback loop.
  if (!ttsPlaybackActive && !speakerTestActive && !wakeRecording) {
    es8311Present = i2cProbe(ADDR_ES8311);
    es7210Present = i2cProbe(ADDR_ES7210);
    tca9555Present = i2cProbe(ADDR_TCA9555);
    rtcPresent = i2cProbe(ADDR_RTC);
  }

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
  json += "\"min_free_heap\":" + String(ESP.getMinFreeHeap()) + ",";
  json += "\"max_alloc_heap\":" + String(ESP.getMaxAllocHeap()) + ",";
  json += "\"ap_mode\":" + String(apMode ? "true" : "false") + ",";
  json += "\"hostname\":\"" + jsonEscape(deviceHostname()) + "\",";
  {
    char id[13];
    snprintf(id, sizeof(id), "%012llx", (unsigned long long)ESP.getEfuseMac());
    json += "\"device_id\":\"" + String(id) + "\",";
  }
  json += "\"device_name\":\"" + jsonEscape(cfg.deviceName) + "\",";
  json += "\"volume\":" + String(cfg.volume) + ",";
  json += "\"muted\":" + String(muted ? "true" : "false") + ",";

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
  json += "\"amp_enabled\":" + String(amplifierEnabled ? "true" : "false") + ",";
  json += "\"led_phase\":\"" + String(ledPhaseName()) + "\",";
  json += "\"reset_reason\":\"" + jsonEscape(resetReasonName()) + "\"";
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
  json += "\"mic_last_bytes\":" + String(micLastBytes) + ",";

  // Everything the sensitivity depends on, so the web UI can show why a
  // recording ended when it did.
  {
    float floorDb = srRoomNoiseValid ? srRoomNoiseDb : wakeNoiseFloorDb;
    float speechDb = floorDb + VAD_SPEECH_MARGIN_DB;
    if (speechDb < VAD_MIN_SPEECH_DB) speechDb = VAD_MIN_SPEECH_DB;

    json += "\"room_noise_db\":" + String(floorDb, 1) + ",";
    json += "\"speech_threshold_db\":" + String(speechDb, 1) + ",";
    json += "\"release_threshold_db\":" + String(floorDb + (float)cfg.vadReleaseDb, 1) + ",";
    json += "\"noise_age_ms\":" + String(srRoomNoiseValid ? (millis() - srRoomNoiseAt) : 0);
  }
  json += "},";

  json += "\"assist\":{";
  json += "\"busy\":" + String(wakeBusy ? "true" : "false") + ",";
  json += "\"recording\":" + String(wakeRecording ? "true" : "false") + ",";
  json += "\"sending\":" + String(wakeSending ? "true" : "false") + ",";
  json += "\"state\":\"" + jsonEscape(wakeLastState) + "\",";
  json += "\"message\":\"" + jsonEscape(wakeLastMessage) + "\",";
  json += "\"ws_stage\":\"" + jsonEscape(wakeLastWsStage) + "\",";
  json += "\"ws_detail\":\"" + jsonEscape(wakeLastWsDetail) + "\",";
  json += "\"bytes\":" + String(wakeLastBytes) + ",";
  json += "\"duration_ms\":" + String(wakeLastDurationMs) + ",";
  json += "\"ws_code\":" + String(wakeLastHttpCode) + ",";
  json += "\"transcript\":\"" + jsonEscape(wakeTranscript) + "\",";
  json += "\"assistant_text\":\"" + jsonEscape(wakeAssistantText) + "\",";
  json += "\"tts_url\":\"" + jsonEscape(wakeTtsUrl) + "\",";
  json += "\"tts_status\":\"" + jsonEscape(wakeTtsStatus) + "\",";
  json += "\"conversation_id\":\"" + jsonEscape(haConversationId) + "\",";
  json += "\"pipeline\":\"" + jsonEscape(cfg.haPipeline) + "\",";
  json += "\"vad\":" + String(cfg.vadEnabled ? "true" : "false") + ",";
  json += "\"follow_up\":" + String(cfg.followUp ? "true" : "false") + ",";
  json += "\"noise_floor_db\":" + String(wakeNoiseFloorDb, 1) + ",";
  json += "\"silence_ms\":" + String(wakeLastVadSilenceMs) + ",";
  json += "\"queued\":" + String(wakeRequested ? "true" : "false") + ",";
  json += "\"local_command\":" + String(localCommandHandled ? "true" : "false") + ",";
  json += "\"source\":\"" + jsonEscape(String(wakeRequestSource)) + "\"";
  json += "},";

  json += "\"wake_word\":{";
  json += "\"enabled\":" + String(cfg.wakeWord ? "true" : "false") + ",";
  json += "\"available\":" + String(srAvailable ? "true" : "false") + ",";
  json += "\"running\":" + String(srRunning ? "true" : "false") + ",";
  json += "\"paused\":" + String(srPaused ? "true" : "false") + ",";
  json += "\"word\":\"" + jsonEscape(srActiveWords) + "\",";
  json += "\"model\":\"" + jsonEscape(srActiveModel) + "\",";
  json += "\"models\":[";
  for (uint8_t i = 0; i < srModelCount; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":\"" + jsonEscape(srModelId[i]) + "\",";
    json += "\"words\":\"" + jsonEscape(srModelWords[i]) + "\"}";
  }
  json += "],";
  json += "\"detections\":" + String(srDetections) + ",";
  json += "\"last_ms_ago\":" + String(srLastDetectMs > 0 ? (millis() - srLastDetectMs) : 0) + ",";
  json += "\"room_noise_db\":" + String(srRoomNoiseValid ? srRoomNoiseDb : -96.0f, 1) + ",";
  json += "\"status\":\"" + jsonEscape(srStatus) + "\"";
  json += "},";

  json += "\"schedule\":{";
  json += "\"enabled\":" + String(cfg.scheduleEnabled ? "true" : "false") + ",";
  json += "\"time_valid\":" + String(timeValid ? "true" : "false") + ",";
  json += "\"rtc\":" + String(rtcPresent ? "true" : "false") + ",";
  {
    struct tm t;
    if (getLocalTime(&t, 5)) {
      char now[6];
      snprintf(now, sizeof(now), "%02d:%02d", t.tm_hour, t.tm_min);
      json += "\"now\":\"" + String(now) + "\",";
      json += "\"dst\":" + String(t.tm_isdst > 0 ? "true" : "false") + ",";
      json += "\"tzname\":\"" + String(t.tm_isdst > 0 ? "MESZ" : "MEZ") + "\",";
    } else {
      json += "\"now\":\"--:--\",";
      json += "\"dst\":false,";
      json += "\"tzname\":\"-\",";
    }
  }
  json += "\"period\":\"" + String(scheduleApplied < 0 ? "-" : (scheduleApplied ? "Nacht" : "Tag")) + "\"";
  json += "},";

  json += "\"update\":{";
  json += "\"installed\":\"" + String(FW_VERSION) + "\",";
  json += "\"status\":\"" + jsonEscape(updateStatus) + "\",";
  json += "\"checked\":" + String(updateChecked ? "true" : "false") + ",";
  json += "\"checking_enabled\":" + String(cfg.updateCheckEnabled ? "true" : "false") + ",";
  json += "\"in_progress\":" + String(updateInProgress ? "true" : "false") + ",";
  json += "\"progress\":" + String(updateProgress) + ",";
  json += "\"age_s\":" + String(updateChecked ? (millis() - updateCheckedAt) / 1000UL : 0UL) + ",";
  {
    int newest = updateNewestIndex();
    json += "\"latest\":\"" + jsonEscape(newest >= 0 ? updateTags[newest] : "") + "\",";
    json += "\"available\":" +
            String(newest >= 0 && updateIsNewer(updateTags[newest]) ? "true" : "false") + ",";
  }
  json += "\"releases\":[";
  for (uint8_t i = 0; i < updateReleaseCount; i++) {
    if (i > 0) json += ",";
    json += "{\"tag\":\"" + jsonEscape(updateTags[i]) + "\",";
    json += "\"firmware\":" + String(updateUrls[i].length() > 0 ? "true" : "false") + ",";
    json += "\"newer\":" + String(updateIsNewer(updateTags[i]) ? "true" : "false") + "}";
  }
  json += "]},";

  json += "\"auto_volume\":{";
  json += "\"enabled\":" + String(cfg.autoVolumeEnabled ? "true" : "false") + ",";
  json += "\"max_db\":" + String(cfg.autoVolumeMaxDb) + ",";
  json += "\"quiet_db\":" + String(AUTO_VOLUME_QUIET_DB, 1) + ",";
  json += "\"noise_db\":" + String(autoVolumeNoiseValid ? autoVolumeNoiseDb : -96.0f, 1) + ",";
  json += "\"boost_db\":" + String(autoVolumeBoostDb(), 1) + ",";
  json += "\"last_boost_db\":" + String(autoVolumeLastBoost, 1) + ",";
  json += "\"age_s\":" + String(autoVolumeNoiseValid ? (millis() - autoVolumeNoiseAt) / 1000UL : 0UL);
  json += "},";

  json += "\"groups\":[";
  for (uint8_t i = 0; i < groupCount; i++) {
    if (i > 0) json += ",";
    json += "\"" + jsonEscape(groupNames[i]) + "\"";
  }
  json += "],";

  json += "\"alarm\":{";
  json += "\"set\":" + String(cfg.alarmMinutes >= 0 ? "true" : "false") + ",";
  json += "\"time\":\"" + clockText(cfg.alarmMinutes) + "\",";
  json += "\"minutes\":" + String(cfg.alarmMinutes) + ",";
  json += "\"daily\":" + String(cfg.alarmDaily ? "true" : "false") + ",";
  json += "\"seconds_until\":" + String(alarmSecondsUntil()) + ",";
  json += "\"ringing\":" + String(alarmRinging ? "true" : "false") + ",";
  json += "\"sound\":\"" + jsonEscape(cfg.alarmSound) + "\",";
  json += "\"briefing\":\"" + jsonEscape(cfg.alarmBriefing) + "\"";
  json += "},";

  json += "\"timer\":{";
  json += "\"active\":" + String(timerEndsAt != 0 ? "true" : "false") + ",";
  json += "\"remaining_s\":" + String(timerSecondsLeft()) + ",";
  json += "\"total_s\":" + String(timerTotalSec) + ",";
  json += "\"sound\":\"" + jsonEscape(cfg.timerSound) + "\"";
  json += "},";

  json += "\"radio\":{";
  json += "\"active\":" + String(radioActive ? "true" : "false") + ",";
  json += "\"paused\":" + String(radioPausedForTurn ? "true" : "false") + ",";
  json += "\"station\":\"" + jsonEscape(radioStationName) + "\",";
  json += "\"url\":\"" + jsonEscape(radioStationUrl) + "\",";
  json += "\"status\":\"" + jsonEscape(radioStatus) + "\",";
  json += "\"rate\":" + String(radioSampleRate) + ",";
  json += "\"channels\":" + String(radioChannels) + ",";
  json += "\"tls\":" + String(radioUsingTls ? "true" : "false") + ",";
  json += "\"kbytes\":" + String(radioBytesIn / 1024UL) + ",";
  json += "\"underruns\":" + String(radioUnderruns) + ",";
  json += "\"reconnects\":" + String(radioReconnects) + ",";
  json += "\"seconds\":" + String(radioActive ? (millis() - radioStartedAt) / 1000UL : 0UL) + ",";
  json += "\"stations\":[";
  for (uint8_t i = 0; i < radioStationCount; i++) {
    if (i > 0) json += ",";
    json += "\"" + jsonEscape(radioStationNames[i]) + "\"";
  }
  json += "]},";

  json += "\"multi\":{";
  json += "\"enabled\":" + String(cfg.multiEnabled ? "true" : "false") + ",";
  json += "\"ready\":" + String(multiReady ? "true" : "false") + ",";
  json += "\"id\":\"" + jsonEscape(multiOwnId()) + "\",";
  json += "\"window_ms\":" + String(cfg.multiWindowMs) + ",";
  json += "\"score\":" + String(multiWakeScore(), 1) + ",";
  json += "\"peak_db\":" + String(srRecentPeakDb, 1) + ",";
  json += "\"decision\":\"" + jsonEscape(multiLastDecision) + "\",";
  json += "\"peers\":[";
  {
    bool first = true;
    for (uint8_t i = 0; i < multiPeerCount; i++) {
      uint32_t age = millis() - multiPeers[i].lastSeen;
      if (age >= MULTI_PEER_TTL_MS) continue;
      if (!first) json += ",";
      first = false;
      json += "{\"id\":\"" + jsonEscape(multiPeers[i].id) + "\",";
      json += "\"name\":\"" + jsonEscape(multiPeers[i].name) + "\",";
      json += "\"ip\":\"" + jsonEscape(multiPeers[i].ip) + "\",";
      json += "\"age_s\":" + String(age / 1000UL) + ",";
      json += "\"last_score\":" + String(multiPeers[i].lastScore, 1) + "}";
    }
  }
  json += "]},";

  json += "\"ha_state\":{";
  json += "\"enabled\":" + String(cfg.haPublish ? "true" : "false") + ",";
  json += "\"entity\":\"" + jsonEscape(haEntityId) + "\",";
  json += "\"status\":\"" + jsonEscape(haPublishStatus) + "\"";
  json += "},";

  json += "\"pipelines\":{";
  json += "\"status\":\"" + jsonEscape(pipelineListStatus) + "\",";
  json += "\"preferred\":\"" + jsonEscape(pipelinePreferred) + "\",";
  json += "\"list\":[";
  for (uint8_t i = 0; i < pipelineCount; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":\"" + jsonEscape(pipelineId[i]) + "\",";
    json += "\"name\":\"" + jsonEscape(pipelineName[i]) + "\",";
    json += "\"voice\":\"" + jsonEscape(pipelineVoice[i]) + "\"}";
  }
  json += "]},";

  json += "\"ack\":{";
  json += "\"enabled\":" + String(cfg.ackEnabled ? "true" : "false") + ",";
  json += "\"clips\":" + String(ackClipCount) + ",";
  json += "\"fs\":" + String(ackFsReady ? "true" : "false") + ",";
  json += "\"engine\":\"" + jsonEscape(haTtsEngine) + "\",";
  json += "\"language\":\"" + jsonEscape(haTtsLanguage) + "\",";
  json += "\"voice\":\"" + jsonEscape(haTtsVoice) + "\",";
  json += "\"building\":" + String(ackBuildRequested ? "true" : "false") + ",";
  json += "\"announce\":\"" + jsonEscape(announceStatus) + "\",";
  json += "\"status\":\"" + jsonEscape(ackBuildStatus) + "\"";
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
  json += "\"ha_pipeline\":\"" + jsonEscape(cfg.haPipeline) + "\",";
  json += "\"volume\":" + String(cfg.volume) + ",";
  json += "\"led_brightness\":" + String(cfg.ledBrightness) + ",";
  json += "\"vad\":" + String(cfg.vadEnabled ? "true" : "false") + ",";
  json += "\"follow_up\":" + String(cfg.followUp ? "true" : "false") + ",";
  json += "\"wake_word\":" + String(cfg.wakeWord ? "true" : "false") + ",";
  json += "\"wake_model\":\"" + jsonEscape(cfg.wakeWordModel.length() > 0
                                                ? cfg.wakeWordModel
                                                : srActiveModel) + "\",";
  json += "\"ack_enabled\":" + String(cfg.ackEnabled ? "true" : "false") + ",";
  json += "\"ack_phrases\":\"" + jsonEscape(cfg.ackPhrases) + "\",";
  json += "\"listen_color\":\"#" + String(cfg.listenColor, HEX) + "\",";
  json += "\"speak_color\":\"#" + String(cfg.speakColor, HEX) + "\",";
  json += "\"tts_speed\":" + String(cfg.ttsSpeed) + ",";
  json += "\"vad_release_db\":" + String(cfg.vadReleaseDb) + ",";
  json += "\"vad_silence_ms\":" + String(cfg.vadSilenceMs) + ",";
  json += "\"schedule_enabled\":" + String(cfg.scheduleEnabled ? "true" : "false") + ",";
  json += "\"day_start\":\"" + minutesToHhMm(cfg.dayStartMin) + "\",";
  json += "\"night_start\":\"" + minutesToHhMm(cfg.nightStartMin) + "\",";
  json += "\"day_volume\":" + String(cfg.dayVolumeStep) + ",";
  json += "\"day_brightness\":" + String(cfg.dayBrightness) + ",";
  json += "\"night_volume\":" + String(cfg.nightVolumeStep) + ",";
  json += "\"night_brightness\":" + String(cfg.nightBrightness) + ",";
  json += "\"timezone\":\"" + jsonEscape(cfg.timezone) + "\",";
  json += "\"ha_publish\":" + String(cfg.haPublish ? "true" : "false") + ",";
  json += "\"clean_markdown\":" + String(cfg.cleanMarkdown ? "true" : "false") + ",";
  json += "\"mic_gain_db\":" + String(cfg.micGainDb) + ",";
  json += "\"stt_ha_vad\":" + String(cfg.sttHaVad ? "true" : "false") + ",";
  json += "\"stt_noise_suppression\":" + String(cfg.sttNoiseSuppression) + ",";
  json += "\"stt_auto_gain_db\":" + String(cfg.sttAutoGainDb) + ",";
  json += "\"stt_volume_percent\":" + String(cfg.sttVolumePercent) + ",";
  json += "\"volume_step\":" + String(cfg.volumeStep) + ",";
  json += "\"update_check\":" + String(cfg.updateCheckEnabled ? "true" : "false") + ",";
  json += "\"auto_volume\":" + String(cfg.autoVolumeEnabled ? "true" : "false") + ",";
  json += "\"auto_volume_max_db\":" + String(cfg.autoVolumeMaxDb) + ",";
  json += "\"multi_enabled\":" + String(cfg.multiEnabled ? "true" : "false") + ",";
  json += "\"multi_window_ms\":" + String(cfg.multiWindowMs) + ",";
  json += "\"hostname\":\"" + jsonEscape(deviceHostname()) + "\"";
  json += "}";

  server.send(200, "application/json; charset=utf-8", json);
}

// Everything except the WiFi credentials and the device name can be applied
// while the device keeps running, so only those two force a restart.
void handlePostConfig() {
  bool needsRestart = false;

  if (server.hasArg("wifi_ssid")) {
    String oldSsid = cfg.wifiSsid;
    String newSsid = server.arg("wifi_ssid");
    newSsid.trim();

    // If SSID changes and password field is empty, clear old password.
    if (newSsid != oldSsid && server.arg("wifi_pass").isEmpty()) {
      cfg.wifiPass = "";
    }

    if (newSsid != oldSsid) needsRestart = true;
    cfg.wifiSsid = newSsid;
  }

  if (server.hasArg("wifi_pass")) {
    String p = server.arg("wifi_pass");
    if (!p.isEmpty() && p != cfg.wifiPass) {
      cfg.wifiPass = p;
      needsRestart = true;
    }
  }

  if (server.hasArg("device_name")) {
    String name = server.arg("device_name");
    name.trim();
    if (name.isEmpty()) name = "VoiceDot";
    if (name != cfg.deviceName) {
      cfg.deviceName = name;
      needsRestart = true;  // hostname and mDNS record are set up at boot
    }
  }

  if (server.hasArg("ha_url")) {
    cfg.haUrl = normalizedHaUrl(server.arg("ha_url"));
  }

  if (server.hasArg("ha_token")) {
    String token = server.arg("ha_token");
    if (!token.isEmpty()) cfg.haToken = token;
  }

  if (server.hasArg("ha_pipeline")) {
    cfg.haPipeline = server.arg("ha_pipeline");
    cfg.haPipeline.trim();
    haConversationId = "";
  }

  if (server.hasArg("volume")) {
    cfg.volume = constrain(server.arg("volume").toInt(), 0, 100);
    muted = (cfg.volume == 0);
    if (cfg.volume > 0) volumeBeforeMute = cfg.volume;
    es8311SetVolume(cfg.volume);
  }

  if (server.hasArg("led_brightness")) {
    cfg.ledBrightness = constrain(server.arg("led_brightness").toInt(),
                                  0, LED_BRIGHTNESS_MAX);
    applyLedBrightness();
    setLedPhase(ledPhase);
  }

  if (server.hasArg("vad")) {
    cfg.vadEnabled = server.arg("vad") == "1" || server.arg("vad") == "true";
  }

  if (server.hasArg("vad_release_db")) {
    cfg.vadReleaseDb = constrain(server.arg("vad_release_db").toInt(), 3, 18);
  }

  if (server.hasArg("vad_silence_ms")) {
    cfg.vadSilenceMs = constrain(server.arg("vad_silence_ms").toInt(), 600, 3000);
  }

  bool scheduleTouched = false;

  if (server.hasArg("schedule_enabled")) {
    bool wanted = server.arg("schedule_enabled") == "1" ||
                  server.arg("schedule_enabled") == "true";
    if (wanted != cfg.scheduleEnabled) scheduleTouched = true;
    cfg.scheduleEnabled = wanted;
  }

  if (server.hasArg("day_start")) {
    cfg.dayStartMin = hhMmToMinutes(server.arg("day_start"), cfg.dayStartMin);
    scheduleTouched = true;
  }

  if (server.hasArg("night_start")) {
    cfg.nightStartMin = hhMmToMinutes(server.arg("night_start"), cfg.nightStartMin);
    scheduleTouched = true;
  }

  if (server.hasArg("day_volume")) {
    cfg.dayVolumeStep = constrain(server.arg("day_volume").toInt(), 0, 10);
    scheduleTouched = true;
  }

  if (server.hasArg("day_brightness")) {
    cfg.dayBrightness = constrain(server.arg("day_brightness").toInt(), 0, LED_BRIGHTNESS_MAX);
    scheduleTouched = true;
  }

  if (server.hasArg("night_volume")) {
    cfg.nightVolumeStep = constrain(server.arg("night_volume").toInt(), 0, 10);
    scheduleTouched = true;
  }

  if (server.hasArg("night_brightness")) {
    cfg.nightBrightness = constrain(server.arg("night_brightness").toInt(), 0, LED_BRIGHTNESS_MAX);
    scheduleTouched = true;
  }

  if (server.hasArg("mic_gain_db")) {
    cfg.micGainDb = constrain(server.arg("mic_gain_db").toInt(), 0, 36);
    es7210ApplyGain(cfg.micGainDb);
  }

  if (server.hasArg("stt_ha_vad")) {
    cfg.sttHaVad = server.arg("stt_ha_vad") == "1" || server.arg("stt_ha_vad") == "true";
  }

  if (server.hasArg("stt_noise_suppression")) {
    cfg.sttNoiseSuppression = constrain(server.arg("stt_noise_suppression").toInt(), 0, 4);
  }

  if (server.hasArg("stt_auto_gain_db")) {
    cfg.sttAutoGainDb = constrain(server.arg("stt_auto_gain_db").toInt(), 0, 31);
  }

  if (server.hasArg("stt_volume_percent")) {
    cfg.sttVolumePercent = constrain(server.arg("stt_volume_percent").toInt(), 100, 400);
  }

  if (server.hasArg("volume_step")) {
    cfg.volumeStep = constrain(server.arg("volume_step").toInt(), 5, 25);
  }

  if (server.hasArg("update_check")) {
    cfg.updateCheckEnabled = server.arg("update_check") == "1" ||
                             server.arg("update_check") == "true";
  }

  if (server.hasArg("auto_volume")) {
    cfg.autoVolumeEnabled = server.arg("auto_volume") == "1" ||
                            server.arg("auto_volume") == "true";
  }

  if (server.hasArg("auto_volume_max_db")) {
    cfg.autoVolumeMaxDb = constrain(server.arg("auto_volume_max_db").toInt(), 0, 18);
  }

  if (server.hasArg("multi_enabled")) {
    cfg.multiEnabled = server.arg("multi_enabled") == "1" ||
                       server.arg("multi_enabled") == "true";
  }

  if (server.hasArg("multi_window_ms")) {
    cfg.multiWindowMs = constrain(server.arg("multi_window_ms").toInt(), 80, 600);
  }

  if (server.hasArg("clean_markdown")) {
    cfg.cleanMarkdown = server.arg("clean_markdown") == "1" ||
                        server.arg("clean_markdown") == "true";
  }

  if (server.hasArg("ha_publish")) {
    cfg.haPublish = server.arg("ha_publish") == "1" || server.arg("ha_publish") == "true";
    if (cfg.haPublish) haPublishAt = 0;  // publish on the next loop pass
  }

  if (server.hasArg("timezone")) {
    String tz = server.arg("timezone");
    tz.trim();
    if (!tz.isEmpty() && tz != cfg.timezone) {
      cfg.timezone = tz;
      configTzTime(cfg.timezone.c_str(), "pool.ntp.org", "time.cloudflare.com");
      scheduleTouched = true;
    }
  }

  // Show the chosen profile straight away instead of waiting for the next
  // period change.
  if (scheduleTouched) applySchedule(true);

  if (server.hasArg("follow_up")) {
    cfg.followUp = server.arg("follow_up") == "1" || server.arg("follow_up") == "true";
  }

  if (server.hasArg("wake_word")) {
    bool wanted = server.arg("wake_word") == "1" || server.arg("wake_word") == "true";
    if (wanted != cfg.wakeWord) {
      cfg.wakeWord = wanted;
      // Starting or stopping the detector blocks on its tasks, so loop() does it.
      srApplyRequested = true;
    }
  }

  if (server.hasArg("ack_enabled")) {
    cfg.ackEnabled = server.arg("ack_enabled") == "1" || server.arg("ack_enabled") == "true";
  }

  if (server.hasArg("ack_phrases")) {
    cfg.ackPhrases = server.arg("ack_phrases");
    cfg.ackPhrases.trim();
    if (cfg.ackPhrases.isEmpty()) cfg.ackPhrases = ACK_DEFAULT_PHRASES;
  }

  if (server.hasArg("listen_color")) {
    String col = server.arg("listen_color");
    col.trim();
    if (col.startsWith("#")) col.remove(0, 1);
    if (col.length() == 6) {
      cfg.listenColor = (uint32_t)strtoul(col.c_str(), nullptr, 16) & 0xFFFFFF;
    }
  }

  if (server.hasArg("speak_color")) {
    String col = server.arg("speak_color");
    col.trim();
    if (col.startsWith("#")) col.remove(0, 1);
    if (col.length() == 6) {
      cfg.speakColor = (uint32_t)strtoul(col.c_str(), nullptr, 16) & 0xFFFFFF;
      // Show the new colour right away if the ring is already pulsing.
      if (ledPhase == LED_PHASE_SPEAK) setLedPhase(LED_PHASE_SPEAK);
    }
  }

  if (server.hasArg("tts_speed")) {
    cfg.ttsSpeed = constrain(server.arg("tts_speed").toInt(),
                             TTS_SPEED_MIN, TTS_SPEED_MAX);
  }

  if (server.hasArg("wake_model")) {
    String model = server.arg("wake_model");
    model.trim();
    if (model != cfg.wakeWordModel) {
      cfg.wakeWordModel = model;
      srApplyRequested = true;  // reloading the AFE takes too long for a handler
    }
  }

  saveConfig();

  if (needsRestart) {
    server.send(200, "text/plain; charset=utf-8",
                "Gespeichert. VoiceDot startet neu ...");
    scheduleRestart(1100);
  } else {
    server.send(200, "text/plain; charset=utf-8",
                "Gespeichert und sofort übernommen.");
  }
}

// Lightweight endpoint for the sliders: applies without saving a full config
// or restarting anything.
void handleRuntime() {
  if (server.hasArg("speak_color")) {
    String col = server.arg("speak_color");
    col.trim();
    if (col.startsWith("#")) col.remove(0, 1);
    if (col.length() == 6) {
      cfg.speakColor = (uint32_t)strtoul(col.c_str(), nullptr, 16) & 0xFFFFFF;
      if (ledPhase == LED_PHASE_SPEAK) setLedPhase(LED_PHASE_SPEAK);
      saveRuntimeConfig();
    }
  }

  if (server.hasArg("listen_color")) {
    String col = server.arg("listen_color");
    col.trim();
    if (col.startsWith("#")) col.remove(0, 1);
    if (col.length() == 6) {
      cfg.listenColor = (uint32_t)strtoul(col.c_str(), nullptr, 16) & 0xFFFFFF;
      saveRuntimeConfig();
    }
  }

  if (server.hasArg("vad_release_db")) {
    cfg.vadReleaseDb = constrain(server.arg("vad_release_db").toInt(), 3, 18);
    prefs.begin("voicedot", false);
    prefs.putUChar("vad_rel_db", cfg.vadReleaseDb);
    prefs.end();
  }

  if (server.hasArg("vad_silence_ms")) {
    cfg.vadSilenceMs = constrain(server.arg("vad_silence_ms").toInt(), 600, 3000);
    prefs.begin("voicedot", false);
    prefs.putUShort("vad_sil_ms", cfg.vadSilenceMs);
    prefs.end();
  }

  if (server.hasArg("tts_speed")) {
    cfg.ttsSpeed = constrain(server.arg("tts_speed").toInt(),
                             TTS_SPEED_MIN, TTS_SPEED_MAX);
    saveRuntimeConfig();
  }

  if (server.hasArg("volume")) {
    setVolume(constrain(server.arg("volume").toInt(), 0, 100));
  }

  if (server.hasArg("led_brightness")) {
    cfg.ledBrightness = constrain(server.arg("led_brightness").toInt(),
                                  0, LED_BRIGHTNESS_MAX);
    applyLedBrightness();
    setLedPhase(ledPhase);
    saveRuntimeConfig();
  }

  server.send(200, "text/plain; charset=utf-8", "OK");
}

// Returns everything newer than the client's last line count, so polling stays
// cheap. A total lower than "since" means the board restarted in between.
void handleLog() {
  uint32_t since = 0;
  if (server.hasArg("since")) {
    since = (uint32_t)strtoul(server.arg("since").c_str(), nullptr, 10);
  }

  portENTER_CRITICAL(&logMux);
  uint32_t total = logTotal;
  uint16_t head = logHead;
  portEXIT_CRITICAL(&logMux);

  uint32_t oldest = total > LOG_LINES ? total - LOG_LINES : 0;
  if (since < oldest) since = oldest;
  if (since > total) since = total;

  String json;
  json.reserve(256 + (size_t)(total - since) * 96);
  json += "{\"total\":" + String(total) + ",\"lines\":[";

  for (uint32_t i = since; i < total; i++) {
    uint16_t slot = (uint16_t)((head + LOG_LINES - (uint16_t)(total - i)) % LOG_LINES);
    if (i > since) json += ",";
    json += "\"" + jsonEscape(String(logRing[slot])) + "\"";
  }

  json += "]}";
  server.send(200, "application/json; charset=utf-8", json);
}

// -----------------------------------------------------------------------------
// Home Assistant state
//
// Pushed over the REST API with the token we already have, so no MQTT broker is
// needed. Entities created this way are read only from HA's side - control goes
// the other way round, through this device's own endpoints.
// -----------------------------------------------------------------------------

static String voiceDotState() {
  if (wakeRecording) return "listening";
  if (wakeSending) return "thinking";
  if (ttsPlaybackActive) return "speaking";
  if (wakeLastState == "error") return "error";
  return "idle";
}

static void haPublishNow() {
  if (!cfg.haPublish) return;
  if (WiFi.status() != WL_CONNECTED || cfg.haUrl.isEmpty() || cfg.haToken.isEmpty()) return;

  haEntityId = "sensor." + deviceHostname() + "_status";
  haEntityId.replace("-", "_");

  HTTPClient http;
  if (!http.begin(normalizedHaUrl(cfg.haUrl) + "/api/states/" + haEntityId)) {
    haPublishStatus = "HTTP-Client fehlgeschlagen";
    return;
  }

  http.useHTTP10(true);
  http.setTimeout(6000);
  http.addHeader("Authorization", "Bearer " + cfg.haToken);
  http.addHeader("Content-Type", "application/json");

  String body = "{\"state\":\"" + voiceDotState() + "\",\"attributes\":{";
  body += "\"friendly_name\":\"" + jsonEscape(cfg.deviceName) + "\",";
  body += "\"icon\":\"mdi:account-voice\",";
  body += "\"volume\":" + String(cfg.volume) + ",";
  body += "\"volume_step\":" + String((cfg.volume + 5) / 10) + ",";
  body += "\"muted\":" + String(muted ? "true" : "false") + ",";
  body += "\"wake_word\":\"" + jsonEscape(srActiveWords) + "\",";
  body += "\"wake_word_enabled\":" + String(cfg.wakeWord ? "true" : "false") + ",";
  body += "\"detections\":" + String(srDetections) + ",";
  body += "\"room_noise_db\":" + String(srRoomNoiseValid ? srRoomNoiseDb : -96.0f, 1) + ",";
  body += "\"transcript\":\"" + jsonEscape(wakeTranscript) + "\",";
  body += "\"answer\":\"" + jsonEscape(wakeAssistantText) + "\",";
  body += "\"profile\":\"" + String(scheduleApplied < 0 ? "-" : (scheduleApplied ? "Nacht" : "Tag")) + "\",";
  body += "\"uptime_s\":" + String(millis() / 1000UL) + ",";
  body += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  body += "\"min_free_heap\":" + String(ESP.getMinFreeHeap()) + ",";
  body += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  body += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  body += "\"firmware\":\"" + String(FW_VERSION) + "\"";
  body += "}}";

  int code = http.POST(body);
  http.end();

  haPublishStatus = (code == 200 || code == 201)
                    ? haEntityId + " aktualisiert"
                    : "HTTP " + String(code);
  haPublishAt = millis();

  if (code != 200 && code != 201) {
    diagLogf("HA_STATE", "push failed http=%d", code);
  }
}

// -----------------------------------------------------------------------------
// PCF85063 real time clock
//
// The board carries one and it was sitting unused. NTP stays the source of
// truth, but without it - no internet at boot, router still coming up - the
// day/night profile would have no idea what time it is.
// -----------------------------------------------------------------------------

static uint8_t bcdToDec(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t decToBcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// Reads UTC from the RTC. Returns false while the oscillator flag says the
// stored time is not trustworthy.
static bool rtcReadUtc(struct tm &out) {
  if (!rtcPresent) return false;

  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x04);  // seconds
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR_RTC, 7) != 7) return false;

  uint8_t sec = Wire.read();
  uint8_t minute = Wire.read();
  uint8_t hour = Wire.read();
  uint8_t day = Wire.read();
  Wire.read();  // weekday, derived by mktime instead
  uint8_t month = Wire.read();
  uint8_t year = Wire.read();

  if (sec & 0x80) return false;  // oscillator stopped, contents are garbage

  memset(&out, 0, sizeof(out));
  out.tm_sec = bcdToDec(sec & 0x7F);
  out.tm_min = bcdToDec(minute & 0x7F);
  out.tm_hour = bcdToDec(hour & 0x3F);
  out.tm_mday = bcdToDec(day & 0x3F);
  out.tm_mon = bcdToDec(month & 0x1F) - 1;
  out.tm_year = bcdToDec(year) + 100;  // years since 1900, RTC counts from 2000
  out.tm_isdst = 0;

  return out.tm_mon >= 0 && out.tm_mon <= 11 && out.tm_mday >= 1 && out.tm_mday <= 31;
}

static bool rtcWriteUtc(const struct tm &t) {
  if (!rtcPresent) return false;

  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x04);
  Wire.write(decToBcd((uint8_t)t.tm_sec) & 0x7F);  // clears the oscillator flag
  Wire.write(decToBcd((uint8_t)t.tm_min));
  Wire.write(decToBcd((uint8_t)t.tm_hour));
  Wire.write(decToBcd((uint8_t)t.tm_mday));
  Wire.write(decToBcd((uint8_t)t.tm_wday));
  Wire.write(decToBcd((uint8_t)(t.tm_mon + 1)));
  Wire.write(decToBcd((uint8_t)(t.tm_year - 100)));
  return Wire.endTransmission() == 0;
}

// newlib has no timegm, and mktime would apply the local timezone. Days from
// civil calendar (Howard Hinnant's algorithm) keeps this self contained.
static time_t utcToEpoch(const struct tm &t) {
  int y = t.tm_year + 1900;
  int m = t.tm_mon + 1;
  int d = t.tm_mday;

  y -= (m <= 2) ? 1 : 0;
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (unsigned)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097 + (long)doe - 719468;

  return (time_t)days * 86400L + t.tm_hour * 3600L + t.tm_min * 60L + t.tm_sec;
}

// Seeds the system clock from the RTC so local time works before NTP answers.
static bool clockFromRtc() {
  struct tm utc;
  if (!rtcReadUtc(utc)) return false;

  time_t epoch = utcToEpoch(utc);
  if (epoch < 1700000000) return false;  // clearly not a real date

  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  timeValid = true;

  struct tm local;
  getLocalTime(&local, 5);
  logPrintf("Zeit: aus RTC uebernommen, %02d:%02d lokal", local.tm_hour, local.tm_min);
  return true;
}

static void clockToRtc() {
  time_t now = time(nullptr);
  if (now < 1700000000) return;

  struct tm utc;
  gmtime_r(&now, &utc);
  if (rtcWriteUtc(utc)) {
    logPrintf("Zeit: RTC gestellt auf %04d-%02d-%02d %02d:%02d UTC",
              utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
              utc.tm_hour, utc.tm_min);
  }
}

// -----------------------------------------------------------------------------
// Day / night profile
// -----------------------------------------------------------------------------

// Minutes since midnight, or -1 while the clock has not been set yet.
static int localMinutesNow() {
  struct tm t;
  if (!getLocalTime(&t, 5)) return -1;
  timeValid = true;
  return t.tm_hour * 60 + t.tm_min;
}

static bool isNightAt(int minutes) {
  if (cfg.dayStartMin == cfg.nightStartMin) return false;

  if (cfg.nightStartMin > cfg.dayStartMin) {
    // The usual case: night wraps around midnight, e.g. 19:00 until 06:00.
    return minutes >= cfg.nightStartMin || minutes < cfg.dayStartMin;
  }
  return minutes >= cfg.nightStartMin && minutes < cfg.dayStartMin;
}

// Applies a profile only when the period actually changes, so a manual volume
// change during the day is not overwritten a minute later.
static void applySchedule(bool force) {
  if (!cfg.scheduleEnabled) {
    scheduleApplied = -1;
    return;
  }

  int minutes = localMinutesNow();
  if (minutes < 0) return;

  int8_t period = isNightAt(minutes) ? 1 : 0;
  if (!force && period == scheduleApplied) return;

  scheduleApplied = period;

  uint8_t step = period ? cfg.nightVolumeStep : cfg.dayVolumeStep;
  uint8_t brightness = period ? cfg.nightBrightness : cfg.dayBrightness;

  setVolume((uint8_t)constrain(step * 10, 0, 100));
  cfg.ledBrightness = brightness;
  applyLedBrightness();
  setLedPhase(ledPhase);

  logPrintf("Profil %s: Lautstaerke %u (%u%%), LED %u%% um %02d:%02d",
            period ? "Nacht" : "Tag",
            step, step * 10, brightness,
            minutes / 60, minutes % 60);
}

// -----------------------------------------------------------------------------
// Multi-device arbitration
//
// Two VoiceDots in adjoining rooms both hear "Alexa" - one loud, one faint -
// and without coordination both would start recording and both would send the
// same command to Home Assistant.
//
// On detection each device broadcasts a claim carrying how loudly it heard the
// word, then waits a moment for competing claims. The loudest wins and runs the
// dialogue; everyone else goes straight back to sleep without recording. Equal
// loudness is broken by the smaller device id, so every device reaches the same
// verdict without a coordinator.
//
// Devices announce themselves periodically. With no peers around the wait is
// skipped entirely, so a single VoiceDot pays nothing for this.
// -----------------------------------------------------------------------------

static String multiOwnId() {
  if (multiDeviceId.length() == 0) {
    char id[13];
    snprintf(id, sizeof(id), "%012llx", (unsigned long long)ESP.getEfuseMac());
    multiDeviceId = id;
  }
  return multiDeviceId;
}

static void multiBroadcast(const String &payload) {
  if (!multiReady || WiFi.status() != WL_CONNECTED) return;

  IPAddress bcast = WiFi.localIP();
  bcast[3] = 255;  // subnet broadcast, quieter than 255.255.255.255

  multiUdp.beginPacket(bcast, MULTI_PORT);
  multiUdp.write((const uint8_t*)payload.c_str(), payload.length());
  multiUdp.endPacket();
}

// Drops peers that have gone quiet, so a device that was switched off stops
// blocking the shortcut for everyone else.
static uint8_t multiActivePeers() {
  uint8_t alive = 0;
  for (uint8_t i = 0; i < multiPeerCount; i++) {
    if ((uint32_t)(millis() - multiPeers[i].lastSeen) < MULTI_PEER_TTL_MS) alive++;
  }
  return alive;
}

static VoiceDotPeer *multiFindPeer(const String &id, bool create) {
  for (uint8_t i = 0; i < multiPeerCount; i++) {
    if (multiPeers[i].id == id) return &multiPeers[i];
  }

  if (!create) return nullptr;

  if (multiPeerCount < MULTI_MAX_PEERS) {
    multiPeers[multiPeerCount].id = id;
    return &multiPeers[multiPeerCount++];
  }

  // Table full: recycle whichever entry has been silent the longest.
  uint8_t oldest = 0;
  for (uint8_t i = 1; i < multiPeerCount; i++) {
    if (multiPeers[i].lastSeen < multiPeers[oldest].lastSeen) oldest = i;
  }
  multiPeers[oldest] = VoiceDotPeer();
  multiPeers[oldest].id = id;
  return &multiPeers[oldest];
}

// True when `a` should win against `b`. Louder wins; equal loudness is decided
// by the id so that every device comes to the same conclusion.
static bool multiScoreWins(float aScore, const String &aId, float bScore, const String &bId) {
  if (aScore > bScore + MULTI_SCORE_TIE_DB) return true;
  if (bScore > aScore + MULTI_SCORE_TIE_DB) return false;
  return aId < bId;
}

static void multiHandleMessage(const String &msg, const IPAddress &from) {
  String type = jsonFindString(msg, "t");
  String id = jsonFindString(msg, "id");
  if (id.length() == 0 || id == multiOwnId()) return;

  VoiceDotPeer *peer = multiFindPeer(id, true);
  if (peer) {
    peer->lastSeen = millis();
    peer->ip = from.toString();
    String name = jsonFindString(msg, "name");
    if (name.length() > 0) peer->name = name;
  }

  if (type == "hello") return;

  if (type == "claim" || type == "win") {
    float score = (float)atof(jsonFindString(msg, "score").c_str());
    if (peer) {
      peer->lastScore = score;
      peer->lastWon = (type == "win");
    }

    arbLastClaimId = id;
    arbLastClaimScore = score;
    arbLastClaimAt = millis();

    if (type == "win") {
      // Somebody already declared themselves the winner. Only yield if they
      // really are louder - otherwise our own claim still stands.
      if (arbActive && multiScoreWins(score, id, arbBestScore, arbBestId)) {
        arbBestScore = score;
        arbBestId = id;
        arbLost = true;
      }
      return;
    }

    if (arbActive && multiScoreWins(score, id, arbBestScore, arbBestId)) {
      arbBestScore = score;
      arbBestId = id;
    }
  }
}

static void multiPoll() {
  if (!multiReady) return;

  int size = multiUdp.parsePacket();
  while (size > 0) {
    char buf[321];
    int len = multiUdp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      multiHandleMessage(String(buf), multiUdp.remoteIP());
    }
    size = multiUdp.parsePacket();
  }
}

static void multiSendHello() {
  String msg = "{\"t\":\"hello\",\"id\":\"" + multiOwnId() + "\"";
  msg += ",\"name\":\"" + jsonEscape(cfg.deviceName) + "\"";
  msg += ",\"fw\":\"" + String(FW_VERSION) + "\"}";
  multiBroadcast(msg);
}

static void multiSendClaim(const char *type, float score) {
  String msg = "{\"t\":\"" + String(type) + "\",\"id\":\"" + multiOwnId() + "\"";
  msg += ",\"name\":\"" + jsonEscape(cfg.deviceName) + "\"";
  msg += ",\"score\":\"" + String(score, 1) + "\"";
  msg += ",\"peak\":\"" + String(srRecentPeakDb, 1) + "\"";
  msg += ",\"noise\":\"" + String(srRoomNoiseValid ? srRoomNoiseDb : -96.0f, 1) + "\"}";
  multiBroadcast(msg);
}

static void multiBegin() {
  multiOwnId();
  if (WiFi.status() != WL_CONNECTED) return;

  multiReady = multiUdp.begin(MULTI_PORT);
  if (multiReady) {
    logPrintf("Multi-Dot: UDP %u bereit, id=%s", MULTI_PORT, multiOwnId().c_str());
    multiSendHello();
  } else {
    logPrintf("Multi-Dot: UDP-Port %u konnte nicht geoeffnet werden", MULTI_PORT);
  }
}

// How loudly this device heard the wake word, measured against its own room
// noise. Using the margin over the local noise floor rather than the raw level
// keeps a noisy room from winning just because it is noisy.
static float multiWakeScore() {
  float noise = srRoomNoiseValid ? srRoomNoiseDb : VAD_NOISE_FLOOR_START_DB;
  return srRecentPeakDb - noise;
}

// Returns true when this device should run the dialogue.
static bool multiArbitrate() {
  if (!cfg.multiEnabled) return true;

  if (!multiReady) {
    multiBegin();
    if (!multiReady) return true;
  }

  float score = multiWakeScore();

  if (multiActivePeers() == 0) {
    // Nobody else is around, so there is nothing to wait for.
    multiLastDecision = "allein, Score " + String(score, 1) + " dB";
    return true;
  }

  arbActive = true;
  arbLost = false;
  arbStartedAt = millis();
  arbBestScore = score;
  arbBestId = multiOwnId();

  // A neighbour that triggered a moment earlier has already sent its claim.
  if (arbLastClaimId.length() > 0 &&
      (uint32_t)(millis() - arbLastClaimAt) < MULTI_CLAIM_LOOKBACK_MS &&
      multiScoreWins(arbLastClaimScore, arbLastClaimId, arbBestScore, arbBestId)) {
    arbBestScore = arbLastClaimScore;
    arbBestId = arbLastClaimId;
  }

  multiSendClaim("claim", score);

  while ((uint32_t)(millis() - arbStartedAt) < cfg.multiWindowMs && !arbLost) {
    multiPoll();
    delay(2);
    yield();
  }

  arbActive = false;
  bool won = !arbLost && arbBestId == multiOwnId();

  if (won) {
    // Tell the others straight away instead of making them wait out the window.
    multiSendClaim("win", score);
    multiLastDecision = "gewonnen mit " + String(score, 1) + " dB";
  } else {
    String name = arbBestId;
    VoiceDotPeer *peer = multiFindPeer(arbBestId, false);
    if (peer && peer->name.length() > 0) name = peer->name;
    multiLastDecision = "abgegeben an " + name + " (" + String(arbBestScore, 1) +
                        " dB gegen " + String(score, 1) + " dB)";
  }

  diagLogf("MULTI", "%s", multiLastDecision.c_str());
  return won;
}

// -----------------------------------------------------------------------------
// Firmware updates from GitHub releases
//
// The releases on GitHub are the single source of truth: attach a .bin to a
// release and it shows up here, in the web interface and in Home Assistant,
// without a second list to keep in sync.
//
// The device caches what it found. The GitHub API allows 60 unauthenticated
// requests per hour and per address, which a Home Assistant integration polling
// every ten seconds would burn through in a minute - so everything else reads
// this cache instead of asking GitHub itself.
// -----------------------------------------------------------------------------

// "v0.8.1" and "0.8.1" both parse; anything after the third number is ignored.
static void updateParseVersion(const String &in, int out[3]) {
  out[0] = out[1] = out[2] = 0;

  String v = in;
  v.trim();
  if (v.startsWith("v") || v.startsWith("V")) v.remove(0, 1);

  uint8_t part = 0;
  String number = "";
  for (size_t i = 0; i <= v.length() && part < 3; i++) {
    char c = i < v.length() ? v[i] : '.';
    if (c >= '0' && c <= '9') {
      number += c;
      continue;
    }
    if (c == '.' || i == v.length()) {
      out[part++] = number.toInt();
      number = "";
      if (c != '.') break;
      continue;
    }
    break;  // a suffix like -rc1 ends the version
  }
}

// Positive when a is newer than b.
static int updateVersionCompare(const String &a, const String &b) {
  int va[3];
  int vb[3];
  updateParseVersion(a, va);
  updateParseVersion(b, vb);

  for (uint8_t i = 0; i < 3; i++) {
    if (va[i] != vb[i]) return va[i] > vb[i] ? 1 : -1;
  }
  return 0;
}

static bool updateIsNewer(const String &tag) {
  return updateVersionCompare(tag, String(FW_VERSION)) > 0;
}

// The newest release that actually carries a firmware image, or -1.
static int updateNewestIndex() {
  int best = -1;
  for (uint8_t i = 0; i < updateReleaseCount; i++) {
    if (updateUrls[i].length() == 0) continue;
    if (best < 0 || updateVersionCompare(updateTags[i], updateTags[best]) > 0) best = i;
  }
  return best;
}

static bool updateFetchReleasesInner();

bool updateFetchReleases() {
  bool yielded = netMakeRoom(UPDATE_TLS_MIN_HEAP);
  bool ok = updateFetchReleasesInner();
  if (yielded) netReleaseRoom();
  return ok;
}

static bool updateFetchReleasesInner() {
  if (WiFi.status() != WL_CONNECTED || apMode) {
    updateStatus = "kein WLAN";
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String("https://api.github.com/repos/") + UPDATE_REPO +
               "/releases?per_page=" + String(UPDATE_MAX_RELEASES);

  if (!http.begin(client, url)) {
    updateStatus = "Verbindung zu GitHub fehlgeschlagen";
    return false;
  }

  http.useHTTP10(true);
  http.setTimeout(12000);
  http.setReuse(false);
  // GitHub answers 403 to requests without a User-Agent.
  http.addHeader("User-Agent", String("VoiceDot/") + FW_VERSION);
  http.addHeader("Accept", "application/vnd.github+json");

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    if (code < 0) {
      updateStatus = "Abfrage fehlgeschlagen (frei " +
                     String(ESP.getFreeHeap() / 1024) + " kB, groesster Block " +
                     String(ESP.getMaxAllocHeap() / 1024) + " kB).";
    } else {
      updateStatus = "GitHub antwortet mit HTTP " + String(code);
    }
    diagLogf("UPDATE", "release list failed code=%d maxAlloc=%lu",
             code, (unsigned long)ESP.getMaxAllocHeap());
    http.end();
    return false;
  }

  // Scanned rather than buffered: release notes run into kilobytes and only two
  // fields per release are of any interest.
  static const char *KEY_TAG = "\"tag_name\":\"";
  static const char *KEY_URL = "\"browser_download_url\":\"";

  Client *stream = http.getStreamPtr();
  updateReleaseCount = 0;

  uint8_t tagPos = 0;
  uint8_t urlPos = 0;
  uint8_t capture = 0;  // 1 = tag, 2 = asset url
  bool escaped = false;
  int pending = -1;
  String value = "";
  uint32_t deadline = millis() + 20000;

  while ((http.connected() || stream->available() > 0) &&
         (int32_t)(millis() - deadline) < 0) {
    int waiting = stream->available();
    if (waiting <= 0) {
      ledTick();
      pumpServices();
      delay(4);
      continue;
    }

    while (waiting-- > 0) {
      int raw = stream->read();
      if (raw < 0) break;
      char c = (char)raw;

      if (capture != 0) {
        if (escaped) { value += c; escaped = false; continue; }
        if (c == '\\') { escaped = true; continue; }
        if (c != '"') {
          value += c;
          if (value.length() > 300) { capture = 0; value = ""; }
          continue;
        }

        if (capture == 1) {
          if (updateReleaseCount < UPDATE_MAX_RELEASES) {
            updateTags[updateReleaseCount] = value;
            updateUrls[updateReleaseCount] = "";
            pending = updateReleaseCount;
            updateReleaseCount++;
          } else {
            pending = -1;
          }
        } else if (pending >= 0 && value.endsWith(".bin") &&
                   updateUrls[pending].length() == 0) {
          updateUrls[pending] = value;
        }

        capture = 0;
        value = "";
        continue;
      }

      tagPos = (c == KEY_TAG[tagPos]) ? tagPos + 1 : (c == KEY_TAG[0] ? 1 : 0);
      urlPos = (c == KEY_URL[urlPos]) ? urlPos + 1 : (c == KEY_URL[0] ? 1 : 0);

      if (KEY_TAG[tagPos] == '\0') {
        capture = 1; tagPos = 0; urlPos = 0; value = "";
      } else if (KEY_URL[urlPos] == '\0') {
        capture = 2; tagPos = 0; urlPos = 0; value = "";
      }
    }

    ledTick();
    pumpServices();
  }

  http.end();
  updateCheckedAt = millis();
  updateChecked = true;

  int newest = updateNewestIndex();
  if (updateReleaseCount == 0) {
    updateStatus = "keine Releases gefunden";
  } else if (newest < 0) {
    updateStatus = String(updateReleaseCount) + " Releases, aber keines mit Firmware";
  } else if (updateIsNewer(updateTags[newest])) {
    updateStatus = updateTags[newest] + " steht bereit";
  } else {
    updateStatus = "aktuell";
  }

  diagLogf("UPDATE", "found %u releases, newest with firmware: %s",
           updateReleaseCount,
           newest >= 0 ? updateTags[newest].c_str() : "-");
  return true;
}

// Remembers what to install across the reboot. The release list does not
// survive it, so the URL travels along and no second GitHub call is needed.
static void updateStorePending(const String &tag, const String &url) {
  prefs.begin("voicedot", false);
  prefs.putString("upd_tag", tag);
  prefs.putString("upd_url", url);
  prefs.end();
}

static bool updateHasPending() {
  prefs.begin("voicedot", true);
  bool pending = prefs.getString("upd_tag", "").length() > 0;
  prefs.end();
  return pending;
}

// What the last attempt did, so the reason for a failure survives the reboot
// and can be shown in the interface.
static void updateStoreResult(const String &message) {
  prefs.begin("voicedot", false);
  prefs.putString("upd_msg", message);
  prefs.end();
}

static void updateLoadResult() {
  prefs.begin("voicedot", true);
  String message = prefs.getString("upd_msg", "");
  prefs.end();
  if (message.length() > 0) updateStatus = message;
}

static void updateDownloadAndWriteInner(const String &tag, const String &url);

static void updateDownloadAndWrite(const String &tag, const String &url) {
  bool yielded = netMakeRoom(UPDATE_TLS_MIN_HEAP);
  updateDownloadAndWriteInner(tag, url);
  // Only reached when the update did not happen: a written one reboots.
  if (yielded) netReleaseRoom();
}

static void updateDownloadAndWriteInner(const String &tag, const String &url) {
  // Nothing else may touch the speaker, the network or the flash from here on.
  radioStop("Firmware-Update");
  srPauseDetection();
  setAmplifier(false);

  updateInProgress = true;
  updateProgress = 0;
  updateStatus = "laedt " + tag;
  logPrintf("Update: %s wird geladen", tag.c_str());

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  bool ok = http.begin(client, url);
  if (ok) {
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(15000);
    http.setReuse(false);
    http.addHeader("User-Agent", String("VoiceDot/") + FW_VERSION);
  }

  int code = ok ? http.GET() : -1;
  int total = ok ? http.getSize() : -1;

  if (code != HTTP_CODE_OK || total <= 0) {
    updateStatus = "Download fehlgeschlagen (HTTP " + String(code) + ")";
    diagLogf("UPDATE", "download failed code=%d size=%d", code, total);
    http.end();
    updateStoreResult(updateStatus);
    updateInProgress = false;
    updateProgress = -1;
    srResumeDetection();
    return;
  }

  if (!Update.begin((size_t)total)) {
    updateStatus = "Kein Platz fuer das Update";
    diagLogf("UPDATE", "Update.begin failed for %d bytes", total);
    http.end();
    updateStoreResult(updateStatus);
    updateInProgress = false;
    updateProgress = -1;
    srResumeDetection();
    return;
  }

  Client *stream = http.getStreamPtr();
  uint8_t buf[1024];
  size_t written = 0;
  bool checkedMagic = false;
  bool failed = false;
  uint32_t lastData = millis();
  int lastLogged = -1;

  while (written < (size_t)total && !failed) {
    int waiting = stream->available();
    if (waiting <= 0) {
      if ((uint32_t)(millis() - lastData) > UPDATE_STALL_TIMEOUT_MS) {
        updateStatus = "Download abgebrochen (keine Daten mehr)";
        failed = true;
        break;
      }
      pumpServices();
      delay(4);
      continue;
    }

    int got = stream->read(buf, min<size_t>(sizeof(buf), (size_t)waiting));
    if (got <= 0) continue;
    lastData = millis();

    // An ESP32 image starts with 0xE9. Anything else is an error page that
    // arrived with a 200, and writing it would brick the other partition.
    if (!checkedMagic) {
      checkedMagic = true;
      if (buf[0] != 0xE9) {
        updateStatus = "Die Datei ist kein ESP32-Firmware-Image";
        diagLogf("UPDATE", "bad magic 0x%02X", buf[0]);
        failed = true;
        break;
      }
    }

    if (Update.write(buf, (size_t)got) != (size_t)got) {
      updateStatus = "Schreibfehler im Flash";
      failed = true;
      break;
    }

    written += (size_t)got;
    updateProgress = (int)((written * 100ULL) / (size_t)total);

    if (updateProgress / 10 != lastLogged) {
      lastLogged = updateProgress / 10;
      logPrintf("Update: %d %% (%lu von %lu Byte)",
                updateProgress, (unsigned long)written, (unsigned long)total);
    }

    // A ring that fills up, so the progress is visible without a browser.
    if (pixels) {
      uint8_t lit = (uint8_t)((updateProgress * 7) / 100);
      pixels->clear();
      for (uint8_t i = 0; i < 7; i++) {
        if (i <= lit) pixels->setPixelColor(i, pixels->Color(0, 0, 160));
      }
      pixels->show();
    }

    pumpServices();
  }

  http.end();

  if (failed || !Update.end(true)) {
    if (!failed) updateStatus = "Update abgeschlossen, aber ungueltig";
    Update.abort();
    ledsStatusError();
    diagLogf("UPDATE", "failed: %s", updateStatus.c_str());
    updateStoreResult(updateStatus);
    updateInProgress = false;
    updateProgress = -1;
    srResumeDetection();
    return;
  }

  updateStoreResult(tag + " installiert");
  updateStatus = tag + " installiert, Neustart";
  logPrintf("Update: %s installiert, Neustart", tag.c_str());
  setAllLeds(0, 160, 0);
  delay(600);
  ESP.restart();
}

// Runs during boot, before the detector claims its share of the heap. The
// request is cleared before the attempt: a version that cannot be downloaded
// must not turn into a reboot loop.
static void updateRunPending() {
  prefs.begin("voicedot", false);
  String tag = prefs.getString("upd_tag", "");
  String url = prefs.getString("upd_url", "");
  prefs.remove("upd_tag");
  prefs.remove("upd_url");
  prefs.end();

  if (tag.length() == 0 || url.length() == 0) return;

  logPrintf("Update: installiere %s (Heap frei: %lu)",
            tag.c_str(), (unsigned long)ESP.getFreeHeap());
  updateDownloadAndWrite(tag, url);  // reboots when it worked
}

// -----------------------------------------------------------------------------
// Web radio
//
// Unlike a spoken answer, a radio stream never ends, so this cannot be a
// function that runs until it is done. It is a state that gets ticked from the
// main loop: refill the buffer, decode a couple of frames, hand them to I2S,
// come back next time.
//
// The output is resampled to 16 kHz on purpose. TX and RX share one I2S clock,
// and the wake word detector needs exactly 16 kHz - running the music at its
// native 44.1 or 48 kHz would drag the microphone along and deafen the
// detector. This costs treble and buys "Alexa, stopp" while the music plays.
// -----------------------------------------------------------------------------

// Folds a spoken station name and a stored one onto common ground: lower case,
// umlauts spelled out, everything else reduced to single spaces.
static String radioNormalize(const String &in) {
  String lower = in;
  lower.toLowerCase();

  String flat;
  flat.reserve(lower.length() + 4);

  for (size_t i = 0; i < lower.length(); i++) {
    unsigned char c = (unsigned char)lower[i];

    if (c == 0xC3 && i + 1 < lower.length()) {
      unsigned char next = (unsigned char)lower[i + 1];
      i++;
      if (next == 0xA4 || next == 0x84) flat += "ae";
      else if (next == 0xB6 || next == 0x96) flat += "oe";
      else if (next == 0xBC || next == 0x9C) flat += "ue";
      else if (next == 0x9F) flat += "ss";
      else flat += ' ';
      continue;
    }

    if (isalnum(c)) flat += (char)c;
    else flat += ' ';
  }

  String out;
  out.reserve(flat.length());
  bool lastWasSpace = true;
  for (size_t i = 0; i < flat.length(); i++) {
    if (flat[i] == ' ') {
      if (!lastWasSpace) out += ' ';
      lastWasSpace = true;
    } else {
      out += flat[i];
      lastWasSpace = false;
    }
  }
  out.trim();
  return out;
}

static void radioLoadStations() {
  radioStationCount = 0;

  File f = LittleFS.open(RADIO_STATION_FILE, "r");
  if (!f) return;

  while (f.available() && radioStationCount < RADIO_MAX_STATIONS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int tab = line.indexOf('\t');
    if (tab <= 0) continue;

    String name = line.substring(0, tab);
    String url = line.substring(tab + 1);
    name.trim();
    url.trim();
    if (name.length() == 0 || url.length() == 0) continue;

    radioStationNames[radioStationCount] = name;
    radioStationUrls[radioStationCount] = url;
    radioStationCount++;
  }

  f.close();
}

// Written beside the real file and swapped in afterwards. Opening the real one
// for writing truncates it first, and a write that then fails leaves no list at
// all - which is what happened once while a TLS stream had eaten the heap.
static bool radioSaveStations() {
  static const char *tempFile = "/radio.tmp";

  File f = LittleFS.open(tempFile, "w");
  if (!f) return false;

  size_t written = 0;
  for (uint8_t i = 0; i < radioStationCount; i++) {
    written += f.print(radioStationNames[i]);
    written += f.print('\t');
    written += f.print(radioStationUrls[i]);
    written += f.print('\n');
  }

  f.close();

  if (radioStationCount > 0 && written == 0) {
    LittleFS.remove(tempFile);
    diagLog("RADIO", "station list not written, keeping the old one");
    return false;
  }

  LittleFS.remove(RADIO_STATION_FILE);
  if (!LittleFS.rename(tempFile, RADIO_STATION_FILE)) {
    diagLog("RADIO", "station list could not be swapped in");
    return false;
  }
  return true;
}

// Two words count as the same when one contains the other or they share their
// first four letters. Speech recognition writing a German sentence turns
// "Energy" into "Energie", and that must not cost the match.
static bool radioWordsMatch(const String &a, const String &b) {
  if (a == b) return true;
  if (a.length() >= 3 && b.indexOf(a) >= 0) return true;
  if (b.length() >= 3 && a.indexOf(b) >= 0) return true;
  if (a.length() >= 4 && b.length() >= 4 && a.substring(0, 4) == b.substring(0, 4)) {
    return true;
  }
  return false;
}

// Does any word of `spoken` match `word`?
static bool radioAnyWordMatches(const String &spoken, const String &word) {
  String rest = spoken;
  while (rest.length() > 0) {
    int space = rest.indexOf(' ');
    String candidate = space < 0 ? rest : rest.substring(0, space);
    rest = space < 0 ? "" : rest.substring(space + 1);
    if (radioWordsMatch(word, candidate)) return true;
  }
  return false;
}

// Which station the user meant. An exact hit wins, a contained name is next,
// and beyond that the station sharing the most words with what was said - so
// "spiele energy" still finds "ENERGY Wien" when it is the only Energy around.
static int radioFindStation(const String &spokenNorm) {
  if (spokenNorm.length() == 0) return -1;

  int best = -1;
  int bestScore = 0;

  for (uint8_t i = 0; i < radioStationCount; i++) {
    String name = radioNormalize(radioStationNames[i]);
    if (name.length() == 0) continue;

    int score = 0;
    if (spokenNorm == name) {
      score = 10000;
    } else if (spokenNorm.indexOf(name) >= 0) {
      score = 5000 + (int)name.length();
    } else if (name.indexOf(spokenNorm) >= 0) {
      score = 4000 + (int)spokenNorm.length();
    } else {
      // Counted as a fraction of the station's own words rather than as an
      // absolute number: half of "Energy Wien" is a match, half of a name with
      // six words is not.
      uint8_t nameWords = 0;
      uint8_t matched = 0;

      String rest = name;
      while (rest.length() > 0) {
        int space = rest.indexOf(' ');
        String word = space < 0 ? rest : rest.substring(0, space);
        rest = space < 0 ? "" : rest.substring(space + 1);
        if (word.length() < 3) continue;

        nameWords++;
        if (radioAnyWordMatches(spokenNorm, word)) matched++;
      }

      if (nameWords > 0 && matched > 0 && matched * 2 >= nameWords) {
        score = 100 * matched;
      }
    }

    if (score > bestScore) {
      bestScore = score;
      best = i;
    }
  }

  return bestScore >= 100 ? best : -1;
}

static void radioBeginOutput() {
  setAmplifier(true);
  es8311SetMute(false);
  es8311SetVolume(cfg.volume > 0 ? cfg.volume : 60);
  // Deliberately not scaledPlaybackRate(): the speech tempo setting belongs to
  // the assistant's voice, not to music.
  audioSetSampleRate(RADIO_OUTPUT_RATE);
  audioClearTx();
}

static void radioEndOutput() {
  audioClearTx();
  audioSetSampleRate(AUDIO_SAMPLE_RATE);
  es8311SetVolume(cfg.volume);
  if (cfg.volume == 0 || muted) es8311SetMute(true);
  setAmplifier(false);
}

static void radioCloseStream() {
  radioStream = nullptr;

  if (radioHttp) {
    radioHttp->end();
    delete radioHttp;
    radioHttp = nullptr;
  }
  if (radioSecure) {
    delete radioSecure;
    radioSecure = nullptr;
  }
  if (radioPlain) {
    delete radioPlain;
    radioPlain = nullptr;
  }

  radioInLen = 0;
  radioOutFrames = 0;
  radioOutOffset = 0;
}

static void radioFreeBuffers() {
  if (radioInBuf) { free(radioInBuf); radioInBuf = nullptr; }
  if (radioPcm) { free(radioPcm); radioPcm = nullptr; }
  if (radioOut) { free(radioOut); radioOut = nullptr; }
}

static bool radioAllocBuffers() {
  if (radioInBuf && radioPcm && radioOut) return true;

  radioFreeBuffers();
  radioInBuf = (uint8_t*)ps_malloc(RADIO_IN_BUFFER_SIZE);
  radioPcm = (int16_t*)ps_malloc(MP3_PCM_MAX_SAMPLES * sizeof(int16_t));
  radioOut = (int16_t*)ps_malloc(MP3_PCM_MAX_SAMPLES * 2 * sizeof(int16_t));

  if (!radioInBuf || !radioPcm || !radioOut) {
    radioFreeBuffers();
    return false;
  }
  return true;
}

void radioStop(const char *why) {
  if (!radioActive) return;

  radioActive = false;
  radioPausedForTurn = false;
  radioCloseStream();
  MP3Decoder_FreeBuffers();
  radioFreeBuffers();
  radioEndOutput();

  radioStatus = "aus";
  radioStationName = "";
  radioStationUrl = "";
  radioSampleRate = 0;

  if (ledPhase == LED_PHASE_RADIO) setLedPhase(LED_PHASE_IDLE);
  diagLogf("RADIO", "stopped: %s", why ? why : "-");
}

static bool radioConnect(const String &url) {
  radioCloseStream();

  if (WiFi.status() != WL_CONNECTED) return false;

  radioHttp = new HTTPClient();
  if (!radioHttp) return false;

  bool ok = false;
  if (url.startsWith("https://")) {
    radioSecure = new WiFiClientSecure();
    if (radioSecure) {
      radioSecure->setInsecure();
      // Without this the handshake may sit there for two minutes.
      radioSecure->setHandshakeTimeout(10);
      ok = radioHttp->begin(*radioSecure, url);
    }
  } else {
    radioPlain = new WiFiClient();
    if (radioPlain) ok = radioHttp->begin(*radioPlain, url);
  }

  if (!ok) {
    radioStatus = "Verbindung fehlgeschlagen";
    radioCloseStream();
    return false;
  }

  // Same reason as the TTS path: we read the socket ourselves, so the stream
  // must not carry chunked framing. No Icy-MetaData either - the title blocks
  // would land in the middle of the audio and the decoder would choke on them.
  radioHttp->useHTTP10(true);
  radioHttp->setTimeout(8000);
  radioHttp->setReuse(false);
  radioHttp->setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  radioHttp->addHeader("User-Agent", String("VoiceDot/") + FW_VERSION);
  radioHttp->addHeader("Accept", "audio/mpeg,audio/mp3,*/*");

  int code = radioHttp->GET();
  if (code != HTTP_CODE_OK) {
    radioStatus = "HTTP " + String(code);
    diagLogf("RADIO", "GET failed code=%d url=%s", code, url.c_str());
    radioCloseStream();
    return false;
  }

  if (!MP3Decoder_AllocateBuffers()) {
    radioStatus = "Decoder-Speicher fehlt";
    radioCloseStream();
    return false;
  }

  radioStream = radioHttp->getStreamPtr();
  radioInLen = 0;
  radioOutFrames = 0;
  radioOutOffset = 0;
  radioResampRate = 0;
  radioLastDataMs = millis();
  radioStatus = "spielt";

  diagLogf("RADIO", "connected %s (Heap frei: %lu)",
           url.c_str(), (unsigned long)ESP.getFreeHeap());
  return true;
}

// Most Icecast servers answer on plain HTTP as well, and doing without TLS is
// worth a lot here: it is the difference between a responsive device and one
// that needs 25 seconds per page. So an https:// station is tried without TLS
// first, and only falls back when that really does not work.
static bool radioOpenStream() {
  radioUsingTls = false;

  if (radioStationUrl.startsWith("https://")) {
    String plain = "http://" + radioStationUrl.substring(8);
    if (radioConnect(plain)) {
      diagLog("RADIO", "served over plain HTTP, no TLS needed");
      return true;
    }
    diagLog("RADIO", "plain HTTP refused, trying TLS");

    uint32_t freeNow = ESP.getFreeHeap();
    if (freeNow < RADIO_TLS_HEAP_COST + RADIO_TLS_MIN_HEAP) {
      diagLogf("RADIO", "no TLS attempt, %lu bytes free is not enough",
               (unsigned long)freeNow);
      radioStatus = "HTTPS braucht mehr Speicher als frei ist (" +
                    String(freeNow / 1024) + " kB) - bitte einen http-Stream waehlen";
      return false;
    }
  }

  if (!radioConnect(radioStationUrl)) return false;

  radioUsingTls = radioStationUrl.startsWith("https://");
  if (!radioUsingTls) return true;

  // A device whose web interface no longer answers looks broken, and the user
  // has no way of knowing why. Better to say so and play nothing.
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < RADIO_TLS_MIN_HEAP) {
    diagLogf("RADIO", "TLS stream leaves only %lu bytes of heap, refusing",
             (unsigned long)freeHeap);
    radioCloseStream();
    MP3Decoder_FreeBuffers();
    radioStatus = "HTTPS braucht mehr Speicher als frei ist (" +
                  String(freeHeap / 1024) + " kB) - bitte einen http-Stream waehlen";
    radioUsingTls = false;
    return false;
  }

  return true;
}

bool radioStart(const String &name, const String &url) {
  if (url.length() == 0) return false;

  if (!audioI2sReady || !codecPlaybackReady) {
    radioStatus = "Audio nicht bereit";
    return false;
  }

  if (radioActive) radioStop("neuer Sender");

  if (!radioAllocBuffers()) {
    radioStatus = "Puffer-Speicher fehlt";
    diagLog("RADIO", "no memory for the stream buffers");
    return false;
  }

  radioStationName = name.length() > 0 ? name : url;
  radioStationUrl = url;
  radioActive = true;
  radioPausedForTurn = false;
  radioStartedAt = millis();
  radioBytesIn = 0;
  radioUnderruns = 0;
  radioReconnects = 0;
  radioRetryAt = 0;
  radioStatus = "verbindet";

  radioBeginOutput();

  if (!radioOpenStream()) {
    if (radioStatus.startsWith("HTTPS braucht")) {
      String why = radioStatus;
      radioStop("zu wenig Speicher fuer HTTPS");
      radioStatus = why;
      diagLogf("RADIO", "start refused: %s", why.c_str());
      return false;
    }
    // Keep the state and let the tick retry: a station that is busy for a
    // moment should not need a second voice command.
    radioRetryAt = millis() + RADIO_RECONNECT_DELAY_MS;
  }

  setLedPhase(LED_PHASE_RADIO);
  diagLogf("RADIO", "start \"%s\" %s", radioStationName.c_str(), url.c_str());
  return true;
}

// A live stream cannot be paused, so during an assist turn the socket stays
// open and the bytes are thrown away. Catching up afterwards would only build
// a delay that never goes away.
void radioPauseForTurn() {
  if (!radioActive || radioPausedForTurn) return;

  radioPausedForTurn = true;
  radioInLen = 0;
  radioOutFrames = 0;
  radioOutOffset = 0;
  MP3Decoder_FreeBuffers();
  diagLog("RADIO", "paused for the assist turn");
}

void radioResumeAfterTurn() {
  if (!radioActive || !radioPausedForTurn) return;

  radioPausedForTurn = false;
  if (!MP3Decoder_AllocateBuffers()) {
    radioStop("Decoder-Speicher fehlt nach der Runde");
    return;
  }

  if (radioStream) {
    uint8_t sink[512];
    uint8_t guard = 0;
    while (radioStream->available() > 0 && guard++ < 64) {
      radioStream->read(sink, sizeof(sink));
    }
  }

  radioResampRate = 0;
  radioLastDataMs = millis();
  radioBeginOutput();
  setLedPhase(LED_PHASE_RADIO);
  diagLog("RADIO", "resumed");
}

// Box-filter decimation down to 16 kHz. Averaging every input sample that
// falls into an output slot is both the resampler and the anti-alias filter,
// which is as much as this speaker deserves.
static void radioResampleFrame(const int16_t *pcm, int samples, int channels, int rate) {
  if ((uint32_t)rate != radioResampRate) {
    radioResampRate = (uint32_t)rate;
    radioResampStep = (uint32_t)(((uint64_t)rate << 16) / RADIO_OUTPUT_RATE);
    if (radioResampStep == 0) radioResampStep = 65536;
    radioResampPhase = 0;
    radioAccL = 0;
    radioAccR = 0;
    radioAccN = 0;
  }

  int frames = channels == 2 ? samples / 2 : samples;
  size_t out = 0;

  for (int i = 0; i < frames; i++) {
    int32_t l = channels == 2 ? pcm[i * 2] : pcm[i];
    int32_t r = channels == 2 ? pcm[i * 2 + 1] : pcm[i];

    radioAccL += l;
    radioAccR += r;
    radioAccN++;
    radioResampPhase += 65536;

    if (radioResampPhase >= radioResampStep && radioAccN > 0) {
      radioResampPhase -= radioResampStep;
      radioOut[out * 2] = (int16_t)(radioAccL / (int32_t)radioAccN);
      radioOut[out * 2 + 1] = (int16_t)(radioAccR / (int32_t)radioAccN);
      out++;
      radioAccL = 0;
      radioAccR = 0;
      radioAccN = 0;
    }
  }

  radioOutFrames = out;
  radioOutOffset = 0;
}

// Writes what is pending straight to I2S, deliberately with a short timeout:
// a full DMA means we are ahead of real time and the rest can wait for the
// next tick. Not audioWrite(), because a short write is normal here and would
// otherwise log an error every few milliseconds.
static bool radioFlushOutput() {
  while (radioOutOffset < radioOutFrames) {
    size_t remaining = (radioOutFrames - radioOutOffset) * 2 * sizeof(int16_t);
    size_t written = 0;

    esp_err_t err = i2s_channel_write(audioTxChan,
                                      radioOut + radioOutOffset * 2,
                                      remaining,
                                      &written,
                                      RADIO_WRITE_TIMEOUT_MS);

    if (written > 0) radioOutOffset += written / (2 * sizeof(int16_t));
    if (err != ESP_OK || written < remaining) return false;
  }

  radioOutFrames = 0;
  radioOutOffset = 0;
  return true;
}

void radioTick() {
  if (!radioActive) return;

  if (apMode || WiFi.status() != WL_CONNECTED) {
    radioStop("kein WLAN");
    return;
  }

  // The assistant owns the speaker while a turn runs.
  if (wakeBusy || ttsPlaybackActive || speakerTestActive) {
    radioPauseForTurn();
  }

  if (!radioStream) {
    if ((int32_t)(millis() - radioRetryAt) < 0) return;
    if (!radioOpenStream()) {
      // Not enough memory is not going to fix itself in three seconds.
      if (radioStatus.startsWith("HTTPS braucht")) {
        String why = radioStatus;
        radioStop("zu wenig Speicher fuer HTTPS");
        radioStatus = why;
        return;
      }
      radioReconnects++;
      radioRetryAt = millis() + RADIO_RECONNECT_DELAY_MS;
      if (radioReconnects > RADIO_MAX_RECONNECTS) radioStop("Sender nicht erreichbar");
      return;
    }
  }

  int available = radioStream->available();
  while (available > 0 && radioInLen < RADIO_IN_BUFFER_SIZE) {
    size_t room = RADIO_IN_BUFFER_SIZE - radioInLen;
    size_t want = min<size_t>(room, min<size_t>((size_t)available, 4096));
    int got = radioStream->read(radioInBuf + radioInLen, want);
    if (got <= 0) break;
    radioInLen += (size_t)got;
    radioBytesIn += (uint32_t)got;
    radioLastDataMs = millis();
    available = radioStream->available();
  }

  if ((uint32_t)(millis() - radioLastDataMs) > RADIO_STALL_TIMEOUT_MS) {
    diagLogf("RADIO", "stream stalled for %lu ms, reconnecting",
             (unsigned long)(millis() - radioLastDataMs));
    radioCloseStream();
    MP3Decoder_FreeBuffers();
    radioReconnects++;
    radioRetryAt = millis() + RADIO_RECONNECT_DELAY_MS;
    radioStatus = "verbindet neu";
    if (radioReconnects > RADIO_MAX_RECONNECTS) radioStop("Sender antwortet nicht mehr");
    return;
  }

  if (radioPausedForTurn) {
    radioInLen = 0;  // stay live instead of playing the backlog later
    return;
  }

  if (ledPhase == LED_PHASE_IDLE) setLedPhase(LED_PHASE_RADIO);

  if (!radioFlushOutput()) return;

  for (uint8_t frame = 0; frame < RADIO_FRAMES_PER_TICK; frame++) {
    if (radioInLen < 1024) {
      radioUnderruns++;
      return;
    }

    int sync = MP3FindSyncWord(radioInBuf, radioInLen);
    if (sync < 0) {
      // Nothing usable in here; keep the tail in case a header straddles it.
      if (radioInLen > 512) {
        memmove(radioInBuf, radioInBuf + radioInLen - 512, 512);
        radioInLen = 512;
      }
      return;
    }
    if (sync > 0) {
      memmove(radioInBuf, radioInBuf + sync, radioInLen - sync);
      radioInLen -= sync;
    }

    int before = (int)radioInLen;
    int left = before;
    int err = MP3Decode(radioInBuf, &left, radioPcm, 0);
    int consumed = before - left;
    if (consumed > 0 && left >= 0) {
      memmove(radioInBuf, radioInBuf + consumed, left);
      radioInLen = (size_t)left;
    }

    if (err == ERR_MP3_INDATA_UNDERFLOW) {
      radioUnderruns++;
      return;
    }

    if (err != ERR_MP3_NONE && err != ERR_MP3_MAINDATA_UNDERFLOW) {
      if (consumed <= 0 && radioInLen > 1) {
        memmove(radioInBuf, radioInBuf + 1, radioInLen - 1);
        radioInLen--;
      }
      continue;
    }

    int rate = MP3GetSampRate();
    int channels = MP3GetChannels();
    int samples = MP3GetOutputSamps();
    if (rate < 8000 || rate > 48000 || (channels != 1 && channels != 2) ||
        samples <= 0 || samples > (int)MP3_PCM_MAX_SAMPLES) {
      continue;
    }

    if ((uint16_t)rate != radioSampleRate) {
      radioSampleRate = (uint16_t)rate;
      radioChannels = (uint8_t)channels;
      diagLogf("RADIO", "stream %d Hz %d ch -> %lu Hz out",
               rate, channels, (unsigned long)RADIO_OUTPUT_RATE);
    }

    radioResampleFrame(radioPcm, samples, channels, rate);
    if (!radioFlushOutput()) return;
  }
}

// The stored names as something speakable: "A, B oder C".
static String radioStationSentence() {
  String out;
  for (uint8_t i = 0; i < radioStationCount; i++) {
    if (i > 0) out += (i + 1 == radioStationCount) ? " oder " : ", ";
    out += radioStationNames[i];
  }
  return out;
}

// "Spiele ..." always means a station from the local list, so a name we do not
// know is a question to ask back. Handing it to the assistant would only
// produce an apology - it cannot play radio either.
static void radioAskWhichStation(const String &heard) {
  if (radioStationCount == 0) {
    radioAskText = "Es sind noch keine Radiosender gespeichert. "
                   "Im Webinterface kannst du welche hinzufuegen.";
    radioAnswerUntil = 0;
  } else {
    radioAskText = "Den Sender kenne ich nicht. Ich habe " +
                   radioStationSentence() + ". Welchen soll ich spielen?";
    radioAnswerUntil = millis() + RADIO_ANSWER_WINDOW_MS;
  }

  radioAskPending = true;
  wakeLastMessage = "Nachfrage: welcher Sender?";
  diagLogf("RADIO", "unknown station \"%s\", asking back", heard.c_str());
}

static bool radioStartByIndex(int index) {
  if (index < 0 || index >= (int)radioStationCount) return false;

  if (!radioStart(radioStationNames[index], radioStationUrls[index])) {
    wakeLastMessage = "Radio konnte nicht starten: " + radioStatus;
    return true;
  }

  wakeLastMessage = "Radio " + radioStationNames[index] + " gestartet.";
  return true;
}

// "Spiele Energy Wien" and "Stopp" are ours: the station list lives on this
// device, so resolving the name here saves a round trip through the assistant.
static bool radioHandleVoiceCommand(const String &lower) {
  uint8_t words = 1;
  for (size_t i = 0; i < lower.length(); i++) {
    if (lower[i] == ' ') words++;
  }
  if (words > 8) return false;

  String norm = radioNormalize(lower);

  // Answering the question we just asked: the whole utterance is the station
  // name, without a "spiele" in front of it.
  if (radioAnswerUntil != 0 && (int32_t)(millis() - radioAnswerUntil) < 0) {
    radioAnswerUntil = 0;

    if (norm.length() == 0 || norm == "keinen" || norm == "keiner" ||
        norm == "nichts" || norm == "abbrechen" || norm == "egal" ||
        norm == "danke" || norm == "vergiss es" || norm == "lass es") {
      wakeLastMessage = "Radiowahl abgebrochen.";
      return true;
    }

    int chosen = radioFindStation(norm);
    if (chosen >= 0) return radioStartByIndex(chosen);

    // Asking again would only go in circles.
    radioAskText = "Den kenne ich auch nicht.";
    radioAnswerUntil = 0;
    radioAskPending = true;
    wakeLastMessage = "Sender weiterhin unbekannt.";
    return true;
  }

  if (radioActive) {
    if (norm == "stopp" || norm == "stop" || norm == "aus" || norm == "halt" ||
        norm == "ruhe" || norm == "radio aus" || norm == "radio stopp" ||
        norm == "radio stop" || norm == "musik aus" || norm == "musik stopp" ||
        norm == "stopp radio" || norm == "hoer auf" || norm == "sei still") {
      String was = radioStationName;
      radioStop("Sprachbefehl");
      wakeLastMessage = "Radio " + was + " gestoppt.";
      return true;
    }
  }

  // "Spiele" on its own is already a radio command - just without a name yet.
  if (norm == "spiele" || norm == "spiel" || norm == "radio" ||
      norm == "spiele radio" || norm == "starte radio" || norm == "musik") {
    radioAskWhichStation("");
    return true;
  }

  const char *prefixes[] = {"spiele radio ", "spiel radio ", "starte radio ",
                            "spiele ", "spiel ", "spielen sie ", "radio "};
  String wanted = "";
  bool isPlayCommand = false;
  for (uint8_t i = 0; i < sizeof(prefixes) / sizeof(prefixes[0]); i++) {
    String prefix = prefixes[i];
    if (norm.startsWith(prefix)) {
      wanted = norm.substring(prefix.length());
      isPlayCommand = true;
      break;
    }
  }
  if (!isPlayCommand) return false;

  wanted.trim();

  int index = wanted.length() >= 2 ? radioFindStation(wanted) : -1;
  if (index < 0) {
    radioAskWhichStation(wanted);
    return true;
  }

  return radioStartByIndex(index);
}

// -----------------------------------------------------------------------------
// Alarm clock and timer
//
// Both live on the device rather than in Home Assistant: they have to ring even
// when the network is down, and a countdown that depends on a round trip to a
// language model is not a countdown.
// -----------------------------------------------------------------------------

// German speech-to-text writes small numbers either way, so both are accepted.
// Scanned word by word rather than with a substring search: "eine Minute" has
// to count as one, while the "ein" inside "einschalten" must not. Returns -1
// when nothing number-like is found from `start` onwards.
struct SpokenNumber { const char *word; int value; };

static const SpokenNumber SPOKEN_NUMBERS[] = {
  {"null", 0},
  {"ein", 1}, {"eine", 1}, {"einen", 1}, {"einer", 1}, {"eins", 1},
  {"zwei", 2}, {"zwo", 2},
  {"drei", 3}, {"vier", 4}, {"fuenf", 5}, {"sechs", 6}, {"sieben", 7},
  {"acht", 8}, {"neun", 9}, {"zehn", 10}, {"elf", 11}, {"zwoelf", 12},
  {"dreizehn", 13}, {"vierzehn", 14}, {"fuenfzehn", 15}, {"sechzehn", 16},
  {"siebzehn", 17}, {"achtzehn", 18}, {"neunzehn", 19}, {"zwanzig", 20},
  {"dreissig", 30}, {"vierzig", 40}, {"fuenfzig", 50}, {"sechzig", 60},
};

static int spokenNumberAt(const String &text, int start, int *beginsAt, int *endsAt) {
  int i = start;
  while (i < (int)text.length()) {
    while (i < (int)text.length() && text[i] == ' ') i++;
    int wordStart = i;
    while (i < (int)text.length() && text[i] != ' ') i++;
    if (i <= wordStart) break;

    String word = text.substring(wordStart, i);

    bool digits = true;
    for (size_t k = 0; k < word.length(); k++) {
      if (!isdigit((unsigned char)word[k])) { digits = false; break; }
    }
    if (digits) {
      long value = word.toInt();
      if (value >= 0 && value <= 9999) {
        if (beginsAt) *beginsAt = wordStart;
        if (endsAt) *endsAt = i;
        return (int)value;
      }
      continue;
    }

    for (size_t n = 0; n < sizeof(SPOKEN_NUMBERS) / sizeof(SPOKEN_NUMBERS[0]); n++) {
      if (word == SPOKEN_NUMBERS[n].word) {
        if (beginsAt) *beginsAt = wordStart;
        if (endsAt) *endsAt = i;
        return SPOKEN_NUMBERS[n].value;
      }
    }
  }

  return -1;
}

// "7:30", "7 uhr 30", "sieben uhr" - all of them end up as minutes since
// midnight. -1 when the sentence carries no usable time.
static int parseSpokenClock(const String &norm) {
  int hourBegin = 0;
  int hourEnd = 0;
  int hour = spokenNumberAt(norm, 0, &hourBegin, &hourEnd);
  if (hour < 0 || hour > 23) return -1;

  int minute = 0;

  // A second number after the hour is the minute, but only when it follows
  // closely: "7 uhr 30" and "07 30" qualify, "7 uhr ... 30 grad" does not.
  // Five characters cover the " uhr " that sits between them.
  int minuteBegin = 0;
  int minuteEnd = 0;
  int candidate = spokenNumberAt(norm, hourEnd, &minuteBegin, &minuteEnd);
  if (candidate >= 0 && candidate <= 59 && (minuteBegin - hourEnd) <= 5) {
    minute = candidate;
  }

  return hour * 60 + minute;
}

static String clockText(int minutes) {
  if (minutes < 0) return "-";
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d:%02d", minutes / 60, minutes % 60);
  return String(buf);
}

// localMinutesNow() belongs to the day/night schedule and is reused here.
static int localDayNow() {
  struct tm t;
  if (!getLocalTime(&t, 5)) return -1;
  return t.tm_yday;
}

// -------------------------------------------------------------------- alarm

static void alarmClear(const char *why) {
  cfg.alarmMinutes = -1;
  alarmFiredDay = -1;
  saveConfig();
  diagLogf("ALARM", "cleared: %s", why ? why : "-");
}

static void alarmSet(int minutes) {
  cfg.alarmMinutes = constrain(minutes, 0, 1439);
  alarmFiredDay = -1;

  // Setting the alarm to a time that has already passed today means tomorrow,
  // and that falls out of the fired-day bookkeeping on its own.
  saveConfig();
  logPrintf("Wecker gestellt auf %s", clockText(cfg.alarmMinutes).c_str());
}

// Whole minutes until it rings, for the interface and for Home Assistant.
static long alarmSecondsUntil() {
  if (cfg.alarmMinutes < 0) return -1;
  int now = localMinutesNow();
  if (now < 0) return -1;

  int delta = cfg.alarmMinutes - now;
  if (delta <= 0) delta += 24 * 60;
  return (long)delta * 60;
}

// The briefing is an instruction for the assistant, not a sentence to read out.
// It goes through the configured pipeline starting at the intent stage, so the
// agent can look at the weather, the calendars and what happened overnight -
// and the answer comes back already turned into speech by the same voice the
// assistant otherwise uses.
static bool alarmSpeakBriefing(const String &instruction) {
  if (instruction.length() == 0) return false;

  String host;
  uint16_t port = 0;
  bool secure = false;
  if (WiFi.status() != WL_CONNECTED || cfg.haUrl.isEmpty() || cfg.haToken.isEmpty() ||
      !parseHaUrl(cfg.haUrl, host, port, secure)) {
    diagLog("BRIEFING", "Home Assistant nicht erreichbar");
    return false;
  }

  WiFiClient plain;
  WiFiClientSecure tls;
  Client *client = nullptr;
  if (secure) { tls.setInsecure(); client = &tls; } else { client = &plain; }

  if (!wsConnectHa(*client, host, port, cfg.haToken)) {
    client->stop();
    diagLog("BRIEFING", "WS/Auth fehlgeschlagen");
    return false;
  }

  String run = "{\"id\":1,\"type\":\"assist_pipeline/run\"";
  run += ",\"start_stage\":\"intent\",\"end_stage\":\"tts\"";
  run += ",\"input\":{\"text\":\"" + jsonEscape(instruction) + "\"}";
  run += ",\"timeout\":60";
  if (cfg.haPipeline.length() > 0) {
    run += ",\"pipeline\":\"" + jsonEscape(cfg.haPipeline) + "\"";
  }
  run += "}";

  wsSendText(*client, run);
  diagLogf("BRIEFING", "Anweisung gesendet (%u Zeichen)", instruction.length());

  String msg;
  uint8_t opcode = 0;
  String spoken = "";
  String ttsUrl = "";
  uint32_t start = millis();

  // A briefing that asks for weather and three calendar entries takes the agent
  // a while; a minute is generous but still bounded.
  while ((uint32_t)(millis() - start) < 60000) {
    if (!wsReadFrame(*client, msg, opcode, 60000)) {
      diagLogf("BRIEFING", "Verbindung beendet nach %lu ms - meist ungueltiges UTF-8",
               (unsigned long)(millis() - start));
      break;
    }
    if (opcode == 0x8) {
      diagLog("BRIEFING", "Home Assistant hat die Verbindung geschlossen");
      break;
    }

    // The token-by-token deltas would bury everything else; the events that
    // decide whether this worked are still logged in full.
    if (msg.indexOf("\"intent-progress\"") < 0) {
      diagLogf("BRIEFING", "WS: %s", msg.substring(0, 200).c_str());
    }

    String type = assistEventType(msg);
    if (type == "intent-end") {
      String speech = jsonFindLastString(msg, "speech");
      if (speech.length() > 0) spoken = speech;
    } else if (type == "tts-end") {
      String path = jsonFindString(msg, "path");
      if (path.length() > 0) ttsUrl = path;
      else ttsUrl = jsonFindString(msg, "url");
    } else if (type == "error") {
      diagLogf("BRIEFING", "Fehler von Home Assistant: %s", msg.substring(0, 160).c_str());
      break;
    } else if (type == "run-end") {
      break;
    }

    ledTick();
    pumpServices();
  }

  client->stop();

  if (spoken.length() > 0) {
    wakeAssistantText = spoken;
    logPrintf("Briefing: %s", spoken.substring(0, 120).c_str());
  }

  if (ttsUrl.length() == 0) {
    diagLog("BRIEFING", "keine Sprachausgabe erhalten");
    // Better a spoken answer without the assistant's voice than silence.
    if (spoken.length() > 0) return playAnnouncement(spoken);
    return false;
  }

  return fetchAndPlayTtsUrl(ttsUrl);
}

static void alarmFire() {
  alarmFiredDay = localDayNow();
  if (!cfg.alarmDaily) cfg.alarmMinutes = -1;
  saveConfig();

  logPrintf("Wecker klingelt (%s)", clockText(cfg.alarmMinutes).c_str());
  alarmRinging = true;

  if (cfg.alarmSound.length() > 0) playSoundFile(cfg.alarmSound);

  // Spoken after the sound, so the sound stays the thing that wakes you and the
  // words are for once you are awake.
  if (cfg.alarmBriefing.length() > 0) alarmSpeakBriefing(cfg.alarmBriefing);

  alarmRinging = false;
}

// -------------------------------------------------------------------- timer

static void timerClear(const char *why) {
  timerEndsAt = 0;
  timerTotalSec = 0;
  diagLogf("TIMER", "cleared: %s", why ? why : "-");
}

static void timerSet(uint32_t seconds) {
  if (seconds == 0) { timerClear("auf null gesetzt"); return; }

  timerTotalSec = seconds;
  timerEndsAt = time(nullptr) + (time_t)seconds;
  logPrintf("Timer gestellt auf %lu Sekunden", (unsigned long)seconds);
}

static long timerSecondsLeft() {
  if (timerEndsAt == 0) return -1;
  long left = (long)(timerEndsAt - time(nullptr));
  return left > 0 ? left : 0;
}

// "noch 2 Minuten und 30 Sekunden" reads better than "150 Sekunden".
static String timerSpokenRemaining() {
  long left = timerSecondsLeft();
  if (left < 0) return "Es läuft gerade kein Timer.";
  if (left == 0) return "Der Timer ist gerade abgelaufen.";

  long minutes = left / 60;
  long seconds = left % 60;

  if (minutes == 0) return "Noch " + String(seconds) + " Sekunden.";
  if (seconds == 0) {
    return "Noch " + String(minutes) + (minutes == 1 ? " Minute." : " Minuten.");
  }
  return "Noch " + String(minutes) + (minutes == 1 ? " Minute und " : " Minuten und ") +
         String(seconds) + " Sekunden.";
}

static void timerFire() {
  timerClear("abgelaufen");
  logPrintf("Timer abgelaufen");

  if (cfg.timerSound.length() > 0) playSoundFile(cfg.timerSound);
  else playAnnouncement("Der Timer ist abgelaufen.");
}

// Called from the loop. Both are checked at minute resolution for the alarm and
// at second resolution for the timer, which is as precise as either needs.
void alarmTimerTick() {
  if (wakeBusy || ttsPlaybackActive || speakerTestActive || updateInProgress) return;

  if (timerEndsAt != 0 && time(nullptr) >= timerEndsAt) {
    timerFire();
    return;
  }

  if (cfg.alarmMinutes < 0 || !timeValid) return;

  int now = localMinutesNow();
  int day = localDayNow();
  if (now < 0 || day < 0) return;

  if (now == cfg.alarmMinutes && day != alarmFiredDay) alarmFire();
}

// --------------------------------------------------------- the voice commands

static bool alarmTimerVoiceCommand(const String &lower) {
  String norm = radioNormalize(lower);
  if (norm.length() == 0) return false;

  bool mentionsAlarm = norm.indexOf("wecker") >= 0;
  bool mentionsTimer = norm.indexOf("timer") >= 0;

  // A question about the running timer, in the forms people actually use.
  if (timerEndsAt != 0 &&
      (norm.indexOf("wie lange") >= 0 || norm.indexOf("wie viel zeit") >= 0 ||
       (mentionsTimer && (norm.indexOf("noch") >= 0 || norm.indexOf("rest") >= 0 ||
                          norm.indexOf("status") >= 0)))) {
    wakeAnnounceText = timerSpokenRemaining();
    wakeLastMessage = wakeAnnounceText;
    return true;
  }

  if (!mentionsAlarm && !mentionsTimer) return false;

  bool cancels = norm.indexOf("loesch") >= 0 || norm.indexOf("aus") >= 0 ||
                 norm.indexOf("abbrech") >= 0 || norm.indexOf("stopp") >= 0 ||
                 norm.indexOf("stop") >= 0 || norm.indexOf("entfern") >= 0;

  if (mentionsTimer && cancels) {
    timerClear("Sprachbefehl");
    wakeAnnounceText = "Timer gelöscht.";
    wakeLastMessage = wakeAnnounceText;
    return true;
  }

  if (mentionsAlarm && cancels) {
    alarmClear("Sprachbefehl");
    wakeAnnounceText = "Wecker gelöscht.";
    wakeLastMessage = wakeAnnounceText;
    return true;
  }

  // "wann klingelt der wecker"
  if (mentionsAlarm && (norm.indexOf("wann") >= 0 || norm.indexOf("steht") >= 0 ||
                        norm.indexOf("status") >= 0)) {
    if (cfg.alarmMinutes < 0) wakeAnnounceText = "Es ist kein Wecker gestellt.";
    else wakeAnnounceText = "Der Wecker klingelt um " + clockText(cfg.alarmMinutes) + " Uhr.";
    wakeLastMessage = wakeAnnounceText;
    return true;
  }

  if (mentionsTimer) {
    int begin = 0;
    int end = 0;
    int value = spokenNumberAt(norm, 0, &begin, &end);
    if (value <= 0) return false;

    // The unit decides the scale; minutes are the default because that is what
    // people say without thinking about it.
    uint32_t seconds = (uint32_t)value * 60;
    String tail = norm.substring(end);
    if (tail.indexOf("sekund") >= 0) seconds = (uint32_t)value;
    else if (tail.indexOf("stund") >= 0) seconds = (uint32_t)value * 3600;

    if (seconds < 5 || seconds > 12UL * 3600UL) return false;

    timerSet(seconds);
    wakeAnnounceText = "Timer läuft.";
    wakeLastMessage = "Timer auf " + String(seconds) + " Sekunden gestellt.";
    return true;
  }

  // Everything left mentions the alarm and carries a time.
  int minutes = parseSpokenClock(norm);
  if (minutes < 0) return false;

  alarmSet(minutes);
  wakeAnnounceText = "Wecker gestellt auf " + clockText(minutes) + " Uhr.";
  wakeLastMessage = wakeAnnounceText;
  return true;
}

// -----------------------------------------------------------------------------
// Group control
//
// "Schalte im Obergeschoss das Licht aus" names a group and an action. Both are
// resolved here and turned into one Home Assistant service call, because
// sending it through a language model means waiting seconds for a decision that
// is already made - and getting a different set of entities each time.
// -----------------------------------------------------------------------------

static void groupsLoad() {
  groupCount = 0;

  File f = LittleFS.open(GROUP_FILE, "r");
  if (!f) return;

  while (f.available() && groupCount < GROUP_MAX) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int tab = line.indexOf('\t');
    if (tab <= 0) continue;

    String name = line.substring(0, tab);
    String entities = line.substring(tab + 1);
    name.trim();
    entities.trim();
    if (name.length() == 0 || entities.length() == 0) continue;

    groupNames[groupCount] = name;
    groupEntities[groupCount] = entities;
    groupCount++;
  }

  f.close();
}

// Written beside the real file and swapped in, for the same reason the station
// list is: a failed write must not leave an empty one.
static bool groupsSave() {
  static const char *tempFile = "/groups.tmp";

  File f = LittleFS.open(tempFile, "w");
  if (!f) return false;

  size_t written = 0;
  for (uint8_t i = 0; i < groupCount; i++) {
    written += f.print(groupNames[i]);
    written += f.print('\t');
    written += f.print(groupEntities[i]);
    written += f.print('\n');
  }
  f.close();

  if (groupCount > 0 && written == 0) {
    LittleFS.remove(tempFile);
    return false;
  }

  LittleFS.remove(GROUP_FILE);
  return LittleFS.rename(tempFile, GROUP_FILE);
}

// One service call for the whole group. homeassistant.turn_on/off works across
// domains, which is what makes a mixed group of lights and switches behave the
// way the sentence implies.
static bool groupCallService(const String &domain, const String &service,
                             const String &entities) {
  if (cfg.haUrl.length() == 0 || cfg.haToken.length() == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = normalizedHaUrl(cfg.haUrl) + "/api/services/" + domain + "/" + service;

  HTTPClient http;
  WiFiClientSecure secureClient;
  bool ok;
  if (url.startsWith("https://")) {
    secureClient.setInsecure();
    ok = http.begin(secureClient, url);
  } else {
    ok = http.begin(url);
  }
  if (!ok) return false;

  http.setTimeout(8000);
  http.setReuse(false);
  http.addHeader("Authorization", "Bearer " + cfg.haToken);
  http.addHeader("Content-Type", "application/json");

  String body = "{\"entity_id\":[";
  String rest = entities;
  bool first = true;
  while (rest.length() > 0) {
    int comma = rest.indexOf(',');
    String one = comma < 0 ? rest : rest.substring(0, comma);
    rest = comma < 0 ? "" : rest.substring(comma + 1);
    one.trim();
    if (one.length() == 0) continue;
    if (!first) body += ",";
    first = false;
    body += "\"" + jsonEscape(one) + "\"";
  }
  body += "]}";

  int code = http.POST(body);
  http.end();

  diagLogf("GROUP", "%s.%s -> HTTP %d", domain.c_str(), service.c_str(), code);
  return code >= 200 && code < 300;
}

// How many of the group's own words appear in the sentence. Judged as a
// fraction, so a two-word group needs both and a longer one is not punished
// for the filler words around it.
static int groupMatchScore(const String &groupName, const String &spoken) {
  String name = radioNormalize(groupName);
  if (name.length() == 0) return 0;

  uint8_t total = 0;
  uint8_t matched = 0;
  String rest = name;
  while (rest.length() > 0) {
    int space = rest.indexOf(' ');
    String word = space < 0 ? rest : rest.substring(0, space);
    rest = space < 0 ? "" : rest.substring(space + 1);
    if (word.length() < 3) continue;

    total++;
    if (radioAnyWordMatches(spoken, word)) matched++;
  }

  if (total == 0 || matched == 0) return 0;
  if (matched * 2 < total) return 0;  // half the name has to be there
  return matched * 100 + (matched == total ? 50 : 0);
}

// Short action words have to match whole words: "aus" appears inside
// "Aussenbeleuchtung", and switching that off when asked to switch it on would
// be the worst kind of bug - confidently wrong.
static bool normHasWord(const String &norm, const char *word) {
  String padded = " " + norm + " ";
  return padded.indexOf(" " + String(word) + " ") >= 0;
}

static bool groupsVoiceCommand(const String &lower) {
  if (groupCount == 0) return false;

  String norm = radioNormalize(lower);
  if (norm.length() == 0) return false;

  // The action first: without one this is not a group command at all, and
  // checking it first keeps questions about a room from switching anything.
  String domain = "homeassistant";
  String service = "";

  if (norm.indexOf("oeffn") >= 0 || norm.indexOf("aufmach") >= 0 ||
      norm.indexOf("hochfahr") >= 0 || normHasWord(norm, "auf") ||
      normHasWord(norm, "hoch")) {
    domain = "cover"; service = "open_cover";
  } else if (norm.indexOf("schliess") >= 0 || norm.indexOf("zumach") >= 0 ||
             norm.indexOf("runterfahr") >= 0 || norm.indexOf("herunterfahr") >= 0 ||
             normHasWord(norm, "zu") || normHasWord(norm, "runter")) {
    domain = "cover"; service = "close_cover";
  } else if (norm.indexOf("ausschalt") >= 0 || norm.indexOf("ausmach") >= 0 ||
             normHasWord(norm, "aus")) {
    service = "turn_off";
  } else if (norm.indexOf("einschalt") >= 0 || norm.indexOf("anschalt") >= 0 ||
             norm.indexOf("anmach") >= 0 ||
             normHasWord(norm, "an") || normHasWord(norm, "ein")) {
    service = "turn_on";
  }

  if (service.length() == 0) return false;

  int best = -1;
  int bestScore = 0;
  for (uint8_t i = 0; i < groupCount; i++) {
    if (groupEntities[i].length() == 0) continue;  // created but not filled yet
    int score = groupMatchScore(groupNames[i], norm);
    if (score > bestScore) {
      bestScore = score;
      best = i;
    }
  }

  if (best < 0) return false;

  bool ok = groupCallService(domain, service, groupEntities[best]);
  const char *what = service == "turn_on"    ? "eingeschaltet"
                   : service == "turn_off"   ? "ausgeschaltet"
                   : service == "open_cover" ? "geoeffnet"
                                             : "geschlossen";

  if (ok) {
    wakeLastMessage = groupNames[best] + " " + what + ".";
    diagLogf("GROUP", "\"%s\" %s", groupNames[best].c_str(), what);
  } else {
    wakeAnnounceText = "Das hat nicht geklappt.";
    wakeLastMessage = "Gruppe " + groupNames[best] + ": Aufruf fehlgeschlagen.";
  }
  return true;
}

// -----------------------------------------------------------------------------
// Sound library
// -----------------------------------------------------------------------------

// Keeps a plain file name: no paths, no surprises, and an audio extension.
static String soundSanitizeName(const String &raw) {
  String name = raw;

  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  slash = name.lastIndexOf('\\');
  if (slash >= 0) name = name.substring(slash + 1);
  name.trim();

  String out;
  for (size_t i = 0; i < name.length() && out.length() < 40; i++) {
    char c = name[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-') {
      out += c;
    }
  }

  String lower = out;
  lower.toLowerCase();
  if (!lower.endsWith(".mp3") && !lower.endsWith(".wav")) return "";
  if (out.length() < 5) return "";

  return out;
}

static String soundPath(const String &name) {
  return String(SOUND_DIR) + "/" + name;
}

static uint8_t soundCount() {
  if (!ackFsReady) return 0;

  uint8_t n = 0;
  File dir = LittleFS.open(SOUND_DIR);
  if (!dir || !dir.isDirectory()) return 0;

  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) n++;
    entry = dir.openNextFile();
  }
  return n;
}

// Plays one uploaded file. MP3 and WAV both work, the decoder is picked by
// extension just like for the acknowledgement clips.
bool playSoundFile(const String &name) {
  if (!ackFsReady) return false;

  String clean = soundSanitizeName(name);
  if (clean.isEmpty()) {
    diagLogf("SOUND", "invalid name \"%s\"", name.c_str());
    return false;
  }

  String path = soundPath(clean);
  if (!LittleFS.exists(path)) {
    diagLogf("SOUND", "not found: %s", path.c_str());
    return false;
  }

  String lower = clean;
  lower.toLowerCase();

  bool ok = false;
  if (lower.endsWith(".wav")) {
    File f = LittleFS.open(path, "r");
    if (f) {
      ttsPlaybackActive = true;
      ok = playWavPcm16(f);
      f.close();
    }
  } else {
    ok = playMp3File(path);
  }

  diagLogf("SOUND", "played %s ok=%u", clean.c_str(), ok ? 1 : 0);
  return ok;
}

// Speaks a text handed in over HTTP. Accepts a form field, a query parameter
// or a JSON body, so it is easy to call from a Home Assistant rest_command.
static String minutesToHhMm(uint16_t minutes) {
  char buf[6];
  snprintf(buf, sizeof(buf), "%02u:%02u", minutes / 60U, minutes % 60U);
  return String(buf);
}

static uint16_t hhMmToMinutes(const String &value, uint16_t fallback) {
  int colon = value.indexOf(':');
  if (colon < 1) return fallback;

  int h = value.substring(0, colon).toInt();
  int m = value.substring(colon + 1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return fallback;
  return (uint16_t)(h * 60 + m);
}

void handleAnnounce() {
  String text = server.hasArg("text") ? server.arg("text") : String("");

  if (text.isEmpty() && server.hasArg("plain")) {
    text = jsonFindString(server.arg("plain"), "text");
    if (text.isEmpty()) text = jsonFindString(server.arg("plain"), "message");
  }

  text.trim();
  if (text.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8",
                "Kein Text. Erwartet wird ?text=... , text=... im Formular "
                "oder {\"text\":\"...\"} als JSON.");
    return;
  }

  if (announceRequested) {
    server.send(409, "text/plain; charset=utf-8", "Es wartet bereits eine Ansage.");
    return;
  }

  if (haTtsEngine.isEmpty()) {
    server.send(409, "text/plain; charset=utf-8",
                "Die TTS-Engine ist noch unbekannt. Bitte einmal eine Frage stellen.");
    return;
  }

  announceText = text;
  announceRequested = true;
  announceStatus = "eingereiht: " + text;
  server.send(200, "text/plain; charset=utf-8", "Ansage eingereiht.");
}

// Lets Home Assistant set the volume without going through speech.
void handleVolume() {
  int percent = -1;

  if (server.hasArg("percent")) percent = server.arg("percent").toInt();
  else if (server.hasArg("step")) percent = server.arg("step").toInt() * 10;
  else if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int step = jsonFindInt(body, "step", -1);
    int pct = jsonFindInt(body, "percent", -1);
    percent = pct >= 0 ? pct : (step >= 0 ? step * 10 : -1);
  }

  if (percent < 0) {
    server.send(400, "text/plain; charset=utf-8",
                "Erwartet wird percent=0..100 oder step=0..10.");
    return;
  }

  setVolume((uint8_t)constrain(percent, 0, 100));
  haPublishAt = 0;  // report the change straight away
  server.send(200, "text/plain; charset=utf-8",
              "Lautstärke " + String(cfg.volume) + " %");
}

// Multipart upload of one sound file.
void handleSoundUploadData() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    soundUploadBytes = 0;
    soundUploadFailed = false;
    soundUploadName = soundSanitizeName(upload.filename);

    if (!ackFsReady) {
      soundUploadFailed = true;
      soundUploadStatus = "LittleFS ist nicht bereit.";
      return;
    }
    if (soundUploadName.isEmpty()) {
      soundUploadFailed = true;
      soundUploadStatus = "Nur .mp3 und .wav, Name ohne Sonderzeichen.";
      return;
    }
    if (!LittleFS.exists(soundPath(soundUploadName)) && soundCount() >= SOUND_MAX_FILES) {
      soundUploadFailed = true;
      soundUploadStatus = "Maximal " + String(SOUND_MAX_FILES) + " Dateien.";
      return;
    }

    LittleFS.mkdir(SOUND_DIR);
    soundUploadFile = LittleFS.open(soundPath(soundUploadName), "w");
    if (!soundUploadFile) {
      soundUploadFailed = true;
      soundUploadStatus = "Datei konnte nicht angelegt werden.";
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (soundUploadFailed || !soundUploadFile) return;

    if (soundUploadBytes + upload.currentSize > SOUND_MAX_BYTES) {
      soundUploadFailed = true;
      soundUploadStatus = "Datei ist größer als " + String(SOUND_MAX_BYTES / 1024) + " kB.";
      soundUploadFile.close();
      LittleFS.remove(soundPath(soundUploadName));
      return;
    }

    soundUploadFile.write(upload.buf, upload.currentSize);
    soundUploadBytes += upload.currentSize;
  } else if (upload.status == UPLOAD_FILE_END) {
    if (soundUploadFile) soundUploadFile.close();
    if (!soundUploadFailed) {
      soundUploadStatus = soundUploadName + " gespeichert (" +
                          String(soundUploadBytes / 1024) + " kB)";
      diagLogf("SOUND", "uploaded %s (%lu bytes)",
               soundUploadName.c_str(), (unsigned long)soundUploadBytes);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (soundUploadFile) soundUploadFile.close();
    LittleFS.remove(soundPath(soundUploadName));
    soundUploadFailed = true;
    soundUploadStatus = "Upload abgebrochen.";
  }
}

void handleSoundUploadDone() {
  server.send(soundUploadFailed ? 400 : 200, "text/plain; charset=utf-8", soundUploadStatus);
}

void handleSoundList() {
  String json = "{\"max_files\":" + String(SOUND_MAX_FILES) +
                ",\"max_bytes\":" + String(SOUND_MAX_BYTES) +
                ",\"status\":\"" + jsonEscape(soundUploadStatus) + "\",\"files\":[";

  if (ackFsReady) {
    File dir = LittleFS.open(SOUND_DIR);
    if (dir && dir.isDirectory()) {
      bool first = true;
      File entry = dir.openNextFile();
      while (entry) {
        if (!entry.isDirectory()) {
          String name = entry.name();
          int slash = name.lastIndexOf('/');
          if (slash >= 0) name = name.substring(slash + 1);

          if (!first) json += ",";
          first = false;
          json += "{\"name\":\"" + jsonEscape(name) + "\",\"size\":" + String(entry.size()) + "}";
        }
        entry = dir.openNextFile();
      }
    }
  }

  json += "]}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleSoundDelete() {
  String name = soundSanitizeName(server.hasArg("name") ? server.arg("name") : String(""));
  if (name.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "Kein gültiger Dateiname.");
    return;
  }

  if (!LittleFS.exists(soundPath(name))) {
    server.send(404, "text/plain; charset=utf-8", name + " gibt es nicht.");
    return;
  }

  LittleFS.remove(soundPath(name));
  soundUploadStatus = name + " gelöscht";
  diagLogf("SOUND", "deleted %s", name.c_str());
  server.send(200, "text/plain; charset=utf-8", soundUploadStatus);
}

void handleSoundPlay() {
  String name = soundSanitizeName(server.hasArg("name") ? server.arg("name") : String(""));
  if (name.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "Kein gültiger Dateiname.");
    return;
  }

  if (wakeBusy || ttsPlaybackActive || speakerTestActive) {
    server.send(409, "text/plain; charset=utf-8", "Gerade beschäftigt.");
    return;
  }

  soundPlayRequest = name;
  server.send(200, "text/plain; charset=utf-8", name + " wird abgespielt.");
}

void handlePipelineRefresh() {
  pipelineListRequested = true;
  pipelineListStatus = "wird abgerufen ...";
  server.send(200, "text/plain; charset=utf-8", "Pipelines werden abgerufen.");
}

// Renders the acknowledgement clips. The work happens in loop().
void handleAckBuild() {
  if (!ackFsReady) {
    server.send(500, "text/plain; charset=utf-8", "LittleFS ist nicht bereit.");
    return;
  }
  if (haTtsEngine.isEmpty()) {
    server.send(409, "text/plain; charset=utf-8",
                "Die TTS-Engine ist noch unbekannt. Bitte einmal eine Frage stellen, "
                "danach kennt VoiceDot die Stimme.");
    return;
  }

  ackBuildRequested = true;
  ackBuildStatus = "eingereiht ...";
  server.send(200, "text/plain; charset=utf-8",
              "Ansagen werden erzeugt. Das dauert ein paar Sekunden.");
}

// Plays one clip so the user can hear what they configured.
void handleAckTest() {
  if (wakeBusy || ttsPlaybackActive || speakerTestActive) {
    server.send(409, "text/plain; charset=utf-8", "Gerade beschäftigt.");
    return;
  }

  ackTestRequested = true;
  server.send(200, "text/plain; charset=utf-8", "Ansage wird abgespielt.");
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

  HTTPClient http;
  WiFiClientSecure secureClient;
  bool ok = false;

  if (url.startsWith("https://")) {
    // Self-signed certificates are the norm for a local HA instance.
    secureClient.setInsecure();
    ok = http.begin(secureClient, url + "/api/");
  } else {
    ok = http.begin(url + "/api/");
  }

  if (!ok) {
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

void handleUpdateCheck() {
  if (updateInProgress) {
    server.send(409, "text/plain; charset=utf-8", "Es laeuft bereits ein Update.");
    return;
  }
  if (!updateFetchReleases()) {
    server.send(502, "text/plain; charset=utf-8", updateStatus);
    return;
  }
  server.send(200, "text/plain; charset=utf-8",
              String(updateReleaseCount) + " Releases gefunden: " + updateStatus);
}

void handleUpdateInstall() {
  if (updateInProgress) {
    server.send(409, "text/plain; charset=utf-8", "Es laeuft bereits ein Update.");
    return;
  }

  String tag = server.arg("tag");
  if (tag.length() == 0) {
    int newest = updateNewestIndex();
    if (newest < 0) {
      server.send(404, "text/plain; charset=utf-8",
                  "Kein Release mit Firmware bekannt. Erst nach Updates suchen.");
      return;
    }
    tag = updateTags[newest];
  }

  int index = -1;
  for (uint8_t i = 0; i < updateReleaseCount; i++) {
    if (updateTags[i] == tag) { index = i; break; }
  }
  if (index < 0) {
    server.send(404, "text/plain; charset=utf-8", "Version " + tag + " ist nicht bekannt.");
    return;
  }
  if (updateUrls[index].length() == 0) {
    server.send(404, "text/plain; charset=utf-8",
                "Release " + tag + " enthaelt keine .bin-Datei.");
    return;
  }

  // Answer first, then reboot into the install. Doing it here would run the TLS
  // handshake against a heap the detector has already spoken for - measured, it
  // left 380 bytes and failed.
  updateStorePending(tag, updateUrls[index]);
  updateRebootAt = millis() + 400;
  updateStatus = "Neustart zum Installieren von " + tag;
  server.send(200, "text/plain; charset=utf-8",
              tag + " wird installiert. Das Geraet startet dafuer jetzt neu und "
              "meldet sich in etwa einer Minute zurueck.");
}

// Home Assistant's state list runs to hundreds of kilobytes on a busy
// instance, so it is scanned out of the socket and only matches are kept -
// the same reason the release list is not buffered either.
void handleHaEntities() {
  String query = server.arg("q");
  query.toLowerCase();
  query.trim();

  if (cfg.haUrl.length() == 0 || cfg.haToken.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "Home Assistant ist nicht eingerichtet.");
    return;
  }

  HTTPClient http;
  WiFiClientSecure secureClient;
  String url = normalizedHaUrl(cfg.haUrl) + "/api/states";
  bool ok;
  if (url.startsWith("https://")) {
    secureClient.setInsecure();
    ok = http.begin(secureClient, url);
  } else {
    ok = http.begin(url);
  }
  if (!ok) {
    server.send(502, "text/plain; charset=utf-8", "Verbindung zu Home Assistant fehlgeschlagen.");
    return;
  }

  http.useHTTP10(true);
  http.setTimeout(12000);
  http.setReuse(false);
  http.addHeader("Authorization", "Bearer " + cfg.haToken);
  http.addHeader("Accept", "application/json");

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    server.send(502, "text/plain; charset=utf-8", "Home Assistant antwortet mit HTTP " + String(code));
    return;
  }

  static const char *KEY_ID = "\"entity_id\":\"";
  static const char *KEY_NAME = "\"friendly_name\":\"";

  Client *stream = http.getStreamPtr();
  uint8_t idPos = 0;
  uint8_t namePos = 0;
  uint8_t capture = 0;      // 1 = entity_id, 2 = friendly_name
  bool escaped = false;
  String value = "";
  String pendingId = "";
  uint16_t found = 0;
  uint32_t deadline = millis() + 15000;

  String json = "{\"entities\":[";

  while ((http.connected() || stream->available() > 0) &&
         (int32_t)(millis() - deadline) < 0 && found < HA_ENTITY_LIMIT) {
    int waiting = stream->available();
    if (waiting <= 0) { pumpServices(); delay(2); continue; }

    while (waiting-- > 0 && found < HA_ENTITY_LIMIT) {
      int raw = stream->read();
      if (raw < 0) break;
      char c = (char)raw;

      if (capture != 0) {
        if (escaped) { value += c; escaped = false; continue; }
        if (c == '\\') { escaped = true; continue; }
        if (c != '"') {
          value += c;
          if (value.length() > 120) { capture = 0; value = ""; }
          continue;
        }

        if (capture == 1) {
          pendingId = value;
        } else if (pendingId.length() > 0) {
          // An entity is a hit when the search text appears in either half.
          String haystack = pendingId + " " + value;
          haystack.toLowerCase();
          if (query.length() == 0 || haystack.indexOf(query) >= 0) {
            if (found > 0) json += ",";
            json += "{\"id\":\"" + jsonEscape(pendingId) + "\",";
            json += "\"name\":\"" + jsonEscape(value) + "\"}";
            found++;
          }
          pendingId = "";
        }

        capture = 0;
        value = "";
        continue;
      }

      idPos = (c == KEY_ID[idPos]) ? idPos + 1 : (c == KEY_ID[0] ? 1 : 0);
      namePos = (c == KEY_NAME[namePos]) ? namePos + 1 : (c == KEY_NAME[0] ? 1 : 0);

      if (KEY_ID[idPos] == '\0') { capture = 1; idPos = 0; namePos = 0; value = ""; }
      else if (KEY_NAME[namePos] == '\0') { capture = 2; idPos = 0; namePos = 0; value = ""; }
    }

    pumpServices();
  }

  http.end();
  json += "],\"count\":" + String(found);
  json += ",\"truncated\":" + String(found >= HA_ENTITY_LIMIT ? "true" : "false") + "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleGroups() {
  if (server.args() == 0) {
    String json = "{\"groups\":[";
    for (uint8_t i = 0; i < groupCount; i++) {
      if (i > 0) json += ",";
      json += "{\"name\":\"" + jsonEscape(groupNames[i]) + "\",";
      json += "\"entities\":\"" + jsonEscape(groupEntities[i]) + "\"}";
    }
    json += "]}";
    server.send(200, "application/json; charset=utf-8", json);
    return;
  }

  String name = server.arg("name");
  name.trim();
  if (name.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "name fehlt.");
    return;
  }

  int slot = -1;
  for (uint8_t i = 0; i < groupCount; i++) {
    if (groupNames[i].equalsIgnoreCase(name)) { slot = i; break; }
  }

  if (server.hasArg("delete")) {
    if (slot < 0) {
      server.send(404, "text/plain; charset=utf-8", "Gruppe nicht gefunden.");
      return;
    }
    for (uint8_t i = slot; i + 1 < groupCount; i++) {
      groupNames[i] = groupNames[i + 1];
      groupEntities[i] = groupEntities[i + 1];
    }
    groupCount--;
    groupNames[groupCount] = "";
    groupEntities[groupCount] = "";
    groupsSave();
    server.send(200, "text/plain; charset=utf-8", name + " geloescht.");
    return;
  }

  String entities = server.arg("entities");
  entities.trim();
  name.replace("\t", " ");
  entities.replace("\t", "");

  if (slot < 0) {
    if (groupCount >= GROUP_MAX) {
      server.send(507, "text/plain; charset=utf-8",
                  "Es passen " + String(GROUP_MAX) + " Gruppen.");
      return;
    }
    slot = groupCount++;
  }

  groupNames[slot] = name;
  groupEntities[slot] = entities;

  if (!groupsSave()) {
    server.send(500, "text/plain; charset=utf-8", "Gruppen konnten nicht gespeichert werden.");
    return;
  }
  server.send(200, "text/plain; charset=utf-8", name + " gespeichert.");
}

// Waiting until seven in the morning to find out whether the instruction works
// is not a way to develop one.
void handleBriefingTest() {
  String text = server.hasArg("text") ? server.arg("text") : cfg.alarmBriefing;
  if (text.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "Kein Briefing hinterlegt.");
    return;
  }

  briefingTestRequest = text;
  server.send(200, "text/plain; charset=utf-8",
              "Briefing wird abgefragt, das dauert einen Moment.");
}

void handleAlarm() {
  bool reading = server.args() == 0;
  if (reading) {
    String json = "{\"set\":" + String(cfg.alarmMinutes >= 0 ? "true" : "false");
    json += ",\"time\":\"" + clockText(cfg.alarmMinutes) + "\"";
    json += ",\"minutes\":" + String(cfg.alarmMinutes);
    json += ",\"daily\":" + String(cfg.alarmDaily ? "true" : "false");
    json += ",\"seconds_until\":" + String(alarmSecondsUntil());
    json += ",\"sound\":\"" + jsonEscape(cfg.alarmSound) + "\"";
    json += ",\"briefing\":\"" + jsonEscape(cfg.alarmBriefing) + "\"}";
    server.send(200, "application/json; charset=utf-8", json);
    return;
  }

  if (server.hasArg("clear") || server.hasArg("delete")) {
    alarmClear("API");
    server.send(200, "text/plain; charset=utf-8", "Wecker geloescht.");
    return;
  }

  if (server.hasArg("daily")) {
    cfg.alarmDaily = server.arg("daily") == "1" || server.arg("daily") == "true";
  }
  if (server.hasArg("sound")) cfg.alarmSound = server.arg("sound");
  if (server.hasArg("briefing")) {
    String text = server.arg("briefing");
    String clean = sanitizeUtf8(text);
    if (clean != text) {
      diagLog("ALARM", "Briefing enthielt ungueltiges UTF-8, wurde repariert");
    }
    cfg.alarmBriefing = clean;
  }

  if (server.hasArg("time")) {
    String value = server.arg("time");
    int colon = value.indexOf(':');
    if (colon < 1) {
      server.send(400, "text/plain; charset=utf-8", "Zeit als HH:MM angeben.");
      return;
    }
    int hh = value.substring(0, colon).toInt();
    int mm = value.substring(colon + 1).toInt();
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59) {
      server.send(400, "text/plain; charset=utf-8", "Ungueltige Zeit.");
      return;
    }
    alarmSet(hh * 60 + mm);
    server.send(200, "text/plain; charset=utf-8",
                "Wecker auf " + clockText(cfg.alarmMinutes) + " gestellt.");
    return;
  }

  saveConfig();
  server.send(200, "text/plain; charset=utf-8", "Gespeichert.");
}

void handleTimer() {
  bool reading = server.args() == 0;
  if (reading) {
    String json = "{\"active\":" + String(timerEndsAt != 0 ? "true" : "false");
    json += ",\"remaining_s\":" + String(timerSecondsLeft());
    json += ",\"total_s\":" + String(timerTotalSec);
    json += ",\"sound\":\"" + jsonEscape(cfg.timerSound) + "\"}";
    server.send(200, "application/json; charset=utf-8", json);
    return;
  }

  if (server.hasArg("clear") || server.hasArg("delete")) {
    timerClear("API");
    server.send(200, "text/plain; charset=utf-8", "Timer geloescht.");
    return;
  }

  if (server.hasArg("sound")) {
    cfg.timerSound = server.arg("sound");
    saveConfig();
  }

  if (server.hasArg("seconds") || server.hasArg("minutes")) {
    long seconds = server.hasArg("seconds") ? server.arg("seconds").toInt()
                                            : server.arg("minutes").toInt() * 60;
    if (seconds < 5 || seconds > 12L * 3600L) {
      server.send(400, "text/plain; charset=utf-8", "Zwischen 5 Sekunden und 12 Stunden.");
      return;
    }
    timerSet((uint32_t)seconds);
    server.send(200, "text/plain; charset=utf-8",
                "Timer laeuft " + String(seconds) + " Sekunden.");
    return;
  }

  server.send(200, "text/plain; charset=utf-8", "Gespeichert.");
}

void handleRadioList() {
  String json = "{\"active\":" + String(radioActive ? "true" : "false");
  json += ",\"station\":\"" + jsonEscape(radioStationName) + "\"";
  json += ",\"status\":\"" + jsonEscape(radioStatus) + "\"";
  json += ",\"stations\":[";
  for (uint8_t i = 0; i < radioStationCount; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + jsonEscape(radioStationNames[i]) + "\",";
    json += "\"url\":\"" + jsonEscape(radioStationUrls[i]) + "\"}";
  }
  json += "]}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleRadioPlay() {
  String name = server.arg("name");
  String url = server.arg("url");

  if (url.length() == 0 && name.length() > 0) {
    int index = radioFindStation(radioNormalize(name));
    if (index < 0) {
      server.send(404, "text/plain; charset=utf-8", "Sender nicht gefunden.");
      return;
    }
    name = radioStationNames[index];
    url = radioStationUrls[index];
  }

  if (url.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "name oder url fehlt.");
    return;
  }

  if (!radioStart(name, url)) {
    server.send(500, "text/plain; charset=utf-8", "Radio startet nicht: " + radioStatus);
    return;
  }

  server.send(200, "text/plain; charset=utf-8", "Radio " + radioStationName + " laeuft.");
}

void handleRadioStop() {
  if (!radioActive) {
    server.send(200, "text/plain; charset=utf-8", "Radio war schon aus.");
    return;
  }
  radioStop("Webinterface");
  server.send(200, "text/plain; charset=utf-8", "Radio gestoppt.");
}

void handleRadioSave() {
  String name = server.arg("name");
  String url = server.arg("url");
  name.trim();
  url.trim();

  if (name.length() == 0 || url.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "Name und URL werden gebraucht.");
    return;
  }
  if (!url.startsWith("http://") && !url.startsWith("https://")) {
    server.send(400, "text/plain; charset=utf-8", "Die URL muss mit http:// oder https:// beginnen.");
    return;
  }
  // Tab is the field separator in the station file.
  name.replace("\t", " ");
  url.replace("\t", "");

  int slot = -1;
  for (uint8_t i = 0; i < radioStationCount; i++) {
    if (radioStationNames[i].equalsIgnoreCase(name)) { slot = i; break; }
  }
  if (slot < 0) {
    if (radioStationCount >= RADIO_MAX_STATIONS) {
      server.send(507, "text/plain; charset=utf-8",
                  "Liste voll, es passen " + String(RADIO_MAX_STATIONS) + " Sender.");
      return;
    }
    slot = radioStationCount++;
  }

  radioStationNames[slot] = name;
  radioStationUrls[slot] = url;

  if (!radioSaveStations()) {
    server.send(500, "text/plain; charset=utf-8", "Senderliste konnte nicht gespeichert werden.");
    return;
  }

  server.send(200, "text/plain; charset=utf-8", name + " gespeichert.");
}

void handleRadioDelete() {
  String name = server.arg("name");
  int slot = -1;
  for (uint8_t i = 0; i < radioStationCount; i++) {
    if (radioStationNames[i] == name) { slot = i; break; }
  }
  if (slot < 0) {
    server.send(404, "text/plain; charset=utf-8", "Sender nicht gefunden.");
    return;
  }

  for (uint8_t i = slot; i + 1 < radioStationCount; i++) {
    radioStationNames[i] = radioStationNames[i + 1];
    radioStationUrls[i] = radioStationUrls[i + 1];
  }
  radioStationCount--;
  radioStationNames[radioStationCount] = "";
  radioStationUrls[radioStationCount] = "";
  radioSaveStations();

  server.send(200, "text/plain; charset=utf-8", name + " geloescht.");
}

// Reading the codec back is the only way to tell a setting that was applied
// from one that was merely intended.
void handleMicInfo() {
  String json = "{\"present\":" + String(es7210Present ? "true" : "false");
  json += ",\"configured_gain_db\":" + String(cfg.micGainDb);
  json += ",\"expected_register\":" + String(0x10 | es7210GainReg((float)cfg.micGainDb));
  json += ",\"registers\":{";

  const uint8_t regs[] = {0x01, 0x02, 0x07, 0x20, 0x21, 0x22, 0x23,
                          0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46};
  bool first = true;
  for (uint8_t i = 0; i < sizeof(regs); i++) {
    uint8_t value = 0;
    bool ok = codecRead(ADDR_ES7210, regs[i], value);
    if (!first) json += ",";
    first = false;
    json += "\"0x" + String(regs[i], HEX) + "\":" + (ok ? String(value) : String(-1));
  }
  json += "}}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleAdcScan() {
  String json = "{";

  uint8_t p0 = 0;
  uint8_t p1 = 0;
  bool expanderOk = tca9555Present &&
                    codecRead(ADDR_TCA9555, 0x00, p0) &&
                    codecRead(ADDR_TCA9555, 0x01, p1);
  json += "\"expander\":{\"present\":" + String(tca9555Present ? "true" : "false");
  json += ",\"read_ok\":" + String(expanderOk ? "true" : "false");
  json += ",\"port0\":" + String(p0) + ",\"port1\":" + String(p1) + "},";

  json += "\"channels\":[";
  for (uint8_t pin = 1; pin <= 9; pin++) {
    if (pin > 1) json += ",";
    uint32_t mv = analogReadMilliVolts(pin);
    json += "{\"gpio\":" + String(pin) + ",\"mv\":" + String(mv) + "}";
  }
  json += "]}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleLedTest() {
  if (!pixels) {
    server.send(500, "text/plain; charset=utf-8", "LED-Treiber nicht initialisiert.");
    return;
  }

  if (server.arg("phase") == "yield") {
    setLedPhase(LED_PHASE_YIELD);
    server.send(200, "text/plain; charset=utf-8", "Zeige das Schlafenlegen.");
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

  speakerTestRequested = true;
  server.send(200, "text/plain; charset=utf-8", "Testton läuft.");
}

void handleAssistWake() {
  if (!wakeCanStart()) {
    diagLogf("HTTP_WAKE", "blocked busy=%u cooldownLeft=%ld",
             wakeBusy ? 1 : 0,
             (long)((int32_t)(wakeIgnoreUntil - millis())));
    server.send(409, "text/plain; charset=utf-8",
                "Assist läuft bereits oder Cooldown aktiv.");
    return;
  }

  diagLog("HTTP_WAKE", "accepted");
  requestWake("web");
  server.send(200, "text/plain; charset=utf-8",
              "Wake gestartet. Jetzt sprechen ...");
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
  server.on("/api/runtime", HTTP_POST, handleRuntime);
  server.on("/api/log", HTTP_GET, handleLog);
  // HTTP_ANY so a plain browser URL works too:
  //   http://<host>/api/announce?text=Hallo
  server.on("/api/announce", HTTP_ANY, handleAnnounce);
  server.on("/api/hardware/mic-info", HTTP_GET, handleMicInfo);
  server.on("/api/hardware/adc-scan", HTTP_GET, handleAdcScan);
  server.on("/api/update/check", HTTP_ANY, handleUpdateCheck);
  server.on("/api/update/install", HTTP_ANY, handleUpdateInstall);
  server.on("/api/ha/entities", HTTP_GET, handleHaEntities);
  server.on("/api/groups", HTTP_ANY, handleGroups);
  server.on("/api/alarm/briefing-test", HTTP_ANY, handleBriefingTest);
  server.on("/api/alarm", HTTP_ANY, handleAlarm);
  server.on("/api/timer", HTTP_ANY, handleTimer);
  server.on("/api/radio/list", HTTP_GET, handleRadioList);
  server.on("/api/radio/play", HTTP_ANY, handleRadioPlay);
  server.on("/api/radio/stop", HTTP_ANY, handleRadioStop);
  server.on("/api/radio/save", HTTP_ANY, handleRadioSave);
  server.on("/api/radio/delete", HTTP_ANY, handleRadioDelete);
  server.on("/api/sound/list", HTTP_GET, handleSoundList);
  server.on("/api/sound/delete", HTTP_ANY, handleSoundDelete);
  server.on("/api/sound/play", HTTP_ANY, handleSoundPlay);
  server.on("/api/sound/upload", HTTP_POST, handleSoundUploadDone, handleSoundUploadData);
  server.on("/api/volume", HTTP_ANY, handleVolume);
  server.on("/api/ha/pipelines", HTTP_POST, handlePipelineRefresh);
  server.on("/api/ack/build", HTTP_POST, handleAckBuild);
  server.on("/api/ack/test", HTTP_POST, handleAckTest);
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
  Serial.println("BOOT: setup entered");
  Serial.println("==============================================");
  Serial.printf(" VoiceDot Waveshare Firmware v%s\n", FW_VERSION);
  Serial.println("==============================================");

  Serial.printf("Flash size : %lu\n", (unsigned long)ESP.getFlashChipSize());
  Serial.printf("PSRAM size : %lu\n", (unsigned long)ESP.getPsramSize());
  Serial.printf("Free heap  : %lu\n", (unsigned long)ESP.getFreeHeap());

  Serial.printf("BOOT: last reset was %s\n", resetReasonName().c_str());

  Serial.println("BOOT: load config");
  loadConfig();

  // Detect the exact Waveshare audio hardware first.
  Serial.println("BOOT: detect board");
  detectBoardProfile();
  Serial.println("BOOT: setup buttons");
  setupButtons();

  // RGB ring
  Serial.println("BOOT: setup leds");
  setupLeds();
  ledsBootAnimation();

  // Keep amp muted during boot.
  Serial.println("BOOT: amp off");
  if (tca9555Present) {
    setAmplifier(false);
  }

  Serial.println("BOOT: setup audio");
  setupAudio();

  Serial.println("BOOT: filesystem");
  ackFsReady = LittleFS.begin(true);
  if (ackFsReady) {
    ackScanClips();
    logPrintf("LittleFS: ready, %u Ansage-Clips gespeichert", ackClipCount);
    if (ackClipCount > 0) ackBuildStatus = String(ackClipCount) + " Ansagen gespeichert";
  } else {
    logPrintf("LittleFS: mount failed, Ansagen nicht verfuegbar");
  }

  logPrintf("Neustart-Grund: %s", resetReasonName().c_str());
  updateLoadResult();

  bool pendingUpdate = updateHasPending();

  Serial.println("BOOT: wifi");
  bool wifiOk = connectToWiFi();

  if (!wifiOk) {
    startProvisioningAp();
    ledsStatusSetup();
  } else {
    apMode = false;
    startMdns();
    ledsStatusReady();
  }

  if (!apMode) {
    // NTP with the timezone rule, so local time and the DST switch are right
    // without any extra library.
    // The RTC gets us a usable clock immediately; NTP then corrects it.
    setenv("TZ", cfg.timezone.c_str(), 1);
    tzset();
    clockFromRtc();

    configTzTime(cfg.timezone.c_str(), "pool.ntp.org", "time.cloudflare.com");
    struct tm t;
    if (getLocalTime(&t, 4000)) {
      timeValid = true;
      logPrintf("Zeit: %04d-%02d-%02d %02d:%02d (%s)",
                t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                t.tm_hour, t.tm_min, cfg.timezone.c_str());
      clockToRtc();
    } else if (timeValid) {
      logPrintf("Zeit: NTP stumm, laeuft auf der RTC weiter");
    } else {
      logPrintf("Zeit: weder NTP noch RTC, Tag/Nacht-Profil wartet");
    }
  } else {
    setenv("TZ", cfg.timezone.c_str(), 1);
    tzset();
    clockFromRtc();
  }

  applySchedule(true);

  // Everything that needs TLS happens here, while the heap is still whole.
  if (pendingUpdate && wifiOk) updateRunPending();  // reboots when it worked

  if (cfg.updateCheckEnabled && wifiOk && !apMode) {
    Serial.println("BOOT: nach Updates sehen");
    updateFetchReleases();
  }

  Serial.println("BOOT: wake word");
  srBegin();

  groupsLoad();
  logPrintf("Gruppen: %u geladen", groupCount);

  radioLoadStations();
  logPrintf("Radio: %u Sender gespeichert", radioStationCount);

  multiBegin();

  Serial.println("BOOT: web server");
  setupWebServer();

  setLedPhase(LED_PHASE_IDLE);

  Serial.println();
  Serial.println("VoiceDot ready.");

  if (apMode) {
    Serial.printf("Setup WiFi: %s\n", WiFi.softAPSSID().c_str());
    Serial.printf("Password  : %s\n", AP_PASSWORD);
    Serial.println("Open      : http://192.168.4.1");
  } else {
    Serial.printf("Open      : http://%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("or        : http://%s.local\n", deviceHostname().c_str());
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
  Serial.printf("Wake word   : %s\n", srStatus.c_str());
}

void loop() {
  // Long run health: a slow drift in the minimum free heap is the signal that
  // String churn is fragmenting memory. Cheap enough to log every five minutes.
  {
    static uint32_t lastHeapLog = 0;
    if ((uint32_t)(millis() - lastHeapLog) > 300000UL) {
      lastHeapLog = millis();
      diagLogf("HEALTH", "up=%lus heap=%lu min=%lu maxBlock=%lu psram=%lu rssi=%d",
               (unsigned long)(millis() / 1000UL),
               (unsigned long)ESP.getFreeHeap(),
               (unsigned long)ESP.getMinFreeHeap(),
               (unsigned long)ESP.getMaxAllocHeap(),
               (unsigned long)ESP.getFreePsram(),
               (int)WiFi.RSSI());
    }
  }

  static uint32_t lastLoopDiag = 0;
  if (DEBUG_SERIAL && (wakeBusy || wakeRecording || wakeSending || ttsPlaybackActive) &&
      (uint32_t)(millis() - lastLoopDiag) > 5000) {
    lastLoopDiag = millis();
    diagLogf("LOOP", "alive wifi=%d clients heap=%lu",
             (int)WiFi.status(),
             (unsigned long)ESP.getFreeHeap());
  }

  pumpServices();
  pollButtons();
  pollMicLevel();
  ledTick();

  // Long running jobs are started here, never from inside a web handler.
  if (speakerTestRequested && !wakeBusy && !ttsPlaybackActive) {
    speakerTestRequested = false;
    playSpeakerTestTone();
  }

  if (srWakeFlag) {
    srWakeFlag = false;
    srDetections++;
    srLastDetectMs = millis();
    diagLogf("SR", "wake word detected #%lu", (unsigned long)srDetections);
    if (apMode || WiFi.status() != WL_CONNECTED) {
      diagLog("SR", "wake word ignored, no Home Assistant connection");
      wakeLastMessage = "Wake-Word erkannt, aber kein WLAN.";
    } else if (!wakeCanStart()) {
      diagLog("SR", "wake word ignored, busy or cooling down");
    } else if (!multiArbitrate()) {
      // A louder VoiceDot takes this one. Go back to sleep without recording,
      // and hold off briefly so the tail of the same word cannot retrigger us.
      wakeLastState = "idle";
      wakeLastMessage = "Anderer VoiceDot war näher: " + multiLastDecision;
      wakeIgnoreUntil = millis() + WAKE_COOLDOWN_MS;
      setLedPhase(LED_PHASE_YIELD);
    } else {
      requestWake("wakeword");
    }
  }

  if (wakeRequested && wakeCanStart()) {
    wakeRequested = false;
    runWakeCaptureAndHa();
  }

  if (cfg.multiEnabled && !apMode && WiFi.status() == WL_CONNECTED) {
    if (!multiReady) {
      static uint32_t lastMultiRetry = 0;
      if ((uint32_t)(millis() - lastMultiRetry) > 15000) {
        lastMultiRetry = millis();
        multiBegin();
      }
    } else {
      multiPoll();

      static uint32_t lastHello = 0;
      if ((uint32_t)(millis() - lastHello) > MULTI_HELLO_INTERVAL_MS) {
        lastHello = millis();
        multiSendHello();
      }
    }
  }

  // The noise floor only says something about the room while our own speaker is
  // quiet. During playback the microphone hears us, not the hairdryer.
  if (srRoomNoiseValid && !ttsPlaybackActive && !radioActive &&
      !speakerTestActive && !wakeBusy) {
    autoVolumeNoiseDb = srRoomNoiseDb;
    autoVolumeNoiseValid = true;
    autoVolumeNoiseAt = millis();
  }

  // The install itself happens on the next boot, where the whole heap is still
  // free. Here we only get out of the way.
  if (updateRebootAt != 0 && (int32_t)(millis() - updateRebootAt) >= 0) {
    logPrintf("Update: Neustart zum Installieren");
    ESP.restart();
  }

  if (cfg.updateCheckEnabled && !apMode && WiFi.status() == WL_CONNECTED &&
      !wakeBusy && !ttsPlaybackActive && !updateInProgress && !radioActive) {
    bool due = updateChecked &&
               (uint32_t)(millis() - updateCheckedAt) > UPDATE_AUTO_CHECK_MS;
    if (due) updateFetchReleases();
  }

  if (briefingTestRequest.length() > 0 && !wakeBusy && !ttsPlaybackActive) {
    String text = briefingTestRequest;
    briefingTestRequest = "";
    alarmSpeakBriefing(text);
  }

  alarmTimerTick();

  radioTick();

  if (soundPlayRequest.length() > 0 && !wakeBusy && !ttsPlaybackActive && !speakerTestActive) {
    String name = soundPlayRequest;
    soundPlayRequest = "";
    srPauseDetection();
    setLedPhase(LED_PHASE_SPEAK);
    playSoundFile(name);
    setLedPhase(LED_PHASE_IDLE);
    srResumeDetection();
  }

  if (announceRequested && !wakeBusy && !wakeRequested && !ttsPlaybackActive && !speakerTestActive) {
    announceRequested = false;
    srPauseDetection();
    playAnnouncement(announceText);
    setLedPhase(LED_PHASE_IDLE);
    srResumeDetection();
  }

  if (ackTestRequested && !wakeBusy && !ttsPlaybackActive && !speakerTestActive) {
    ackTestRequested = false;
    srPauseDetection();
    playAckSound();
    setLedPhase(LED_PHASE_IDLE);
    srResumeDetection();
  }

  if (pipelineListRequested && !wakeBusy && !wakeRequested && !ttsPlaybackActive) {
    pipelineListRequested = false;
    srPauseDetection();
    fetchPipelineList();
    srResumeDetection();
  }

  if (ackBuildRequested && !wakeBusy && !wakeRequested && !ttsPlaybackActive && !speakerTestActive) {
    ackBuildRequested = false;
    ackBuildStatus = "wird erzeugt ...";
    srPauseDetection();
    buildAckCache();
    srResumeDetection();
  }

  if (srApplyRequested && !wakeBusy && !wakeRequested && !ttsPlaybackActive && !speakerTestActive) {
    srApplyRequested = false;
    srApplyConfig();
  }

  if (srRunning) {
    static uint32_t lastSrStat = 0;
    static uint32_t lastSrFeeds = 0;
    if ((uint32_t)(millis() - lastSrStat) > 15000) {
      lastSrStat = millis();
      // feeds/s tells us whether the detector keeps up with the microphone:
      // it needs 16000/feedChunk per second, anything less means dropped audio.
      diagLogf("SR", "alive feeds=%lu (%lu/s need %lu/s) short=%lu rMax=%lu mMax=%lu fMax=%lu",
               (unsigned long)srFeedCount,
               (unsigned long)((srFeedCount - lastSrFeeds) * 1000UL / 15000UL),
               (unsigned long)(srFeedChunk > 0 ? 16000UL / (uint32_t)srFeedChunk : 0),
               (unsigned long)srShortReads,
               (unsigned long)srMaxReadMs,
               (unsigned long)srMaxMeterMs,
               (unsigned long)srMaxFeedMs);
      diagLogf("SR", "  fetches=%lu fail=%lu lastState=%d det=%lu paused=%u mic=%d/%ddB",
               (unsigned long)srFetchCount,
               (unsigned long)srFetchFail,
               srLastWakeupState,
               (unsigned long)srDetections,
               srPaused ? 1 : 0,
               (int)micLevel,
               (int)micDb);
      lastSrFeeds = srFeedCount;
      srMaxReadMs = 0;
      srMaxFeedMs = 0;
      srMaxMeterMs = 0;
    }
  }

  // Safety net: nothing is running but the detector is still standing down.
  if (srRunning && srPaused && !wakeBusy && !wakeRequested &&
      !ttsPlaybackActive && !speakerTestActive &&
      (uint32_t)(millis() - srPausedAt) > SR_STUCK_RESUME_MS) {
    diagLog("SR", "resuming after stuck pause");
    srResumeDetection();
  }

  if (cfg.haPublish && !wakeBusy && !ttsPlaybackActive && !speakerTestActive) {
    static String lastPublished = "";
    String now = voiceDotState();

    // Push on every state change, otherwise every 30 s as a heartbeat.
    if (now != lastPublished || (uint32_t)(millis() - haPublishAt) > 30000) {
      lastPublished = now;
      haPublishNow();
    }
  }

  if (timeValid && rtcPresent && !wakeBusy && !ttsPlaybackActive) {
    static uint32_t lastRtcSync = 0;
    if ((uint32_t)(millis() - lastRtcSync) > 3600000UL) {
      lastRtcSync = millis();
      clockToRtc();
    }
  }

  if (cfg.scheduleEnabled && !wakeBusy && !ttsPlaybackActive) {
    static uint32_t lastScheduleCheck = 0;
    if ((uint32_t)(millis() - lastScheduleCheck) > 20000) {
      lastScheduleCheck = millis();
      applySchedule(false);
    }
  }

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
