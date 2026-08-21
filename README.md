# VoiceDot Waveshare

Firmware for a Waveshare ESP32-S3-AUDIO-Board based Home Assistant voice satellite.

## Hardware

- Waveshare ESP32-S3-AUDIO-Board
- ESP32-S3R8, 16 MB flash, 8 MB PSRAM
- ES8311 speaker codec
- ES7210 dual microphone ADC
- TCA9555 I/O expander
- PCF85063 RTC
- 7x WS2812 RGB LEDs on GPIO38

## Arduino IDE Settings

```text
Board: ESP32S3 Dev Module
USB CDC On Boot: Enabled
USB Mode: Hardware CDC and JTAG
Flash Size: 16MB
Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
PSRAM: OPI PSRAM
Upload Speed: 921600
Serial Monitor: 115200 baud
```

Install this Arduino library:

```text
Adafruit NeoPixel
ESP8266Audio
```

## Firmware Versions

- `VoiceDot_Waveshare_v0_2_1`: stable board bring-up, WiFi, web UI, Home Assistant token test, hardware diagnostics.
- `VoiceDot_Waveshare_v0_2_2`: adds onboard button handling.
- `VoiceDot_Waveshare_v0_3_1`: stable local audio bring-up with microphone level and speaker test.
- `VoiceDot_Waveshare_v0_4_0`: test build for K2 wake recording and Home Assistant Assist pipeline upload.
- `VoiceDot_Waveshare_v0_4_1`: adds detailed diagnostics for Home Assistant WebSocket/auth failures.
- `VoiceDot_Waveshare_v0_4_2`: fixes HA WebSocket HTTP 400 upgrade error.
- `VoiceDot_Waveshare_v0_4_3`: parses Assist events, shows assistant/TTS diagnostics, and plays WAV/PCM16 TTS audio over ES8311.
- `VoiceDot_Waveshare_v0_4_4`: compile fix for Arduino-ESP32 Core 3.3.8 TTS stream reading.
- `VoiceDot_Waveshare_v0_4_5`: adds MP3 TTS decode/playback for Home Assistant `/api/tts_proxy/*.mp3`.

## Button Mapping

For `v0.4.x-test`:

```text
K1 / EXIO09  Volume up
K3 / EXIO11  Volume down
K2 / EXIO10  Short press: wake / record / send to Home Assistant Assist
K2 / EXIO10  Long press: mute toggle
BOOT / GPIO0 LED brightness +10, wraps from 100 to 0
```

## First Start

If WiFi is not configured, the board opens:

```text
SSID: VoiceDot-XXXXXX
Password: voicedot
URL: http://192.168.4.1
```

After WiFi setup:

```text
http://voicedot.local
```

or the IP shown in the serial monitor.

## Current Test Milestone

`v0.4.5-test` records a short PCM sample after K2 short press or the web UI wake button,
sends it to the Home Assistant Assist pipeline over WebSocket, then fetches and plays
Home Assistant TTS when the returned media is WAV/PCM16 or MP3. Ogg/Opus is shown in
the diagnostics and needs a later decoder step.
