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
```

## Firmware Versions

- `VoiceDot_Waveshare_v0_2_1`: stable board bring-up, WiFi, web UI, Home Assistant token test, hardware diagnostics.
- `VoiceDot_Waveshare_v0_2_2`: adds onboard button handling.
- `VoiceDot_Waveshare_v0_3_1`: stable local audio bring-up with microphone level and speaker test.
- `VoiceDot_Waveshare_v0_4_0`: test build for K2 wake recording and Home Assistant Assist pipeline upload.

## Button Mapping

For `v0.4.0-test`:

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

`v0.4.0-test` records a short PCM sample after K2 short press or the web UI wake button,
then sends it to the Home Assistant Assist pipeline over WebSocket.
