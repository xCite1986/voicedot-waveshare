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

## Button Mapping

For `v0.2.2`:

```text
K1 / EXIO09  Volume up
K3 / EXIO11  Volume down
K2 / EXIO10  Mute toggle
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

## Next Milestone

`v0.3` will activate the real audio path:

```text
ES7210 microphones -> ESP32-S3 -> live mic level
ESP32-S3 -> ES8311 -> onboard amplifier -> speaker test tone
```

