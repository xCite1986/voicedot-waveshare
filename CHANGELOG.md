# Changelog

## v0.3.1

- Promoted the working v0.3 audio bring-up to stable.
- Fixed ES7210 microphone start clock handling.
- Added RX diagnostics for microphone reads, byte count, silent frames, and errors.
- Verified on device: speaker test works and microphone level responds.

## v0.3.0-test

- Added first ES8311/ES7210 audio init layer.
- Added shared I2S RX/TX setup at 16 kHz / 16-bit stereo.
- Added web UI mic-level meter.
- Added speaker test tone endpoint and button.
- Kept this as a test build; no stable tag yet.

## v0.2.2

- Added onboard button handling:
  - K1 / EXIO09: volume up
  - K3 / EXIO11: volume down
  - K2 / EXIO10: mute toggle
  - BOOT / GPIO0: LED brightness +10, wraps from 100 to 0
- Added runtime saving for volume and LED brightness.
- Changed LED brightness range to 0-100.

## v0.2.1

- Fixed Arduino-ESP32 Core 3.3.8 compile issue caused by JavaScript being interpreted as C++.
- Kept full web UI functionality.
- Removed invalid `uint8_t > 255` warning.
- Fixed `Serial.printf` formatting for ESP32 `uint32_t` values.
- Verified hardware detection on the board:
  - ES8311 at 0x18
  - ES7210 at 0x40
  - TCA9555 at 0x20
  - PCF85063 RTC at 0x51
