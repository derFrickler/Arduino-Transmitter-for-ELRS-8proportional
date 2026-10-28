# SimpleTX ELRS Transmitter

This project turns an Arduino Nano into a compact ELRS transmitter using CRSF over a serial link. The code currently targets full 16-channel operation and supports a mix of analog sticks, logical switches, and optional extra analog inputs.

## Board

- Arduino Nano (ATmega328P)
- Framework: Arduino
- Build tool: PlatformIO

## ELRS / CRSF wiring

The project uses the Nano hardware serial port:

- D0/RX -> ELRS TX
- D1/TX -> ELRS RX
- GND -> common ground
- 5V -> ELRS VCC only if the module supports it and the wiring is compatible with the board used

> For most ELRS Nano modules, the serial is connected as TTL UART with a common ground. Check the ELRS module pinout for exact voltage compatibility before powering it.

## Nano pin mapping

### Main stick inputs

| Arduino pin | Function | ELRS channel |
| --- | --- | --- |
| A0 | Aileron | CH1 |
| A1 | Elevator | CH2 |
| A2 | Throttle | CH3 |
| A3 | Rudder | CH4 |

### Extra analog / switch mapping

| Arduino pin | Function | ELRS channel |
| --- | --- | --- |
| A4 | Analog input | CH6 |
| A5 | Analog input | CH7 |
| A6 | Analog input | CH8 |
| A7 | Analog input | CH9 |
| D5 | Toggle switch | CH5 |
| D6 + D7 | 3-position switch pair | CH12 |
| D8 + D9 | 3-position switch pair | CH10 |
| D10 + D11 | 3-position switch pair | CH11 |

### Output / status pins

| Arduino pin | Function | Notes |
| --- | --- | --- |
| D2 | LED | status LED |
| D3 | Buzzer | active buzzer |
| D0 | Serial RX | ELRS TX |
| D1 | Serial TX | ELRS RX |

## Channel layout

The current custom mapping follows the requested hardware layout:

| ELRS channel | Source |
| --- | --- |
| CH1 | A0 / Aileron |
| CH2 | A1 / Elevator |
| CH3 | A2 / Throttle |
| CH4 | A3 / Rudder |
| CH5 | D5 toggle switch |
| CH6 | A4 analog input |
| CH7 | A5 analog input |
| CH8 | A6 analog input |
| CH9 | A7 analog input |
| CH10 | D8 + D9 3-position switch pair |
| CH11 | D10 + D11 3-position switch pair |
| CH12 | D6 + D7 3-position switch pair |
| CH13..CH16 | centered unless otherwise configured |

## VCC monitoring

Eevery 30 seconds, the firmware measures the Nano's VCC using the internal 1.1 V bandgap, then restores the ADC channel selection. Potentiometers use the default 5 V AVcc reference.

The LED warning starts below 4.9 V VCC, and the buzzer alarm starts below 4.7 V VCC. Both thresholds are configurable with `VCC_WARNING_MILLIVOLTS` and `VCC_BEEPING_MILLIVOLTS` in [src/config.h](src/config.h). The internal measurement observes regulated VCC, not VIN: while the regulator holds VCC steady, firmware cannot determine whether VIN is 6.5 V. An exact VIN threshold requires a voltage divider connected to an ADC pin. The nominal 1.1 V bandgap varies between chips, so calibrate the reference value if more precise thresholds are needed.

## Analog calibration

The calibration routine covers all active analog inputs: A0–A3 for the four stick axes, A4–A6 for CH6–CH8, and A7 for CH9. With `GIMBAL_CALIBRATION` enabled in `src/main.cpp`, hold the stick axes centered for the first 5 seconds, then move each analog control through its full range for the next 15 seconds. The calibrated ranges are saved to EEPROM. Existing calibration data is reinitialized after this calibration format change.

## Channel reverse

Set the corresponding entry in `Is_Channel_Reverse` in `src/config.h` to `1` to reverse an output channel; entries are ordered CH1 through CH16. CH1 remains reversed by default to preserve the existing stick behavior; all other channels default to normal. Reversal applies uniformly to analog and switch channels, and leaves centered channels centered.

## Default radio config

The project defaults to ELRS packet rate:

- `100Hz Full`

The values are defined in [src/config.h](src/config.h) as `SETTING_1_PktRate` and `SETTING_2_PktRate`.

## Build

```bash
platformio run
```

To upload:

```bash
platformio run --target upload
```

## Notes

- The potentiometers use the Nano's default AVcc ADC reference; the bandgap-based VCC reading uses the nominal 1.1 V bandgap value, which may need calibration for a precise threshold.
- Some inputs are optional depending on hardware and config.
- The current pin assignment reflects the project’s current Nano wiring and is intended as the master reference for this build.
