# Grbl_ESP32 – Dual-Head Label Cutter (ESP32-S3)

A reduced fork of [Grbl_ESP32](https://github.com/bdring/Grbl_Esp32) for a
sheet-fed, dual-head label cutter (Vulcan SC 350 style) running on an
**ESP32-S3 N8R2**. It is a serial-only motion controller, driven by a
master controller over UART.

* 3 axes: X knife carriage (homed, soft limits 0…x_max), Y sheet transport and
  Z feed roller (both unbounded)
* 2 solenoid heads on PWM (`M3` / `M4` / `M5`), never on at the same time
* 2 PWM fans (`M67/M68 E0|E1 Q0-100`)
* Paper-present, rear and front IR sensors on interrupts
* `$FEED` and `$EJECT` paper sequences, event driven by the sensor ISRs
* The feed roller never turns without paper
* WiFi and Bluetooth removed; all web and network code deleted

```
pio run              # build (pioarduino platform, Arduino core 3.3.9)
pio run -t upload    # flash
```

Full documentation is in **[doc/LabelCutter.md](doc/LabelCutter.md)**: pin
map, protocol, G-code, sequences, settings, status format, error codes and
the integration guide.

Licensed under the GNU GPL v3, see [LICENSE](LICENSE). Based on Grbl
(Sungeun K. Jeon, Simen Svale Skogsrud) and Grbl_ESP32 (Bart Dring et al.).
