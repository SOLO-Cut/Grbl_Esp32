# Grbl_ESP32 – Dual-Head Label Cutter Firmware (ESP32-S3 N8R2)

This firmware is a reduced fork of Grbl_ESP32. It controls a sheet-fed,
dual-head label cutter similar to the Vulcan SC 350. The ESP32-S3 acts as a
motion controller. A master controller drives it over UART with standard
Grbl G-code plus two paper-handling commands, `$FEED` and `$EJECT`.

Contents

1. [What changed compared to Grbl_ESP32](#1-what-changed-compared-to-grbl_esp32)
2. [Hardware and pin map](#2-hardware-and-pin-map)
3. [Building and flashing](#3-building-and-flashing)
4. [Serial links and protocol](#4-serial-links-and-protocol)
5. [Axes, homing and limits](#5-axes-homing-and-limits)
6. [G-code reference](#6-g-code-reference)
7. [Paper sequences: `$EJECT` and `$FEED`](#7-paper-sequences-eject-and-feed)
8. [Feed-motor interlock](#8-feed-motor-interlock)
9. [Settings](#9-settings)
10. [Status report](#10-status-report)
11. [Error and alarm codes](#11-error-and-alarm-codes)
12. [Event-driven architecture](#12-event-driven-architecture)
13. [Master-controller integration](#13-master-controller-integration)
14. [Commissioning checklist](#14-commissioning-checklist)
15. [Known limitations](#15-known-limitations)

---

## 1. What changed compared to Grbl_ESP32

| Area | Change |
|---|---|
| Target | ESP32-S3 N8R2 (8 MB quad flash, 2 MB quad PSRAM). The firmware was ported from Arduino core 1.0 / IDF 3 to **Arduino core 3.3.9 / ESP-IDF 5.5** (pioarduino platform). |
| Radios | WiFi and Bluetooth are removed. No radio library is linked, so the radios never start. |
| Network / web | All WebUI, HTTP, telnet, mDNS, SSDP, OTA, notification, captive-portal and authentication code is deleted. The web UI assets (`data/`, `embedded/`) and the bundled `ESP32SSDP` / `arduinoWebSockets` libraries are deleted too. |
| Other removed features | SD card, spindle classes (PWM/VFD/laser/…), motor classes (Trinamic, servos, Dynamixel, unipolar), I2S stepping, RMT stepping, probing, coolant, safety door, parking, sleep, macro buttons, kinematics hooks, custom machine files, Visual Studio files, build scripts and CI workflows. |
| File count | `src/` went from about 150 files to **12** (plus `Grbl_Esp32.ino`). |
| Kept from Grbl | Planner (look-ahead, junction deviation), segment generator with AMASS, Bresenham step ISR, G-code parser, jogging, realtime overrides, `$` settings, work coordinate systems, startup lines. |
| Added | 3-axis machine definition (X knife carriage, Y sheet transport, Z feed roller), two mutually exclusive solenoid heads, two fans, three IR sensors, X-only homing and soft limits, `$FEED` and `$EJECT` sequences, feed-motor interlock, two serial ports (console + master link), and an event-driven main loop. |

### Source layout

```
Grbl_Esp32/
  Grbl_Esp32.ino        Arduino entry point (setup / loop)
  src/
    Config.h            Pin map and compile-time constants (edit this for wiring changes)
    Grbl.h              Single shared header: types, enums, prototypes
    Grbl.cpp            Init, soft reset, globals, number parser
    Serial.cpp          UART0 console + UART1 master link, realtime commands
    Protocol.cpp        Main loop, realtime state machine, feed hold / cancel
    GCode.cpp           G-code parser (heads, fans, interlock check)
    Planner.cpp         Motion planner (unchanged algorithm)
    Stepper.cpp         gptimer step ISR + segment generator, feed-roller step gate
    MotionControl.cpp   mc_line/arc/dwell, X homing, limits, jog, reset
    Settings.cpp        $-settings table, NVS storage, $ commands
    Report.cpp          Status / settings / parser-state reports
    Machine.cpp         Heads, fans, sensors (ISRs), $FEED / $EJECT sequences
```

---

## 2. Hardware and pin map

| GPIO | Function | Notes |
|---|---|---|
| 4 | X step | Knife carriage |
| 5 | X direction | |
| 6 | Y step | Sheet transport |
| 7 | Y direction | |
| 15 | Z step | Feed roller ("feed motor") |
| 16 | Z direction | |
| 21 | Stepper enable (shared) | Active low: LOW = drivers enabled (`$4=0`) |
| 8 | X limit switch | Switch to GND, internal pull-up, active LOW |
| 9 | Paper-present IR sensor | Internal pull-up, active LOW |
| 10 | Rear IR sensor | Internal pull-up, active LOW |
| 11 | Front IR sensor | Internal pull-up, active LOW |
| 12 | Head 1 solenoid | PWM 1 kHz, 12-bit |
| 13 | Head 2 solenoid | PWM 1 kHz, 12-bit, never on together with head 1 |
| 14 | Fan 1 | PWM 25 kHz, 10-bit |
| 1 | Fan 2 | PWM 25 kHz, 10-bit |
| 17 / 18 | UART1 TX / RX | Master controller link, 115200 8N1 |
| 43 / 44 | UART0 TX / RX | USB-serial console, 115200 8N1 |

"Active LOW" means the input reads LOW when it is triggered: the switch is
pressed, or the sensor sees paper. You can flip the polarity in the field
without recompiling:

* `$5=1` makes the X limit active HIGH.
* `$305` is a bit mask for the sensors (bit0 paper, bit1 rear, bit2 front). A
  set bit makes that sensor active HIGH.

If you rewire, edit only `src/Config.h`. Step, direction, enable, sensor,
limit and head pins must stay in GPIO 0–31. The ISRs use the 32-bit GPIO
set/clear registers, and the code has compile-time `static_assert`s that
check this.

**Electrical notes**

* The solenoids need a low-side MOSFET driver and a flyback diode. Do not
  drive them straight from the GPIO.
* Use 4-wire PWM fans, or a MOSFET for 2-wire fans.
* The IR sensor lines run next to PWM-switched solenoid wiring. A small RC
  filter (for example 1 kΩ + 10 nF) on each sensor input is recommended,
  because the sensor ISRs act on the first edge.

---

## 3. Building and flashing

The build uses PlatformIO with the **pioarduino** platform 55.03.39. That
platform provides Arduino core 3.3.9 and ESP-IDF 5.5, which support the S3.

```
pio run                  # build
pio run -t upload        # flash through the USB-serial port (UART0, GPIO43/44)
pio device monitor       # console, 115200 baud
```

`platformio.ini` selects the board `esp32-s3-devkitc1-n8r2` (8 MB QIO flash,
2 MB quad PSRAM) and the `default_8MB.csv` partition table. It also sets
`ARDUINO_USB_CDC_ON_BOOT=0`, so the console is the UART0 bridge and not the
native USB port.

If the platform is not installed on a machine, change the `platform =` line
to the release URL given in the comment at the top of `platformio.ini`.

On first boot, and whenever the settings layout changes, the defaults are
written to NVS and `[MSG:Settings restored to defaults]` is printed.

---

## 4. Serial links and protocol

Both UARTs run the same Grbl protocol at **115200 8N1**:

* **UART1 (GPIO17 TX / GPIO18 RX)**: the master controller.
* **UART0 (GPIO43 TX / GPIO44 RX)**: the USB-serial console for setup and
  debugging.

Each port has its own line buffer, `RX_BUFFER_SIZE` = 256 bytes.

* `ok` / `error:N` responses go only to the port that sent the line.
* `ALARM:N`, `[MSG:...]` and the reset banner go to both ports.
* Each line ends with `\n` (`\r\n` is accepted). The maximum line length is
  255 characters.

**Flow control.** Use Grbl "send-response" streaming: send a line, wait for
`ok`/`error`. Character-counting streaming also works if you count against
a 128-byte window. If a host overruns the buffer, the excess bytes are
dropped and the port reports `error:11`. This keeps the receive task from
ever blocking, so realtime characters always get through.

**Realtime characters** are acted on immediately, even during `$FEED` or
homing. They are never queued:

| Byte | Action |
|---|---|
| `?` | Status report (answered to the requesting port) |
| `!` | Feed hold |
| `~` | Cycle start / resume |
| `0x18` (Ctrl-X) | Soft reset. Heads go up immediately and motion stops. |
| `0x85` | Jog cancel |
| `0x90`–`0x94` | Feed override reset / +10 / −10 / +1 / −1 % |
| `0x95`–`0x97` | Rapid override 100 / 50 / 25 % |

**`$` commands**

| Command | Function |
|---|---|
| `$` | Help |
| `$$` | List all settings |
| `$n` | Print one setting, e.g. `$300` |
| `$n=v` | Change a setting (stored in NVS) |
| `$#` | Coordinate offsets G54–G59, G28, G30, G92, TLO |
| `$G` | Parser state |
| `$I` | Build info |
| `$N` / `$N0=line` / `$N1=line` | Startup lines (run after reset and after `$H`) |
| `$C` | Toggle G-code check mode |
| `$X` | Clear the alarm lock |
| `$H` (or `$HX`) | Home X |
| `$J=...` | Jog, e.g. `$J=G91 X10 F3000` |
| `$FEED` | Run the paper feed sequence (section 7) |
| `$EJECT` | Run the paper eject sequence (section 7) |
| `$RST=$` / `$RST=#` / `$RST=*` | Restore settings / clear offsets / both plus startup lines |

---

## 5. Axes, homing and limits

| Axis | Mechanism | Range | Switch |
|---|---|---|---|
| X | Knife carriage | 0 … `$130` (soft limits) | X limit on GPIO8 |
| Y | Sheet transport | Unbounded | none |
| Z | Feed roller | Unbounded | none |

In G-code the feed roller is the **Z** axis. Positive Y moves the sheet
**forward**, from the rear sensor toward the front sensor. Positive Z feeds
paper **into** the machine. If your wiring runs an axis backwards, fix it
with `$3` (direction invert mask) rather than in G-code.

**Homing (`$H`).** Only X physically moves. When X homing succeeds, machine
Y and Z are set to 0 without moving them. If homing fails, Y and Z are left
unchanged.

* With `$23` bit0 = 1 (default), the switch is at the X = 0 end. The carriage
  seeks toward −X at `$25`, backs off by `$27`, re-approaches slowly at
  `$24`, then backs off by `$27` again. X is set to `$27` (the pull-off) and
  the soft-limit range is 0 … `$130`.
* With `$23` bit0 = 0, the switch is at the X = `$130` end, and X ends at
  `$130 − $27`.

The limit switch is serviced by a GPIO interrupt. During the approach, the
ISR locks the X axis on the exact switch edge, so there is no polling.

**Power-up lock.** With homing enabled (`$22=1`), the controller starts in
`Alarm` and rejects G-code until `$H` (or `$X`) is sent.

**Soft limits (`$20=1`).** These apply to X only. A G-code move outside
0 … `$130` causes a controlled stop and `ALARM:2`. A jog outside the range
is rejected with `error:15`. Soft limits require `$22=1`.

**Hard limit (`$21=1`).** If the X switch triggers outside homing, heads go
up, the steppers stop and `ALARM:1` is raised. A reset is required.

**Unbounded axes.** Y and Z are never limited. To keep the float coordinates
small, machine Y and Z are reset to 0 after every `$FEED` and `$EJECT` (see
`$306`).

---

## 6. G-code reference

Supported (same as Grbl unless noted):

* Motion: `G0 G1 G2 G3 G80`, planes `G17 G18 G19`, `G90 G91 G91.1`,
  `G93 G94`, `G20 G21`
* Non-modal: `G4 Pn` (dwell, seconds), `G10 L2/L20`, `G28 G28.1 G30 G30.1`,
  `G53`, `G92 G92.1`
* `G54`–`G59`, `G40`, `G43.1 Z` / `G49`, `G61`
* Program flow: `M0 M1 M2 M30`; `M6` is accepted and ignored
* Words: `F` feed (mm/min), `S` head power, `N` line number, `T` tool (tracked
  only)

### Heads (solenoids)

Head selection uses Grbl's spindle modal group, so the commands are
synchronized with motion. Every head change happens only after all queued
moves have finished.

| Command | Effect |
|---|---|
| `M3` | Head 1 down (head 2 is forced up first) |
| `M4` | Head 2 down (head 1 is forced up first) |
| `M5` | Both heads up |
| `S<n>` | Head hold power. `S$30` (default `S1000`) = 100 % duty. `S` is modal and defaults to `$30` after reset, so a plain `M3` means full power. |

Examples: `M3` (head 1, full power), `M4 S600` (head 2 at 60 %), `M5`.

**Mutual exclusion.** Before any head is switched on, both channels are
stopped immediately in hardware (`ledc_stop`, output held low). Because M3
and M4 are in the same modal group, the parser can never request both. An
emergency stop (reset, alarm, hard limit) disconnects both pins from the PWM
peripheral and drives them low from the ISR.

Optional solenoid tuning:

* `$307` pull-in: the solenoid is driven at 100 % for `$307` ms, then drops
  to the S hold duty.
* `$308` switch delay: when switching directly from head 1 to head 2 (or the
  reverse), the firmware waits `$308` ms after the first head is released.

### Fans

| Command | Effect |
|---|---|
| `M67 E0 Q<0-100>` | Fan 1 duty %, applied after queued motion completes |
| `M67 E1 Q<0-100>` | Fan 2 duty %, applied after queued motion completes |
| `M68 E<0/1> Q<0-100>` | Same, but applied immediately |

Fans switch off on reset, as Grbl does for analog outputs.

### Not supported

Probing (`G38.x`), coolant (`M7 M8 M9`), digital outputs (`M62`–`M65`),
spindle overrides, A/B/C axes.

---

## 7. Paper sequences: `$EJECT` and `$FEED`

Both sequences are `$` commands. The reply (`ok` or `error:N`) arrives when
the sequence has **finished**, so the master just waits for it.

* Send them in `Idle` state. If G-code motion is still queued, the command
  first waits for it to finish.
* While a sequence runs, `?` reports `Run` or `Idle` plus `|Seq:FEED,<step>`
  or `|Seq:EJECT,<step>`.
* Both sequences raise the heads first.
* `!` / `~` pause and resume a sequence. Ctrl-X aborts it with `error:154`.

### How a "move until sensor" step works (event driven)

1. The step queues one normal planner move of up to `$303` mm at `$302`
   mm/min (paper speed of the dominant axis).
2. The sensor GPIO interrupt fires on the edge. Within the ISR it records
   the exact machine position (steps) and requests a motion cancel.
3. The machine decelerates in a controlled way using normal acceleration.
   Steps are not lost.
4. Any follow-up distance (DR, DF) is measured **from the recorded edge
   position**, not from where the machine stopped. Deceleration overshoot
   therefore does not affect registration accuracy.
5. If the edge does not arrive within `$303` mm, the sequence stops with
   `error:151`.

If a condition is already true when a step starts, the step completes
without moving. This covers the case where no edge would ever arrive.

### `$EJECT`

1. If neither the rear nor the front sensor sees paper, it replies
   `[MSG:No sheet to eject]` and `ok`.
2. Otherwise Y moves forward until **both** rear and front sensors are clear.
   It then moves a further `$304` mm (default 0).
3. If `$306=1`, machine Y and Z are set to 0.

### `$FEED`

| Step | Action | Ends when | Failure |
|---|---|---|---|
| 1 | Eject a sheet still in the machine (same as `$EJECT`) | rear and front clear | `error:151` |
| 2 | Check the paper-present sensor | — | `error:150` (no paper) |
| 3 | Feed roller (Z) and Y forward together at `$302` | **rear sensor covered** | `error:152` if the tray runs empty first, `error:151` after `$303` mm |
| 4 | Feed + Y continue to *rear edge + DR* (`$300`) | target reached | — |
| 5 | Feed stops; Y forward | **rear sensor clear** (sheet tail passed) | `error:151` |
| 6 | Y reverses | **front sensor clear** (sheet lead edge at the front sensor) | `error:153` if the front sensor was not covered, `error:151` |
| 7 | Y moves to *front edge + DF* (`$301`, signed: + forward, − backward) | target reached | — |
| 8 | If `$306=1`, machine Y = 0 and Z = 0. Reply `[MSG:Feed complete]`, `ok`. | | |

After a successful `$FEED`, **machine Y = 0 is the registered sheet
position**: DF from the sheet lead edge. The cut job's Y coordinates can
therefore be referenced directly to the sheet.

During step 4, if the tray empties (last sheet), the feed roller stops
automatically (see section 8). Y keeps moving because the sheet is already
gripped by the Y rollers.

The feed roller and Y travel the same distance in steps 3 and 4. Calibrate
`$102` (Z steps/mm) so that 1 mm of Z equals 1 mm of paper surface travel,
the same as `$101` for Y.

### Sequence settings

| Setting | Default | Meaning |
|---|---|---|
| `$300` DR | 20 mm | Feed + Y travel after the rear sensor triggers |
| `$301` DF | 10 mm | Y travel after the front sensor clears (signed) |
| `$302` | 3000 mm/min | Paper speed for all sequence moves |
| `$303` | 600 mm | Maximum search travel for one sensor edge (jam detection) |
| `$304` | 0 mm | Extra Y travel after an eject has cleared both sensors |
| `$305` | 0 | Sensor polarity mask (bit0 paper, bit1 rear, bit2 front; 1 = active HIGH) |
| `$306` | 1 | Zero machine Y and Z after `$FEED` / `$EJECT` |

---

## 8. Feed-motor interlock

The feed roller (Z) must never turn without paper. This is enforced at three
levels:

1. **Parser.** Any G-code or jog line that would change Z (G0/G1/G2/G3,
   G28/G30, `$J=`) is rejected with `error:150` when the paper sensor reports
   an empty tray.
2. **Step ISR.** On every step tick, the Z step output is suppressed while the
   paper sensor reports no paper. The feed roller physically cannot step
   without paper, whatever the source of the motion.
3. **Paper-out event.** If the tray runs empty while a G-code move or jog is
   turning the feed roller, the paper sensor ISR raises a feed hold and
   `[MSG:Paper out - feed hold]` is reported. The same happens when a queued
   block that needs Z starts executing with no paper. Resuming with `~` is
   refused (`[MSG:No paper - load paper to resume]`) until paper is present.

Inside `$FEED` the sequence handles paper-out itself (steps 3 and 4 above).

---

## 9. Settings

| $ | Default | Description |
|---|---|---|
| 0 | 4 | Step pulse length, µs (min 2) |
| 1 | 250 | Step idle delay, ms, before the drivers are disabled (255 = always on) |
| 2 | 0 | Step pulse invert mask (bit0 X, bit1 Y, bit2 Z) |
| 3 | 0 | Direction invert mask |
| 4 | 0 | Invert stepper enable (0 = active-low enable on GPIO21) |
| 5 | 0 | Invert X limit (0 = active LOW) |
| 10 | 1 | Status mask: bit0 = report MPos (else WPos), bit1 = add `Bf:` |
| 11 | 0.010 | Junction deviation, mm |
| 12 | 0.002 | Arc tolerance, mm |
| 13 | 0 | Report in inches |
| 20 | 1 | Soft limits (X only, needs `$22=1`) |
| 21 | 1 | Hard limit (X switch) |
| 22 | 1 | Homing enable (X only; also enables the power-up alarm lock) |
| 23 | 1 | Homing direction mask; bit0 = 1 means the X switch is at X = 0 |
| 24 | 200 | Homing locate feed, mm/min |
| 25 | 2000 | Homing seek rate, mm/min |
| 26 | 250 | Homing debounce delay, ms |
| 27 | 2.000 | Homing pull-off, mm |
| 30 | 1000 | S value that equals 100 % head power |
| 100–102 | 80 | Steps/mm X, Y, Z |
| 110–112 | 12000, 12000, 6000 | Max rate mm/min X, Y, Z |
| 120–122 | 1500, 1500, 800 | Acceleration mm/s² X, Y, Z |
| 130 | 350 | X max travel = soft limit `x_max` (0 disables the X soft limit) |
| 300 | 20 | DR, mm |
| 301 | 10 | DF, mm (signed) |
| 302 | 3000 | Sequence paper speed, mm/min |
| 303 | 600 | Max sensor search travel, mm |
| 304 | 0 | Extra eject travel, mm |
| 305 | 0 | Sensor invert mask |
| 306 | 1 | Zero Y/Z after feed/eject |
| 307 | 0 | Head pull-in time at 100 %, ms (0 = off) |
| 308 | 0 | Head switch-over release delay, ms |
| 309 | 0 | Direction-to-step setup delay, µs (set e.g. 5 for TB6600-class drivers) |

All defaults are in `Settings.cpp` (`defaults`). Settings are stored in NVS
namespace `grbl`.

---

## 10. Status report

```
<Idle|MPos:12.000,0.000,0.000|Bf:15,255|FS:0,0|Pn:X|Sn:PRF|Hd:1|Fn:60,0|Seq:FEED,3|WCO:0.000,0.000,0.000|Ov:100,100,100>
```

| Field | Meaning |
|---|---|
| state | `Idle`, `Run`, `Hold:0` (hold complete), `Hold:1` (stopping), `Jog`, `Home`, `Alarm`, `Check` |
| `MPos` / `WPos` | Position X, Y, Z (Z = feed roller); `$10` selects which |
| `Bf:p,r` | Free planner blocks, free bytes in this port's RX buffer |
| `Ln:` | Line number of the executing block (if N words are used) |
| `FS:f,s` | Current feed rate, active head power S (0 when heads are up) |
| `Pn:X` | X limit switch triggered |
| `Sn:` | **P** paper in tray, **R** rear sensor covered, **F** front sensor covered (omitted when all are off) |
| `Hd:1` / `Hd:2` | Active head (omitted when both are up) |
| `Fn:a,b` | Fan duties in % (omitted when both are off) |
| `Seq:NAME,step` | Running sequence and step number (section 7) |
| `WCO:` / `Ov:` | Work offset and overrides, sent periodically as in Grbl |

---

## 11. Error and alarm codes

Standard Grbl errors are unchanged (1–39, 80, 81, 100). Notable ones:
`8` not idle, `9` locked by alarm, `11` line too long / RX overrun,
`15` jog beyond soft limit, `20` unsupported command.

Errors added for the label cutter:

| Code | Meaning |
|---|---|
| 150 | No paper: a move would turn the feed roller, or `$FEED` was sent with an empty tray |
| 151 | Sensor timeout: the expected sensor edge was not seen within `$303` mm (jam / misfeed) |
| 152 | Paper ran out during feed step 3, before the sheet reached the rear sensor |
| 153 | Feed step 6: the front sensor was not covered (sheet too short or front sensor fault) |
| 154 | Sequence aborted (reset or alarm) |

Alarms: `1` hard limit (X switch), `2` soft limit, `3` reset during motion,
`6` homing reset, `8` homing pull-off failed (switch still pressed),
`9` homing switch not found.

---

## 12. Event-driven architecture

There is no busy loop that polls inputs. Each event source wakes exactly the
code that handles it.

| Source | Mechanism | Action |
|---|---|---|
| Step timing | ESP-IDF `gptimer` alarm ISR (20 MHz, auto-reload) | One Bresenham/AMASS tick; it reprograms its own period per segment. When a segment is consumed it notifies the main task to refill the segment buffer. |
| Paper / rear / front sensors | GPIO any-edge ISR | Sequence watch: captures the machine position and requests a motion cancel. Interlock: feed hold on paper-out. |
| X limit switch | GPIO any-edge ISR | During homing it locks X on the edge; otherwise it raises the hard-limit alarm. |
| UART0 / UART1 | IDF UART driver ISR → event queue → one RX task per port | Realtime characters are executed at once. Line characters are queued and the main task is notified. |
| Head pull-in | `esp_timer` one-shot | Drops the solenoid from pull-in power to hold power. |
| Main task (Arduino `loop`) | Blocks on a FreeRTOS task notification (10 ms safety timeout) | Parses lines, runs the planner and segment generator, and runs the realtime state machine. |

The step ISR, the sensor/limit ISRs, the RX tasks and the main task are all
pinned to core 1, so shared state never races across cores. Realtime flags
use atomic read-modify-write, so an ISR can never lose a flag.

---

## 13. Master-controller integration

Typical job flow over UART1:

```
(boot)                -> Grbl 1.3a ['$' for help] ... [MSG:'$H'|'$X' to unlock]
$H                    -> ok                (X homed)
$FEED                 -> [MSG:Feed complete] ok   (sheet registered, Y = 0)
G0 X10 Y5             -> ok
M3                    -> ok                (head 1 down, after the G0 has finished)
G1 X60 F6000          -> ok
G1 Y40                -> ok
M5                    -> ok
M4 S800               -> ok                (head 2)
G2 X80 Y60 I10 J0     -> ok
M5                    -> ok
$EJECT                -> [MSG:Eject complete] ok  (waits for the job to finish first)
```

Recommendations:

* Poll `?` at 5–10 Hz. Read `Sn:` for paper and sensor state, `Seq:` for
  sequence progress, and the state field for completion.
* Before `$FEED`, check `Sn:` for `P` (or just handle `error:150`).
* On `error:151`/`152`/`153`, the sheet is probably jammed. Show an operator
  message, then try `$EJECT` after the jam is cleared.
* Ctrl-X stops everything immediately (heads up). After a reset during
  motion, the controller is in `ALARM:3`: send `$X` (or `$H`) to continue.

A minimal Python test harness (console or master port):

```python
import serial, time
s = serial.Serial("COM5", 115200, timeout=0.1)
def cmd(line):
    s.write((line + "\n").encode())
    while True:
        r = s.readline().decode().strip()
        if r: print(r)
        if r == "ok" or r.startswith("error") or r.startswith("ALARM"): return r
time.sleep(2); s.reset_input_buffer()
cmd("$X"); cmd("$FEED"); s.write(b"?"); print(s.readline())
```

---

## 14. Commissioning checklist

1. Flash the firmware, open the console (`pio device monitor`) and confirm
   the banner and `[MSG:Using machine:...]`.
2. `$$`: set `$100`–`$102` (steps/mm), `$110`–`$112`, `$120`–`$122` and
   `$130` (X travel).
3. With the motors unpowered, check the inputs with `?`:
   * Press the X switch and look for `Pn:X`.
   * Put paper in the tray and look for `Sn:P`.
   * Cover the rear and front sensors and look for `R` and `F`.
   * Fix any wrong polarity with `$5` / `$305`.
4. `$X`, then jog each axis (`$J=G91 X10 F1000`, `Y10`, `Z10` with paper
   present). Fix wrong directions with `$3`. +Y must move the sheet toward
   the front sensor, and +Z must feed paper in.
5. `$H`: verify homing direction (`$23`) and pull-off (`$27`).
6. Test heads: `M3` / `M5` / `M4` / `M5`. Check that the solenoid force is
   right; adjust `S`, `$307` and `$308` if needed.
7. Run `$EJECT` and `$FEED` with a sheet. Tune `$300` (DR), `$301` (DF) and
   `$302` (speed) until the sheet is registered where you need it. Set `$303`
   a little above the longest sheet travel you expect.

---

## 15. Known limitations

* The firmware has been compiled for the target (Arduino 3.3.9 / IDF 5.5,
  no warnings) but has **not yet been verified on real hardware**. Go
  through the commissioning checklist with the motors unloaded first.
* The sensor ISRs act on the first edge, with no software debounce. Use
  clean sensor signals (section 2).
* The lowest step rate the timer can produce is about 38 steps/s, as in
  Grbl_ESP32. Below that, the axis moves faster than commanded.
* While a long sequence or homing runs, queued input lines wait in the RX
  buffer (status `?`, `!`, `~`, Ctrl-X still work immediately). Do not stream
  more than the buffer can hold without waiting for `ok`.
