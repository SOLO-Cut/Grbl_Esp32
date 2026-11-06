# Prompt: Turn an STM32 Grbl port into the dual-head label cutter controller

> Paste everything below this line into the coding agent, together with the
> STM32 Grbl code base it should modify.

---

## Your role and task

You are an embedded firmware engineer. You are given an existing **Grbl port for
STM32** (the code base in this workspace). Modify it into the motion controller
for a **sheet-fed, dual-head label cutter**. The behaviour of the machine is
fully specified below. It already exists as a working firmware on another MCU.
Your job is to reproduce that behaviour exactly on STM32. Do not port the old
code line by line.

The specification is MCU independent. Wherever it says "interrupt", "hardware
timer" or "PWM channel", use the STM32 peripheral that fits best (TIMx, EXTI,
USART, DWT, flash). Rules that apply only to STM32 are in
[section 14](#14-stm32-implementation-requirements).

Primary target MCU: **STM32F411** (Cortex-M4F, 100 MHz, 512 KB flash, 128 KB
RAM). Keep the hardware layer isolated, so another STM32 (F4/G4/H7) only needs a
new pin map and peripheral layer.

### Ground rules

1. **Keep core Grbl intact:** planner (look-ahead, junction deviation), segment
   generator with AMASS, Bresenham step generation, G-code parser, jogging,
   realtime overrides, `$` settings, work coordinate systems, startup lines and
   check mode. Change them only where this spec requires it.
2. **The external protocol must match this spec byte for byte:** commands,
   responses, status report fields, error and alarm numbers, and setting numbers
   with their defaults. A master controller and a PC GUI already depend on it.
3. **Remove** what this machine does not need: spindle and laser classes,
   coolant, probing, safety door, parking, sleep, A/B/C axes, SD card, network
   code, and any extra output or input pins of the base port.
4. **Event driven:** inputs are serviced by interrupts, not by polling loops.
   The main loop sleeps (`__WFI()` or an RTOS wait) until an interrupt or a
   received line wakes it, with a 10 ms safety timeout.
5. All machine-specific wiring lives in **one header** (`Config.h` or the
   board map of the base port). Run-time tunables are `$` settings.
6. The firmware must compile with **no warnings**. Every compile-time
   constraint (pin conflicts, EXTI line conflicts, timer sharing) is checked
   with `static_assert` or `#error`.

---

## 1. Machine overview

| Axis | Mechanism | Travel | Switch |
|---|---|---|---|
| **X** | Knife carriage, across the sheet | 0 … `$130` mm (soft limits) | One limit/home switch |
| **Y** | Sheet transport rollers | Unbounded | none |
| **Z** | Feed roller ("feed motor") that pulls sheets from the tray | Unbounded | none |

* +Y moves the sheet **forward**, from the rear sensor toward the front sensor.
* +Z feeds paper **into** the machine.
* Wrong motor directions are fixed with `$3`, never in G-code.

Other hardware:

* **Two cutting heads** (solenoids, PWM held). They are **mutually exclusive**:
  both may never be energised at the same time.
* **Two fans** (PWM, 0–100 %).
* **Three IR sensors:** paper-present (tray), rear (sheet entry) and front
  (sheet exit).
* **Stepper drivers:** DM542-class drivers with opto-isolated
  PUL / DIR / ENA inputs. **Each driver has its own enable pin.**
* **Two serial links:** one to a master controller and one to a service
  console. Both speak the same protocol.

---

## 2. Signals (logical I/O list)

Assign the pins in the board header. The default polarities are listed below;
each can be flipped at run time where a setting is named.

| Signal | Dir | Electrical default | Run-time polarity |
|---|---|---|---|
| X_STEP, Y_STEP, Z_STEP | out | Pulse active HIGH | `$2` mask |
| X_DIR, Y_DIR, Z_DIR | out | | `$3` mask |
| X_EN, Y_EN, Z_EN | out | **Active LOW**: LOW = driver enabled, HIGH = disabled | `$4` mask (bit0 X, bit1 Y, bit2 Z) |
| X_LIMIT | in, pull-up, interrupt on both edges | Active LOW (switch to GND) | `$5` |
| PAPER_SENSOR | in, pull-up, interrupt on both edges | Active LOW | `$305` bit0 |
| REAR_SENSOR | in, pull-up, interrupt on both edges | Active LOW | `$305` bit1 |
| FRONT_SENSOR | in, pull-up, interrupt on both edges | Active LOW | `$305` bit2 |
| HEAD1, HEAD2 | out, PWM 1 kHz, ≥ 10-bit resolution | High = energised | — |
| FAN1, FAN2 | out, PWM 25 kHz, ≥ 8-bit resolution | High = on | — |
| MASTER_TX/RX | UART | 115200 8N1 | — |
| CONSOLE_TX/RX | UART or USB-CDC | 115200 8N1 | — |

"Active LOW" means the input reads LOW when it is triggered (switch pressed, or
sensor sees paper).

---

## 3. Serial protocol

### 3.1 Ports

* Two independent ports (master and console). Both run the full protocol.
* Each port has its **own 256-byte line buffer**. The maximum line length is
  255 characters. A line ends with `\n`, and `\r\n` is accepted.
* **Routing:**
  * `ok` / `error:N` go **only** to the port that sent the line.
  * `ALARM:N`, `[MSG:...]` and the reset banner go to **both** ports.
  * A `?` status report goes to the port that asked.
* **Overrun:** if a host overruns a port's buffer, the excess bytes are dropped
  and that port gets `error:11`. Receiving must **never block**, so realtime
  characters always get through.
* Flow control is Grbl "send-response". Hosts may also count characters
  against a 128-byte window.

### 3.2 Realtime characters

Handle these inside the receive interrupt. They take effect immediately, even
during homing or a paper sequence, and are never put in the line buffer.

| Byte | Action |
|---|---|
| `?` | Status report (to the requesting port) |
| `!` | Feed hold |
| `~` | Cycle start / resume |
| `0x18` | Soft reset. Heads go up **immediately** (from interrupt context) and motion stops. |
| `0x85` | Jog cancel |
| `0x90`–`0x94` | Feed override: reset to 100 % / +10 / −10 / +1 / −1 (range 10–200 %) |
| `0x95`–`0x97` | Rapid override 100 / 50 / 25 % |

### 3.3 Banner

After every reset, send `\r\nGrbl 1.3a ['$' for help]\r\n` to both ports. With
homing enabled, also send `[MSG:'$H'|'$X' to unlock]`. `$I` returns
`[VER:1.3a.<build>:<machine name>]`.

### 3.4 `$` commands

| Command | Function |
|---|---|
| `$` | Help |
| `$$` | List all settings, one `$n=v` per line |
| `$n` | Print one setting as `$n=v` |
| `$n=v` | Change a setting and store it in non-volatile memory |
| `$#` | Offsets G54–G59, G28, G30, G92, TLO |
| `$G` | Parser state |
| `$I` | Build info |
| `$N`, `$N0=line`, `$N1=line` | Two startup lines, run after reset and after `$H` |
| `$C` | Toggle check mode |
| `$X` | Clear the alarm lock |
| `$H` or `$HX` | Home X |
| `$J=...` | Jog |
| `$FEED` | Paper feed sequence ([section 7](#7-paper-sequences)) |
| `$EJECT` | Paper eject sequence |
| `$RST=$`, `$RST=#`, `$RST=*` | Restore settings / clear offsets / everything plus startup lines |

---

## 4. Axes, homing and limits

### 4.1 Homing (`$H`): X only

* `$23` bit0 = 1 (default): the switch is at the **X = 0** end, and homing seeks toward −X.
* `$23` bit0 = 0: the switch is at the **X = `$130`** end, and homing seeks toward +X.

Sequence:

1. Seek toward the switch at `$25` mm/min for up to 1.1 × `$130` mm (1000 mm if
   `$130` = 0).
2. Back off `$27` mm at `$25`.
3. Re-approach at `$24` mm/min for up to 5 × `$27` mm.
4. Back off `$27` mm again.
5. Wait `$26` ms after each phase (debounce).

Rules:

* During an approach, the **limit interrupt stops X on the exact switch edge**
  by locking the axis's step output. There is no polling.
* If the switch is already pressed when an approach starts, that approach is
  complete immediately.
* Final position:
  * Switch at the 0 end: machine X = `$27`.
  * Switch at the max end: machine X = `$130 − $27`.
* On success, **machine Y and Z are set to 0 without moving**, the state
  returns to Idle (or stays in its previous non-alarm state), and the startup
  lines run.
* Failures:
  * Switch not found within the seek distance → `ALARM:9`.
  * Switch still pressed after a pull-off → `ALARM:8`.
  * Reset during homing → `ALARM:6`.
* **Power-up lock:** with `$22=1` the controller starts in `Alarm` and rejects
  G-code until `$H` or `$X`.

### 4.2 Soft limits (`$20`): X only

* Allowed range: 0 ≤ X ≤ `$130` in **machine** coordinates. `$130 = 0`
  disables the check.
* A G-code move outside the range:
  * if the machine is moving, feed hold first, then wait until Idle;
  * reset and raise `ALARM:2`.
* A jog outside the range is rejected with `error:15`. Nothing moves and there
  is no alarm.
* Setting `$20=1` while `$22=0` returns an error (soft limits need homing).
  Setting `$22=0` forces `$20=0`.
* Y and Z are never limited.

### 4.3 Hard limit (`$21`)

If the X switch triggers outside homing and `$21=1`:

* the heads go up and the steppers stop, all from the limit interrupt;
* `ALARM:1` is raised;
* a reset is required.

---

## 5. G-code

Supported, with standard Grbl behaviour:

* Motion: `G0 G1 G2 G3 G80`
* Planes and modes: `G17 G18 G19`, `G90 G91 G91.1`, `G93 G94`, `G20 G21`
* Non-modal: `G4 Pn` (seconds), `G10 L2/L20`, `G28 G28.1 G30 G30.1`, `G53`,
  `G92 G92.1`
* `G54`–`G59`, `G40`, `G43.1 Z`, `G49`, `G61`
* Program flow: `M0 M1 M2 M30`
* `M6` is accepted and ignored. `T` is tracked only.
* Words `F N S T`

Not supported (reply `error:20`): `G38.x`, `M7 M8 M9`, `M62`–`M65`, spindle
overrides, A/B/C axes.

### 5.1 Heads (in the spindle modal group)

Heads use Grbl's **spindle modal group**, so `M3` and `M4` can never be active
together. A head change happens only **after all queued moves have finished**
(buffer sync), exactly between moves.

| Command | Effect |
|---|---|
| `M3 [S]` | Head 1 down. Head 2 is forced up first. |
| `M4 [S]` | Head 2 down. Head 1 is forced up first. |
| `M5` | Both heads up |
| `S<n>` | Hold power. `S = $30` (default 1000) is 100 % duty. `S` is modal and defaults to `$30` after reset, so a plain `M3` means full power. `S0` selects the head without powering it. |

Head driver rules:

1. **Mutual exclusion in hardware order.** Before any head is energised, stop
   **both** PWM outputs immediately (output forced LOW, not "duty 0 at the next
   period"). Only then start the selected one.
2. **Pull-in (`$307` ms):**
   * If `$307 > 0` and the hold duty is below 100 %, drive the head at 100 %
     for `$307` ms, then drop to the hold duty.
   * Use a one-shot timer. If the head changed in the meantime, the timer
     callback does nothing.
3. **Switch-over delay (`$308` ms):** when switching **directly** from one head
   to the other, wait `$308` ms after releasing the first one.
4. **Emergency kill** must be callable from **any interrupt**: on reset, alarm,
   hard limit or abort, force both head pins LOW at once. Disconnect them from
   the timer, or switch the pins to GPIO output LOW, or use timer break/forced
   inactive. A soft reset re-arms PWM afterwards.
5. Check mode (`$C`) does not move the heads.

### 5.2 Fans

| Command | Effect |
|---|---|
| `M67 E0 Q<0-100>` / `M67 E1 Q<0-100>` | Fan 1 / fan 2 duty in %, applied after queued motion completes |
| `M68 E0 Q<0-100>` / `M68 E1 Q<0-100>` | Same, applied immediately |

`Q` is clamped to 0–100. `E` other than 0 or 1 is an error. Fans switch off on
every reset.

---

## 6. Feed-motor (Z) interlock

The feed roller must **never turn without paper** in the tray. Enforce this at
three independent levels:

1. **Parser.** Reject with `error:150` any G-code or jog line that would
   change Z while the paper sensor reports an empty tray. This covers
   G0/G1/G2/G3 with a Z target different from the current Z, and G28/G30 whose
   stored Z differs. Do not apply it in check mode.
2. **Step interrupt.** On every step tick, clear the Z step bit while the paper
   sensor reports no paper. Read the input register directly. Do not count
   that step in the position.
3. **Paper-out event.** This applies when no paper sequence is running and the
   state is Cycle or Jog:
   * If the paper sensor interrupt sees "no paper" while the executing block
     moves Z, raise **feed hold**. The main loop then reports
     `[MSG:Paper out - feed hold]`.
   * If a block that needs Z starts executing while there is no paper, raise
     feed hold in the same way.
   * A resume (`~`) while the current block needs Z and there is no paper is
     **refused** with `[MSG:No paper - load paper to resume]`.

Inside `$FEED`, the sequence handles paper-out itself ([section 7](#7-paper-sequences)).

---

## 7. Paper sequences

`$FEED` and `$EJECT` are `$` commands. Their reply (`ok` or `error:N`) is sent
when the sequence has **finished**.

### 7.1 Common rules

* If G-code motion is still queued, wait for it to finish first. The sequence
  then requires Idle, otherwise `error:8`.
* At the start: raise both heads (and set the parser's head mode to `M5`),
  then sync the planner and parser positions.
* While a sequence runs, the status report includes `|Seq:FEED,<step>` or
  `|Seq:EJECT,<step>`.
* `!` and `~` pause and resume a sequence. Reset or alarm aborts it with
  `error:154`.
* Sequence moves are normal planner moves:
  * Y and/or Z, relative;
  * the dominant axis moves at `$302` mm/min (paper speed);
  * feed override has no effect on them.

### 7.2 "Move until sensor condition" (the core mechanism)

A sequence step **arms a watch** with up to two conditions. A condition is a
`(mask, wanted)` pair over the 3-bit sensor state (bit0 paper, bit1 rear,
bit2 front), and it is met when `(sensors & mask) == wanted`.

1. **Arm the watch, then evaluate it once immediately.** If a condition is
   already true, the step completes **without moving**, because no edge would
   ever arrive.
2. Otherwise, queue one move of up to `$303` mm in the requested direction and
   start the cycle.
3. **On every sensor edge, the sensor interrupt:**
   * evaluates the armed conditions;
   * on the first match, **copies the exact machine position in steps**;
   * records which condition fired;
   * requests a **motion cancel** (a controlled deceleration with normal
     acceleration, so no steps are lost).

   Arming and evaluating are protected by a critical section.
4. **Backstop:** if the edge arrived before the cycle was actually running,
   the main loop issues the motion cancel once the state is Cycle.
5. When the machine is Idle again:
   * disarm the watch;
   * flush the rest of the move (stepper reset and planner reset, as Grbl's
     probe cycle does);
   * resync the positions.
6. If no condition fired within `$303` mm, the step fails with `error:151`
   (jam or misfeed).
7. Any follow-up distance (DR, DF) is measured **from the captured edge
   position**, not from where the machine stopped. Deceleration overshoot
   therefore never affects registration accuracy.

The sensor interrupts act on the **first edge** with no software debounce. The
hardware provides an RC filter. You may add a short hardware or digital filter,
but it must not delay the captured position by more than a few steps.

### 7.3 `$EJECT`

1. Step 1. If neither the rear nor the front sensor sees paper, send
   `[MSG:No sheet to eject]` and `ok`.
2. Otherwise, move Y forward until **both** rear and front are clear
   (condition `mask = rear|front, want = 0`). Then move a further `$304` mm if
   `$304 > 0`.
3. If `$306=1`, set machine Y and Z to 0.
4. Send `[MSG:Eject complete]` and `ok`.

### 7.4 `$FEED`

| Step | Action | Ends when | Failure |
|---|---|---|---|
| 1 | Eject a sheet still in the machine (same as `$EJECT` step 2, with no message) | rear and front clear | `error:151` |
| 2 | Check the paper-present sensor | — | `error:150` |
| 3 | Z **and** Y forward together, `$303` mm each | **rear sensor covered** (cond 0: `rear, rear`), or the tray runs empty (cond 1: `paper, 0`) | cond 1 fired → `error:152`; nothing fired → `error:151` |
| 4 | Z and Y continue to *rear edge position + `$300`* (DR), each axis measured from its own captured edge position | target reached | — |
| 5 | Z stops; Y forward | **rear sensor clear** (`rear, 0`), meaning the sheet tail has passed | `error:151` |
| 6 | Y **reverse** (−`$303`) | **front sensor clear** (`front, 0`), meaning the lead edge is at the front sensor | `error:153` if the front sensor is not covered when step 6 starts; `error:151` |
| 7 | Y to *front edge position + `$301`* (DF, signed: + forward, − backward). Z stays where it is. | target reached | — |
| 8 | If `$306=1`, set machine Y = 0 and Z = 0. Send `[MSG:Feed complete]` and `ok`. | | |

After a successful `$FEED`, machine **Y = 0 is the registered sheet position**.

During step 4, if the tray runs empty, the step interrupt interlock stops Z by
itself. Y continues, because the sheet is already gripped. This is intended.

---

## 8. Step generation and driver timing

The drivers are DM542-class: opto inputs, minimum pulse about 2.5 µs, DIR
setup ≥ 5 µs before the step edge, ENA ≥ 5 µs before DIR.

* **Step timer:** a hardware timer, 16 or 32 bits, with ≥ 10 MHz resolution.
  Its interrupt runs one Bresenham/AMASS tick and reprograms its own period at
  each new segment. When a segment is consumed, it wakes the main loop to
  refill the segment buffer.
* **Order inside one tick:**
  1. If the direction changed, write the DIR pins, then wait `$309` µs.
  2. **Set all step pins with one register write** (all step pins on one GPIO
     port, written through `BSRR`), so every axis pulses at the same moment.
  3. Calculate the step bits for the next tick and update the position.
  4. Apply the homing axis lock and the Z paper interlock.
  5. End the pulse after exactly `$0` µs.
* **Pulse end:** you may busy-wait on `DWT->CYCCNT` (simple, and how the
  reference firmware does it). Preferred: end the pulse with a second one-shot
  timer, or a timer output-compare in one-pulse mode, so the step interrupt
  does not stall for `$0` µs. Either way the pulse must never be shorter than
  `$0`.
* **Lowest step rate:** if the timer prescaler cannot reach low rates, use
  AMASS and/or a 32-bit timer (TIM2 or TIM5 on the F411). Do not reproduce the
  old limit of about 38 steps/s.
* **Enable pins (one per driver):**
  * The level is `disable XOR $4 bit(axis)`. All three are switched together.
  * Before the pins become outputs, latch them as **disabled**, so the drivers
    never switch on briefly at boot.
  * `st_wake_up` enables them.
  * After motion stops, disable them after `$1` ms (`$1=255` = always enabled).
  * On an alarm, disable them immediately.
  * Never disable them during homing.
* `$0` must be ≥ 2 (`error` for values below 2).

---

## 9. Settings

Store the settings in non-volatile memory (flash emulated EEPROM or external
EEPROM) together with a **layout version**. If the version or size does not
match, restore the defaults and send `[MSG:Settings restored to defaults]`.
Write coordinate offsets and startup lines in the same way. Writes must not
stall the step interrupt: write only when Idle, or place time-critical code
and vectors in RAM while flash is busy.

| $ | Type | Range | Default | Meaning |
|---|---|---|---|---|
| 0 | u8 | 2–100 | 4 | Step pulse, µs |
| 1 | u8 | 0–255 | 250 | Idle delay before drivers are disabled, ms (255 = always on) |
| 2 | mask | 0–7 | 0 | Step pulse invert |
| 3 | mask | 0–7 | 0 | Direction invert |
| 4 | mask | 0–7 | 0 | Enable invert (bit0 X, bit1 Y, bit2 Z; 1 = active-HIGH enable) |
| 5 | bool | | 0 | Invert X limit (0 = active LOW) |
| 10 | u8 | 0–3 | 1 | Status: bit0 = MPos (else WPos), bit1 = add `Bf:` |
| 11 | float | 0–10 | 0.010 | Junction deviation, mm |
| 12 | float | 0.0001–1 | 0.002 | Arc tolerance, mm |
| 13 | bool | | 0 | Report inches |
| 20 | bool | | 1 | Soft limits (X only, needs `$22=1`) |
| 21 | bool | | 1 | Hard limit (X switch) |
| 22 | bool | | 1 | Homing enable (X only; also the power-up lock) |
| 23 | mask | 0–7 | 1 | Homing direction; bit0 = 1 → switch at X = 0 |
| 24 | float | | 200 | Homing locate feed, mm/min |
| 25 | float | | 2000 | Homing seek rate, mm/min |
| 26 | u16 | 0–10000 | 250 | Homing debounce, ms |
| 27 | float | 0–1000 | 2.000 | Homing pull-off, mm |
| 30 | float | | 1000 | S value for 100 % head power |
| 100–102 | float | | 80, 80, 80 | Steps/mm X, Y, Z |
| 110–112 | float | | 12000, 12000, 6000 | Max rate mm/min X, Y, Z |
| 120–122 | float | | 1500, 1500, 800 | Acceleration mm/s² X, Y, Z |
| 130 | float | 0–100000 | 350 | X max travel = soft limit max (0 = off) |
| 300 | float | 0–10000 | 20 | DR: Y+Z travel after the rear-sensor edge, mm |
| 301 | float | −10000–10000 | 10 | DF: Y travel after the front-sensor edge, mm (signed) |
| 302 | float | | 3000 | Sequence paper speed, mm/min |
| 303 | float | | 600 | Max search travel per sensor edge, mm |
| 304 | float | 0–10000 | 0 | Extra Y travel after an eject, mm |
| 305 | mask | 0–7 | 0 | Sensor invert (bit0 paper, bit1 rear, bit2 front; 1 = active HIGH) |
| 306 | bool | | 1 | Zero machine Y/Z after `$FEED` / `$EJECT` |
| 307 | u16 | 0–10000 | 0 | Head pull-in time at 100 %, ms |
| 308 | u16 | 0–10000 | 0 | Head switch-over release delay, ms |
| 309 | u8 | 0–100 | 0 | DIR-to-STEP setup delay, µs |

Out-of-range values return the standard Grbl range error. Remove every setting
of the base port that is not in this table. (Keeping unknown numbers would
confuse the GUI.)

---

## 10. Status report

```
<Idle|MPos:12.000,0.000,0.000|Bf:15,255|FS:0,0|Pn:X|Sn:PRF|Hd:1|Fn:60,0|Seq:FEED,3|WCO:0.000,0.000,0.000|Ov:100,100,100>
```

Emit the fields in this order:

| Field | When | Meaning |
|---|---|---|
| state | always | `Idle`, `Run`, `Hold:0` (complete), `Hold:1` (stopping), `Jog`, `Home`, `Alarm`, `Check` |
| `MPos:` or `WPos:` | always | X, Y, Z (Z = feed roller); `$10` bit0 selects which |
| `Bf:p,r` | `$10` bit1 | Free planner blocks, free bytes in **this port's** RX buffer |
| `Ln:` | if N words are used | Line number of the executing block |
| `FS:f,s` | always | Current feed rate, active head power S (0 when the heads are up) |
| `Pn:X` | X switch triggered | |
| `Sn:` | any sensor active | Letters in this order: **P** paper in tray, **R** rear covered, **F** front covered |
| `Hd:1` / `Hd:2` | a head is down | |
| `Fn:a,b` | any fan > 0 | Fan duties in % (integers) |
| `Seq:NAME,step` | sequence running | `FEED` or `EJECT`, step number from [section 7](#7-paper-sequences) |
| `WCO:` | every 10th report (Idle/Alarm) or 30th (moving) | Work offset |
| `Ov:` | the report after `WCO:`, and on change | Feed, rapid, spindle (always 100) overrides |

---

## 11. Errors and alarms

Keep the standard Grbl errors 1–39 and their numbers. Notable ones: `8` not
idle, `9` locked by alarm, `10` soft limits need homing, `11` line too long or
RX overrun, `15` jog beyond soft limit, `20` unsupported command.

Add these:

| Code | Meaning |
|---|---|
| 150 | No paper: a move would turn the feed roller, or `$FEED` was sent with an empty tray |
| 151 | Sensor timeout: the expected edge was not seen within `$303` mm |
| 152 | Paper ran out in feed step 3, before the rear sensor |
| 153 | Feed step 6: the front sensor was not covered |
| 154 | Sequence aborted (reset or alarm) |

Alarms:

| Alarm | Meaning |
|---|---|
| 1 | Hard limit |
| 2 | Soft limit |
| 3 | Reset during motion (position lost; `$X` or `$H` to continue) |
| 6 | Homing reset |
| 8 | Homing pull-off failed |
| 9 | Homing switch not found |

**Reset behaviour:** a soft reset (Ctrl-X, alarm, abort) does the following:

* kills the heads from interrupt context;
* stops the steppers;
* if the machine was moving, raises `ALARM:3`;
* switches the fans off;
* disarms any sequence watch and clears the sequence name and step;
* restores the default `S` value;
* sends the banner.

---

## 12. Architecture and concurrency

| Event source | Mechanism | Action |
|---|---|---|
| Step timing | Timer interrupt (highest priority) | One step tick; wakes the main loop when a segment is consumed |
| Paper, rear and front sensors | EXTI on both edges | Sequence watch (capture position, motion cancel); paper-out feed hold; wake main loop |
| X limit | EXTI on both edges | Homing: lock X on the edge. Otherwise: hard-limit alarm |
| UART RX (both ports) | Interrupt (+ DMA optional) | Realtime chars executed at once; line chars into the per-port buffer; wake main loop |
| Head pull-in | One-shot timer | Drop the solenoid to hold duty |
| Main loop | Sleeps until woken (10 ms timeout) | Parse lines, planner, segment preparation, realtime state machine, delayed driver disable, `[MSG]` for interrupt events |

* Interrupt priorities: step timer > limit/sensor EXTI > UART RX > other
  timers. Keep the step interrupt short and deterministic.
* Realtime flags shared with interrupts use atomic read-modify-write
  (`__atomic` builtins, or LDREX/STREX through C11 atomics), so an interrupt
  can never lose a flag.
* Messages are never printed from interrupt context. Interrupts set a flag and
  the main loop prints.
* `sys_position[]` (steps, int32) is written only by the step interrupt and by
  code that runs while the steppers are stopped.

---

## 13. Master-controller job flow (must work unchanged)

```
(boot)                -> Grbl 1.3a ['$' for help] ... [MSG:'$H'|'$X' to unlock]
$H                    -> ok                          (X homed, Y=Z=0)
$FEED                 -> [MSG:Feed complete] ok      (sheet registered, Y = 0)
G0 X10 Y5             -> ok
M3                    -> ok                          (head 1 down after the G0 finished)
G1 X60 F6000          -> ok
G1 Y40                -> ok
M5                    -> ok
M4 S800               -> ok                          (head 2 at 80 %)
G2 X80 Y60 I10 J0     -> ok
M5                    -> ok
$EJECT                -> [MSG:Eject complete] ok     (waits for the job first)
```

---

## 14. STM32 implementation requirements

1. **Step and direction pins:** put all three STEP pins on **one GPIO port**
   and set them with one `BSRR` write. Put the DIR pins on one port as well,
   if possible.
2. **EXTI:** STM32 has one EXTI line per pin *number*: PA9 and PB9 cannot both
   interrupt. The four interrupt inputs (X limit and three sensors) must have
   **different pin numbers**. Add a `static_assert`.
3. **PWM timers:** heads (1 kHz) and fans (25 kHz) need **different timers**,
   because all channels of a timer share one frequency. Head kill = force the
   output compare inactive (`OCxM = force inactive`), or switch the pin to GPIO
   output LOW. It must be done with register writes that are safe in an
   interrupt.
4. **Driver outputs (DM542 opto inputs):** use **5 V-tolerant (FT) pins** for
   STEP/DIR/EN and allow **open-drain** mode through a compile-time option
   (`DRIVER_OUTPUTS_OPEN_DRAIN`). The opto `+` inputs can then be tied to 5 V
   (common anode). In open-drain mode the logical polarity of
   `$2`/`$3`/`$4` still behaves the same from the user's point of view. In
   push-pull mode, document that a 5 V buffer (74HCT245/74AHCT125) is
   recommended.
5. **Console port:** a second USART, or USB-CDC on a board with USB. Both ports
   must stay fully independent.
6. **Settings storage:** flash emulated EEPROM in a dedicated sector (on the
   F411 use sector 1, 16 KB, and keep it out of the linker's code area), with
   wear levelling or an append log. Alternatively an I²C EEPROM selected by a
   compile-time option.
7. **Clock:** run at the maximum system clock (100 MHz on the F411). Enable the
   DWT cycle counter at boot.
8. **No dynamic allocation** after start-up.
9. **Watchdog:** an optional IWDG, refreshed by the main loop. Long sequences
   and homing must keep refreshing it.
10. **Output safety at boot:** heads LOW, drivers disabled and fans off
    **before** any other initialisation. A reset or brown-out must never leave
    a solenoid energised.

---

## 15. Deliverables

1. The modified code base, which builds for the STM32F411 (Black Pill or
   similar) with the base port's tool chain (PlatformIO, STM32CubeIDE or
   Makefile; keep whichever the base uses).
2. `Config.h` or a board map with every pin and timer assignment, with the
   `static_assert` checks described above.
3. `doc/LabelCutter_STM32.md`:
   * pin map;
   * wiring notes for DM542 drivers (push-pull + buffer vs open-drain to 5 V);
   * build and flash steps;
   * the full protocol, settings, status, error and sequence reference from
     this spec.
4. A list of every behaviour of the base port you removed or changed that is
   not covered by this spec.

## 16. Acceptance tests (run them on the bench and report the results)

1. Boot → banner, `Alarm` with the unlock message; `$X` → `Idle`.
2. `$$` lists exactly the settings in [section 9](#9-settings) with their
   defaults (after a settings reset).
3. `?` at 10 Hz during a long `G1`: no lost characters, and `Bf:` is
   per-port.
4. Ports are independent: a line sent on the console gets `ok` only on the
   console, and `ALARM` appears on both.
5. Step timing on a scope: pulse width = `$0` µs, DIR leads STEP by ≥ `$309` µs
   after a reversal, all three STEP pins rise together.
6. `$H` with the switch at either end (`$23`): the final X is correct, Y and Z
   are 0, the axis stops on the switch edge, and `ALARM:9` / `ALARM:8` occur on
   a missing or stuck switch.
7. Soft limits: `G0 X-5` after homing → `ALARM:2`; `$J=G91 X-500 F1000` →
   `error:15`.
8. Heads: `M3`, then `M4`, never overlap. Check on a scope that head 1 is LOW
   before head 2 rises, including the `$308` gap. `$307` pull-in is visible.
   Ctrl-X while a head is down → the head drops immediately.
9. Interlock: no paper → `G1 Z10 F1000` gives `error:150`. Pull the paper
   during a Z move → feed hold + `[MSG:Paper out - feed hold]`; `~` is refused
   until paper is back.
10. `$FEED` with a sheet: `Seq:FEED,n` steps 1–8 in the status, and the
    registered position repeats within ±1 step over 20 feeds, independent of
    `$302` speed. Each failure path (150–154) can be produced and gives the
    right code.
11. `$EJECT` with and without a sheet in the machine.
12. Power loss or reset during a move: heads go off and drivers are disabled at
    once; after reboot, settings, offsets and startup lines are intact.
