#pragma once
/*
  Config.h - Compile-time configuration for the dual-head label cutter
  (Vulcan SC 350 style) running on an ESP32-S3 N8R2.

  Part of Grbl / Grbl_ESP32, reduced to a single machine.
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Everything that is specific to the wiring of the machine lives in this file.
  Run-time tunables (steps/mm, speeds, DR, DF, ...) are $-settings, see
  Settings.cpp and doc/LabelCutter.md.
*/

// ----------------------------------------------------------------------------
// Machine identity
// ----------------------------------------------------------------------------
#define MACHINE_NAME "Dual-head label cutter (ESP32-S3)"

// ----------------------------------------------------------------------------
// Axes.  X = knife carriage, Y = sheet transport, Z = feed roller.
// Only X has a limit switch and soft limits. Y and Z are unbounded.
// ----------------------------------------------------------------------------
#define X_STEP_PIN 4
#define X_DIRECTION_PIN 5
#define Y_STEP_PIN 6
#define Y_DIRECTION_PIN 7
#define Z_STEP_PIN 15  // Feed roller ("feed motor")
#define Z_DIRECTION_PIN 16

// Shared stepper enable, active low. With $4=0 the pin is driven LOW to
// enable the drivers and HIGH to disable them.
#define STEPPERS_DISABLE_PIN 21

// ----------------------------------------------------------------------------
// Inputs. All inputs use the internal pull-up and are active LOW by default
// (":low:pu"). Polarity can be flipped at run time with $5 (limit) and
// $305 (sensors) without recompiling.
// ----------------------------------------------------------------------------
#define X_LIMIT_PIN 8        // Limit/home switch to GND
#define PAPER_SENSOR_PIN 9   // Paper-present IR sensor (feed tray)
#define REAR_SENSOR_PIN 10   // Rear IR sensor (sheet entry side)
#define FRONT_SENSOR_PIN 11  // Front IR sensor (sheet exit side)

// ----------------------------------------------------------------------------
// Outputs (LEDC PWM)
// ----------------------------------------------------------------------------
#define HEAD1_PIN 12  // Head 1 solenoid  (M3)
#define HEAD2_PIN 13  // Head 2 solenoid  (M4) - never on together with head 1
#define FAN1_PIN 14   // Fan 1 (M67/M68 E0)
#define FAN2_PIN 1    // Fan 2 (M67/M68 E1)

#define HEAD_PWM_FREQ 1000      // Hz
#define HEAD_PWM_RES_BITS 12    // 0..4095
#define FAN_PWM_FREQ 25000      // Hz
#define FAN_PWM_RES_BITS 10     // 0..1023 (max for 25 kHz from the 80 MHz APB clock is 11)

// ----------------------------------------------------------------------------
// Serial links. Both ports speak the same Grbl protocol. Replies go to the
// port that sent the command; alarms and [MSG:] lines go to both.
// ----------------------------------------------------------------------------
#define CONSOLE_UART_NUM 0  // USB-serial console
#define CONSOLE_TX_PIN 43
#define CONSOLE_RX_PIN 44
#define MASTER_UART_NUM 1   // Link to the master controller
#define MASTER_TX_PIN 17
#define MASTER_RX_PIN 18
#define BAUD_RATE 115200

// ----------------------------------------------------------------------------
// Core tuning (normally left alone)
// ----------------------------------------------------------------------------
#define LINE_BUFFER_SIZE 256           // Longest accepted input line
#define RX_BUFFER_SIZE 256             // Per-port line buffer reported in Bf:
#define BLOCK_BUFFER_SIZE 16           // Planner look-ahead blocks
#define SEGMENT_BUFFER_SIZE 6          // Step segments (~60 ms of motion)
#define N_STARTUP_LINE 2               // $N0 / $N1
#define ACCELERATION_TICKS_PER_SECOND 100
#define MINIMUM_JUNCTION_SPEED 0.0f    // mm/min
#define MINIMUM_FEED_RATE 1.0f         // mm/min
#define N_ARC_CORRECTION 12
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7f
#define DWELL_TIME_STEP 50             // ms
#define HOMING_LOCATE_CYCLES 1         // Slow re-approach passes after the fast seek
#define HOMING_AXIS_SEARCH_SCALAR 1.1f // Seek distance = 1.1 * $130
#define HOMING_AXIS_LOCATE_SCALAR 5.0f // Locate distance = 5 * pull-off

// Lock G-code (Alarm state) after power-up until $H or $X when homing is enabled.
#define HOMING_INIT_LOCK

// Realtime override limits
#define FEED_OVERRIDE_DEFAULT 100
#define FEED_OVERRIDE_MAX 200
#define FEED_OVERRIDE_MIN 10
#define FEED_OVERRIDE_COARSE 10
#define FEED_OVERRIDE_FINE 1
#define RAPID_OVERRIDE_DEFAULT 100
#define RAPID_OVERRIDE_MEDIUM 50
#define RAPID_OVERRIDE_LOW 25

// Task placement. Everything time critical runs on core 1 so the ISRs, the
// serial task and the main loop never race each other across cores.
#define GRBL_CORE 1
#define SERIAL_TASK_PRIORITY 5
