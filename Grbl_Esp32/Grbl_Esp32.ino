/*
  Grbl_Esp32.ino - Label cutter build of Grbl_ESP32 for the ESP32-S3.

  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.
*/

#include "src/Grbl.h"

void grbl_init();
void run_once();

void setup() {
    grbl_init();
}

void loop() {
    run_once();
}
