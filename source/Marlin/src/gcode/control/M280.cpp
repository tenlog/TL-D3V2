/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#ifdef TLTOUCH

#include "../gcode.h"
#include "../../HAL/PWM/pwm.h"
//#include "../../module/servo.h"

/**
 * M280: Get or set servo position. P<index> [S<angle>]
 */
void GcodeSuite::M280() {

  //if (!parser.seen('P')) return;

  const int servo_index = parser.value_int();
  //if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
    if (parser.seen('S')) {
      const int a = parser.value_int();
      if (a == -1){
        //servo[servo_index].detach();
        set_pwm_hw(0, 255, UN_TLT);
      }else{
        set_pwm_hw(a, 255, UN_TLT);
      }
    }
}

#endif // TLTOUCH
