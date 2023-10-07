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

#include "../gcode.h"

#include "../../MarlinCore.h"   // for marlin_state
#include "../queue.h"           // for flush_and_request_resend

#if HAS_CUTTER
  #include "../../feature/spindle_laser.h"
#endif

/**
 * M999: Restart after being stopped
 *
 * Default behavior is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 */
void GcodeSuite::M999() {
  marlin_state = MF_RUNNING;
  #if ENABLED(TL_L)
    grbl_hold = false;        
    tlStopped = 0;
    //queue.clear();
    quickstop_stepper();    
    TERN_(HAS_CUTTER, cutter.kill()); // Full cutter shutdown including ISR control
    TERN_(HAS_CUTTER, cutter.init());
  #endif
  //ui.reset_alert_level();   //by zyf
  if (parser.boolval('S')) return;
  #if DISABLED(TL_GRBL)
    queue.flush_and_request_resend(queue.ring_buffer.command_port());
  #endif
}
