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

#if HAS_MEDIA_SUBCALLS
//#include "../../module/endstops.h"
#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../module/planner.h" // for synchronize()

#include "../../MarlinCore.h" // for startOrResumeJob
#if ENABLED(TENLOG_TOUCH_LCD)
  #include "../../lcd/tenlog/tenlog_touch_lcd.h"
#endif

/**
 * M32: Select file and start SD Print
 *
 * Examples:
 *
 *    M32 !PATH/TO/FILE.GCO#      ; Start FILE.GCO
 *    M32 P !PATH/TO/FILE.GCO#    ; Start FILE.GCO as a procedure
 *    M32 S60 !PATH/TO/FILE.GCO#  ; Start FILE.GCO at byte 60
 */
void GcodeSuite::M32() {
  //#if ENABLED(TL_LASER_ONLY)
  //ZERO(pre_print_file_name);
  //#endif
  if (IS_SD_PRINTING()) planner.synchronize();

  if (card.isMounted()) {
    const uint8_t call_procedure = parser.boolval('P');
    
    card.openFileRead(parser.string_arg, call_procedure);
    
    #if ENABLED(TL_LASER_ONLY)
    sprintf(pre_print_file_name, "%s", parser.string_arg);
    #endif

    if (parser.seenval('S')) card.setIndex(parser.value_long());
    card.startFileprint();
    TERN_(TENLOG_TOUCH_LCD, startPrintTime=millis());

    // Procedure calls count as normal print time.
    if (!call_procedure) startOrResumeJob();
  }
}

#if BOTH(TENLOG_TOUCH_LCD, TL_LASER_ONLY)
void GcodeSuite::M320() {
  static uint32_t lastClick;
  start_beeper(0, 0);
  char cmd[256];
  if(millis() - lastClick < 2000) return;
  if(!card.isFileOpen()){
    if(strlen(pre_print_file_name)>2){
      EXECUTE_GCODE("G92 X-12 Y2");
      delay(500);
      sprintf(cmd, "M32 !%s", pre_print_file_name);
      ZERO(pre_print_file_name);
      EXECUTE_GCODE(cmd);
      start_beeper(2, 1);
    }
  }else{
    start_beeper(2, 1);
    tlAbortPrinting();
  }
  lastClick = millis();
}
#endif
#endif // HAS_MEDIA_SUBCALLS
