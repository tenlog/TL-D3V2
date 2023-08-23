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
  if (IS_SD_PRINTING()) planner.synchronize();

  if (card.isMounted()) {
    const uint8_t call_procedure = parser.boolval('P');
    
    card.openFileRead(parser.string_arg, call_procedure);
    
    #if ENABLED(TENLOG_L)
    sprintf(pre_print_file_name, "%s", parser.string_arg);
    #endif

    if (parser.seenval('S')) card.setIndex(parser.value_long());
    card.startFileprint();
    TERN_(TENLOG_TOUCH_LCD, startPrintTime=millis());

    // Procedure calls count as normal print time.
    if (!call_procedure) startOrResumeJob();
  }
}

#if BOTH(TENLOG_TOUCH_LCD, TENLOG_L)
void GcodeSuite::M320() {
  static uint32_t lastClick;
  #if ENABLED(TL_BEEPER)  
  start_beeper(0, 0);
  #endif
  char cmd[256];
  if(millis() - lastClick < 2000) return;
  if(tlStopped) {
    EXECUTE_GCODE("M999");
    safe_delay(200);
    tlStopped = 0;  
  }
  if(!card.isFileOpen()){
    if(strlen(pre_print_file_name)<2){
      card.tl_ls(false);
    }
    if(strlen(pre_print_file_name)>2){
      tlStopped = 0;
      EXECUTE_GCODE("G92 X-3 Y-3");
      safe_delay(100);
      EXECUTE_GCODE("G0 X0 Y0");
      safe_delay(500);
      sprintf(cmd, "M32 !%s", pre_print_file_name);
      ZERO(pre_print_file_name);
      EXECUTE_GCODE(cmd);
      #if ENABLED(TL_BEEPER)
      start_beeper(2, 1); //开始打印
      #endif
    }else{
      #if ENABLED(TL_BEEPER)
      start_beeper(0, 0);
      #endif
    }
  }else{
    #if ENABLED(TL_BEEPER)
    start_beeper(2, 1); //停止打印
    #endif
    tlAbortPrinting();
  }
  lastClick = millis();
}
#endif
#endif // HAS_MEDIA_SUBCALLS
