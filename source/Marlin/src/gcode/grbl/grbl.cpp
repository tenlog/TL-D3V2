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

#if ENABLED(TL_GRBL)
#include "../../lcd/tenlog/tenlog_touch_lcd.h"
#include "../gcode.h"
#include "../../module/motion.h"

char message[64]={""};

void GcodeSuite::grbl_a() {
    sprintf(message, "<Idle|MPos:%02f,%02f,%02f|Fs:%d,0>",current_position.x,current_position.y,current_position.z,feedrate_mm_s);
    TLECHO_PRINTLN(message);
}
void GcodeSuite::grbl_d() {
    //char para[30];
    //sprintf(para, "$%s", parser.string_arg);
    //TLDEBUG_PRINTLN(grbl_arg);
    if(grbl_arg[0] == 'I'){
        TLECHO_PRINTLN("[VER:2.0.8.045:]");
        TLECHO_PRINTLN("[OPT:BW$#]");        
        TLECHO_PRINTLN("[MSG:Using machine:TENLOG_L]");        
        TLECHO_PRINTLN("[MSG:No WIFI]");        
        TLECHO_PRINTLN("[MSG:No BT]");        
    }else if(grbl_arg[0] == 'H'){
        EXECUTE_GCODE("M1523");
    }
}
#endif