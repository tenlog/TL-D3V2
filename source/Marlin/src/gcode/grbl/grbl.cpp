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
#include "../../module/planner.h"

void GcodeSuite::grbl_a() {
    grbl_report_status();
    //grbl_1stconnected = true;
}

void GcodeSuite::grbl_dd() {
    char message[128];
    TLECHO_PRINTLN("$0=15");
    TLECHO_PRINTLN("$1=20");
    TLECHO_PRINTLN("$3=0");
    TLECHO_PRINTLN("$4=0");
    TLECHO_PRINTLN("$5=1");
    TLECHO_PRINTLN("$6=0");
    TLECHO_PRINTLN("$10=0");
    TLECHO_PRINTLN("$11=0.01");
    TLECHO_PRINTLN("$12=0.001");
    TLECHO_PRINTLN("$13=0");
    TLECHO_PRINTLN("$20=0");
    TLECHO_PRINTLN("$21=1");
    TLECHO_PRINTLN("$22=1");
    TLECHO_PRINTLN("$23=-1");
    sprintf(message, "$24=%f", homing_feedrate_mm_m.x);
    TLECHO_PRINTLN(message);
    sprintf(message, "$25=%f", homing_feedrate_mm_m.y);
    TLECHO_PRINTLN(message);
    TLECHO_PRINTLN("$26=250");
    sprintf(message, "$27=%d", 5);
    TLECHO_PRINTLN(message);
    TLECHO_PRINTLN("$30=1000");
    TLECHO_PRINTLN("$31=0");
    TLECHO_PRINTLN("$32=1");
    sprintf(message, "$100=%f", planner.settings.axis_steps_per_mm[0]);
    TLECHO_PRINTLN(message);
    sprintf(message, "$101=%f", planner.settings.axis_steps_per_mm[1]);
    TLECHO_PRINTLN(message);
    sprintf(message, "$102=%f", planner.settings.axis_steps_per_mm[2]);
    TLECHO_PRINTLN(message);
    sprintf(message, "$110=%f", planner.settings.max_feedrate_mm_s[0]*49.0);
    TLECHO_PRINTLN(message);
    sprintf(message, "$111=%f", planner.settings.max_feedrate_mm_s[1]*49.0);
    TLECHO_PRINTLN(message);
    sprintf(message, "$112=%f", planner.settings.max_feedrate_mm_s[2]*30.0);
    TLECHO_PRINTLN(message);
    sprintf(message, "$120=%f", planner.settings.acceleration);
    TLECHO_PRINTLN(message);
    sprintf(message, "$121=%f", planner.settings.acceleration);
    TLECHO_PRINTLN(message);
    sprintf(message, "$122=%f", 0);
    TLECHO_PRINTLN(message);
    sprintf(message, "$130=%d", X_MAX_POS);
    TLECHO_PRINTLN(message);
    sprintf(message, "$131=%d", Y_MAX_POS);
    TLECHO_PRINTLN(message);
    sprintf(message, "$132=%d", 0);
    TLECHO_PRINTLN(message);
    TLECHO_PRINTLN("ok");
}
void GcodeSuite::grbl_j() {
    if(grbl_arg[0]=='J' && grbl_arg[1]=='='){
        char cmd[20];
        for(int i=0; i<20; i++){
            cmd[i] = grbl_arg[i+2];
            if(cmd[i] == '\0' || cmd[i]=='\n') break;
        }
        if (cmd[0] == 'G') {
            TLDEBUG_PRINTLN(cmd);
            EXECUTE_GCODE(cmd);
            safe_delay(200);
            if(cmd[0]=='G' && cmd[1]=='9' && cmd[2]=='1'){
                EXECUTE_GCODE("G90");
                safe_delay(100);
            }
            //grbl_hold = false;
        }
    }
}
void GcodeSuite::grbl_d() {
    if(grbl_arg[0] == 'I'){
        char str[128];
        sprintf_P(str, PSTR("[VER:%s.%s]"), SHORT_BUILD_VERSION, TL_SUBVERSION); //VER:2.0.8.038
        TLECHO_PRINTLN(str);
        TLECHO_PRINTLN("[MSG:Using machine:Kentoktool_KT1]");
        sprintf_P(str, PSTR("[MSG:UID:%s]"), tl_hc_sn); //SN
        TLECHO_PRINTLN(str);
        TLECHO_PRINTLN("[MSG:No WIFI]");
        TLECHO_PRINTLN("[MSG:No BT]");        
    }else if(grbl_arg[0] == 'H'){
        EXECUTE_GCODE("M1523");
    }else if(grbl_arg[0] == '?'){
        grbl_a();
    }else if(grbl_arg[0] == 'J'){
        grbl_j();
    }else if(grbl_arg[0] == '$'){   //$$ display settins
        grbl_dd();
    }else if(grbl_arg[0] == '!'){   //$!
        grbl_dd();
    }else if(grbl_arg[0] == 'X'){   //$X
        disable_all_steppers();
    }
}
#endif