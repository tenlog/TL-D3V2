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
#include "../../gcode/gcode.h"
#include "../../gcode/queue.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/settings.h"
#include "../../module/temperature.h"
#include "../../libs/numtostr.h"
#include "../../sd/cardreader.h"
#include "hc32f46x_efm.h"
#include "../../MarlinCore.h"

#include "../../module/endstops.h"


#if ENABLED(HWPWM)
  #include "../../HAL/PWM/pwm.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../feature/runout.h"
#endif

#if ENABLED(ESP8266_WIFI)
  #include "../../HAL/ESP8266_AT/esp8266_wifi.h"
#endif

#if ENABLED(ESP32_WIFI)
  #include "../../HAL/ESP32_SPI/esp32_wifi.h"
#endif

#if ENABLED(TENLOG_TOUCH_LCD)
#include "tenlog_touch_lcd.h"

#define TJC_BAUD    115200
#define DWN_BAUD    115200

unsigned long startPrintTime = 0;

//for eeprom setting...
int8_t tl_languageID = 0;
int8_t tl_Sleep = 0;
int8_t tl_E_FAN_SPEED = 80;
int8_t tl_C_FAN_SPEED = 80;
bool tl_E_FAN_CHANGED = false;
bool tl_C_FAN_CHANGED = false;
int8_t tl_E_FAN_START_TEMP = 80;
int16_t tl_LASER_MAX_VALUE = 1000;
int16_t tl_E_MAX_TEMP = HEATER_0_MAXTEMP;
int8_t tl_ECO_MODE = 0;
int8_t tl_THEME_ID = 0;
int8_t tl_Light = 0;
float tl_Z_HOME_POS = Z_HOME_POS;

int dwnMessageID = -1;
long lLEDTimeoutCount = 0;

bool DWNMoving = false;
bool TLMoving = false;

int tl_print_page_id = 0;
int tl_print_file_id = 0;
int iDWNPageID = 0;

char gsM117[30] = "";
char gsPrinting[30] = "";

long tl_command[256] = {0};
char tl_tjc_ver[10] = "";
bool bLogoGot = false;

int tenlogScreenUpdate;
int iBeepCount = 0;
bool b_PLR_MODULE_Detected = false;
int iMoveRate = 100;
bool bInited = false;
bool bHeatingStop = false;

bool bAtvGot0 = false;
bool bAtvGot1 = false;
bool bAtv = false;
int iOldLogoID = 0;
long lAtvCode = 0;

bool tl_busy_state = false;

bool plr_enabled = false;
int tl_TouchScreenType = 0;

char file_name_list[7][13]={""};
char long_file_name_list[7][27]={""};
char m117_str[15] = {""};
char tl_hc_sn[25] = {""};
char tl_tjc_sn[18] = {""};

bool dwn_is_last_page = false;
bool hotendOffsetChanged = false;

char cmd[128], str_1[32];

int TLPrintingStatus = 0;
bool plr1stZ = false;
bool plr1stE = false;
uint8_t sd_OK = 0;

#if ENABLED(TL_STEPTEST)
    uint32_t STEPTEST_HZ = STEPTEST_HZ_DEFAULT;
#endif

#define TL_LCD_SERIAL LCD_SERIAL
#ifdef PRINT_FROM_Z_HEIGHT
bool PrintFromZHeightFound = true;
float print_from_z_target = 0.0;
#endif

#if ENABLED(TL_BEEPER)
uint8_t beeper_count = 0;
uint8_t beeper_type = 0;
#endif

#if ENABLED(TL_LASER_ONLY)
char pre_print_file_name[13]={""};
#endif

uint32_t wifi_update_interval = 400;

#if ENABLED(TL_LASER_ONLY)
uint32_t last_laser_time = 0;
#endif
uint8_t tl_com_ID = 0;
///////////////// common functions

long ConvertHexLong(long command[], int Len)
{
    long lRet = 0;
    if (Len == command[6] * 2)
    {
        if (Len == 2)
            lRet = 0x100 * command[7] + command[8];
        else if (Len == 4)
        {
            lRet = lRet + 0x100 * command[9] + command[10];
            lRet = lRet + 0x1000000 * command[7] + 0x10000 * command[8];
        }
    }
    return lRet;
}

void TenlogScreen_end()
{
    TL_LCD_SERIAL.end();
}

void get_lcd_command(int ScreenType)
{
    #if DISABLED(TL_NO_SCREEN)
    if(ScreenType == 0)
        ZERO(tl_command);
    else
        NULLZERO(tl_command);

    int i = 0;
    while (MTLSERIAL_available() > 0)
    {
        tl_command[i] = MTLSERIAL_read();
        i++;
        delay(5);
    }

    #endif
}

///////eof common functions

//Detect touch screen..
unsigned long lScreenStart = 0;
int DETECT_TLS(){
    int TryType = 1;
    long lBaud = TJC_BAUD;
    int iTryCount = 1;
    bool bCheckDone = false;
    delay(1000);
    bool CanTry = true;

    while (!bCheckDone)
    {
        if(TryType == 1){
            
            if(CanTry){
                TLDEBUG_PRINTLNPAIR("Trying TJC... ", iTryCount);
                TJC_DELAY;
                TLSTJC_println("sleep=0");
                TenlogScreen_end();
                TenlogScreen_begin(lBaud);
                TJC_DELAY;
                TLSTJC_printEmptyend();
                TJC_DELAY;
                sprintf_P(cmd, PSTR("tStatus.txt=\"Shake hands... %d\""), (iTryCount+1)/2);
                TLSTJC_println(cmd);                
                TLSTJC_println("connect");
                lScreenStart = millis();
                delay(100);
                CanTry = false;
            }

            get_lcd_command(1);

            //char SerialNo[64];
            char ModelNo[64];
            int SLoop = 0;
            int MLoop = 0;
            NULLZERO(tl_tjc_sn);
            NULLZERO(ModelNo);
            int iDHCount = 0;
            for(int i=0; i<100; i++){
                if(tl_command[i] == '\0') break;
                if(tl_command[i] == ',') iDHCount++;
                if(iDHCount > 6) break;
                if(iDHCount > 4 && iDHCount < 6){
                    if((tl_command[i] > 47 && tl_command[i] < 58) || (tl_command[i] > 64 && tl_command[i] < 71)){
                        tl_tjc_sn[SLoop] = tl_command[i];
                        SLoop++;
                    }
                }

                if(iDHCount == 2){
                    ModelNo[MLoop] = tl_command[i];                        
                    if(ModelNo[MLoop] == '_')
                        ModelNo[MLoop] = '\0';
                    if(ModelNo[MLoop] != ',')
                        MLoop++;
                }
            }
            TJC_DELAY;

            if (strlen(tl_tjc_sn) == 16){
                bool Ok16 = true;
                uint8_t total = 0;
                for(uint8_t i=0; i<16; i++){
                    if(tl_tjc_sn[i]>='A' && tl_tjc_sn[i]<='F') total = total + tl_tjc_sn[i] - 'A' + 10;
                    else if(tl_tjc_sn[i]>='0' && tl_tjc_sn[i]<='9') total = total + tl_tjc_sn[i] - '0' + 0;
                    else {
                        Ok16 = false;
                        break;
                    }
                }
                if(Ok16){
                    total = total % 16;
                    sprintf_P(cmd, "loading.sDI.txt=\"%s%X\"", tl_tjc_sn, total);
                    TLSTJC_println(cmd);
                    sprintf_P(cmd, "about.tUID.txt=\"UID:%s\"", tl_hc_sn);
                    TLSTJC_println(cmd);
                    
                    sprintf_P(cmd, "loading.tUI.txt=\"UI %s\"", ModelNo);
                    TLSTJC_println(cmd);

                    uint8_t iLastPageID = TLTJC_GetLastPage();

                    if(iLastPageID == 0){
                        TLSTJC_println("click btA,0");
                        delay(200);
                        get_lcd_command(1);
                        if(tl_command[2] == 'O' && tl_command[3] == 'K'){
                            TLSTJC_println("bkcmd=0");
                            if(lBaud == 9600){
                                delay(50);
                                //TLSTJC_println("bauds=115200");
                                lBaud = 115200;
                                iTryCount--;
                            }else {
                                bCheckDone = true;
                                return 1;
                            }
                        }
                    }else{
                        bCheckDone = true;
                        return 1;
                    }
                }
            }

        }else if(TryType == 0){
            if(CanTry){
                TLDEBUG_PRINTLNPAIR("Trying DWN... ", iTryCount);
                DWN_DELAY;
                TenlogScreen_end();
                delay(50);
                TenlogScreen_begin(DWN_BAUD);
                delay(50);
                sprintf_P(cmd, PSTR("Shake hands... %d"), iTryCount/2);
                DWN_Text(0x7100, 20, cmd);
                delay(50);
                DWN_Get_Ver();
                lScreenStart = millis();
                delay(100);
                CanTry = false;
            }

            get_lcd_command(0);
            if(tl_command[0] == 0x5A && tl_command[1] == 0xA5){
                bCheckDone = true;
                return 0;            
            }
        }
        delay(50);

        if (millis() - lScreenStart > 800){
            CanTry = true;
            //if(TryType == 1) TryType = 0; else TryType = 1;
            TryType = !TryType;
            iTryCount++;
            //SERIAL_ECHOLNPAIR("Tring.. ", lScreenStart);
        }

        if(iTryCount == 9 && TryType == 1){
            if(lBaud == TJC_BAUD){
                lBaud = 9600;
            }
        }
        else if(iTryCount > 10)
        {
            bCheckDone = true;
            return -1;
        }
    }
    return 0;
}


void showDWNLogo(){
    if (iOldLogoID > 99 && iOldLogoID < 111){
        DWN_Data(0x8870, iOldLogoID - 100, 0x02);
    }else
        DWN_Data(0x8870, 0x00, 0x02);
}

void readWriteDWNLogo(int NewID)
{
    if (NewID == 0){
        DWN_NORFData(0x000032, 0x1032, 0x02, false);
        delay(800);
        DWN_RData(0x1032, 0x02);
        delay(100);
    }else{
        DWN_Data(0x1032, NewID, 4);
        delay(50);
        DWN_NORFData(0x000032, 0x1032, 0x02, true);
        iOldLogoID = NewID;
        delay(800);
        showDWNLogo();
    }
}

void tlResetEEPROM(){
    tl_languageID = 0;
    tl_Sleep = 0;
    //tl_FAN2_VALUE = 80;
    //tl_FAN2_START_TEMP = 80;
    tl_E_FAN_SPEED = 80;
    tl_C_FAN_SPEED = 80;
    tl_E_FAN_START_TEMP = 80;
    tl_LASER_MAX_VALUE = 1000;
    if(tl_com_ID==107){
        tl_E_MAX_TEMP = HEATER_0_MAXTEMP_MATRII3D;
    }else{
        tl_E_MAX_TEMP = HEATER_0_MAXTEMP;
    }
    tl_ECO_MODE = 0;
    tl_THEME_ID = 0;
    tl_Light = 0;
    tl_Z_HOME_POS = Z_HOME_POS;
    plr_enabled = false;
}

void TLVersion(){
    if(tl_TouchScreenType == 1){
        sprintf_P(cmd, PSTR("about.tVer.txt=\"%s V%s.%s\""), TL_MODEL_STR, SHORT_BUILD_VERSION, TL_SUBVERSION);
        TLSTJC_println(cmd);
    }else if(tl_TouchScreenType == 0){
        sprintf_P(cmd, PSTR("%s V%s.%s"), TL_MODEL_STR, SHORT_BUILD_VERSION, TL_SUBVERSION);
        DWN_Text(0x7200, 32, cmd);
    }
}

void tlSendSettings(bool only_wifi){
    #define PRINTTJC(a) if(!only_wifi)TLSTJC_println(a)

    if(tl_TouchScreenType == 1)
    {
        //ZERO(wifi_printer_settings);
        PRINTTJC("sleep=0");
        NULLZERO(cmd);
        sprintf_P(cmd, PSTR("main.vLanguageID.val=%d"), tl_languageID);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[0] = tl_languageID;);
        sprintf_P(cmd, PSTR("settings2.nSleep.val=%d"), tl_Sleep);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[1] = tl_Sleep);
        sprintf_P(cmd, PSTR("settings2.cECOMode.val=%d"), tl_ECO_MODE);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[2] = tl_ECO_MODE);
        sprintf_P(cmd, PSTR("main.vThemeID.val=%d"), tl_THEME_ID);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[3] = tl_THEME_ID);
        sprintf_P(cmd, PSTR("settings2.cLight.val=%d"), tl_Light);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[4] = tl_Light);
        sprintf_P(cmd, PSTR("settings.cPLR.val=%d"), plr_enabled);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[37] = plr_enabled);
        sprintf_P(cmd, PSTR("settings2.nEFT.val=%d"), tl_E_FAN_START_TEMP);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[5] = tl_E_FAN_START_TEMP);
        sprintf_P(cmd, PSTR("settings2.nEFS.val=%d"), tl_E_FAN_SPEED);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[6] = tl_E_FAN_SPEED);

        sprintf_P(cmd, PSTR("settings2.nCFS.val=%d"), tl_C_FAN_SPEED);
        PRINTTJC(cmd);

        #if ENABLED(TL_STEPTEST)
        sprintf_P(cmd, PSTR("main.nHZ.val=%d"), STEPTEST_HZ);
        PRINTTJC(cmd);
        #endif

        uint32_t lX = planner.settings.axis_steps_per_mm[X_AXIS] * 100;
        TERN_(ESP32_WIFI, wifi_printer_settings[7] = lX / 0x10000);
        TERN_(ESP32_WIFI, wifi_printer_settings[8] = lX / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[9] = lX % 0x100);
        uint32_t lY = planner.settings.axis_steps_per_mm[Y_AXIS] * 100;
        TERN_(ESP32_WIFI, wifi_printer_settings[10] = lY / 0x10000);
        TERN_(ESP32_WIFI, wifi_printer_settings[11] = lY / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[12] = lY % 0x100);
        uint32_t lZ = planner.settings.axis_steps_per_mm[Z_AXIS] * 100;
        TERN_(ESP32_WIFI, wifi_printer_settings[13] = lZ / 0x10000);
        TERN_(ESP32_WIFI, wifi_printer_settings[14] = lZ / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[15] = lZ % 0x100);
        uint32_t lE = planner.settings.axis_steps_per_mm[E_AXIS] * 100;
        TERN_(ESP32_WIFI, wifi_printer_settings[16] = lE / 0x10000);
        TERN_(ESP32_WIFI, wifi_printer_settings[17] = lE / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[18] = lE % 0x100);

        sprintf_P(cmd, PSTR("settings.xXs.val=%d"), lX);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings.xYs.val=%d"), lY);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings.xZs.val=%d"), lZ);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings.xEs.val=%d"), lE);
        PRINTTJC(cmd);

        #ifdef FILAMENT_RUNOUT_SENSOR
            if(runout.enabled == 0x00){
                PRINTTJC("settings.cFilaSensor.val=0");
            }else{
                PRINTTJC("settings.cFilaSensor.val=1");
            }
            TERN_(ESP32_WIFI, wifi_printer_settings[19] = runout.enabled);
        #endif

        uint32_t lOffsetX = hotend_offset[1].x * 100;
        TERN_(ESP32_WIFI, wifi_printer_settings[20] = lOffsetX / 0x10000);
        TERN_(ESP32_WIFI, wifi_printer_settings[21] = lOffsetX / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[22] = lOffsetX % 0x100);

        uint32_t lOffsetY = hotend_offset[1].y * 100 + 500;
        TERN_(ESP32_WIFI, wifi_printer_settings[23] = lOffsetY / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[24] = lOffsetY % 0x100);

        int32_t lOffsetZ = tl_Z_HOME_POS * -1000;
        uint32_t lOffsetZW = tl_Z_HOME_POS * -1000 + 10000;
        TERN_(ESP32_WIFI, wifi_printer_settings[25] = lOffsetZW / 0x10000);
        TERN_(ESP32_WIFI, wifi_printer_settings[26] = lOffsetZW / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[27] = lOffsetZW % 0x100);

        sprintf_P(cmd, PSTR("settings.xX2.val=%d"), lOffsetX);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings.xY2.val=%d"), lOffsetY);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings.xZOffset.val=%d"), lOffsetZ);
        PRINTTJC(cmd);
        #if ENABLED(HAS_WIFI)
        sprintf_P(cmd, PSTR("wifisetting.vMode.val=%d"), wifi_mode);
        PRINTTJC(cmd);
        //wifi_printer_settings[39] = (http_port)/0x100;
        sprintf_P(cmd, PSTR("wifisetting.tSSID.txt=\"%s\""), wifi_ssid);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("wifisetting.tPwd.txt=\"%s\""), wifi_pswd);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("wifisetting.tAccessCode.txt=\"%s\""), wifi_acce_code);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("wifisetting.tGateway.txt=\"%d.%d.%d.%d\""), wifi_ip_settings[0],wifi_ip_settings[1],wifi_ip_settings[2],wifi_ip_settings[3]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("wifisetting.tSubnet.txt=\"%d.%d.%d.%d\""), wifi_ip_settings[4],wifi_ip_settings[5],wifi_ip_settings[6],wifi_ip_settings[7]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("wifisetting.tLIP.txt=\"%d.%d.%d.%d\""), wifi_ip_settings[8],wifi_ip_settings[9],wifi_ip_settings[10],wifi_ip_settings[11]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("wifisetting.nPort.val=%d"), http_port);
        PRINTTJC(cmd);
        //wifi_printer_settings[37] = (http_port)/0x100;
        //wifi_printer_settings[38] = (http_port)%0x100;
        #endif
        
        #if DISABLED(ELECTROMAGNETIC_VALUE) && DISABLED(TL_LASER_ONLY)
        sprintf_P(cmd, PSTR("main.vTempMax.val=%d"), tl_E_MAX_TEMP - HOTEND_OVERSHOOT);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[28] = (tl_E_MAX_TEMP-HOTEND_OVERSHOOT) / 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[29] = (tl_E_MAX_TEMP-HOTEND_OVERSHOOT) % 0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[30] = (BED_MAXTEMP - BED_OVERSHOOT));
        sprintf_P(cmd, PSTR("main.vBedMax.val=%d"), BED_MAXTEMP - BED_OVERSHOOT);
        PRINTTJC(cmd);
        #endif
        
        sprintf_P(cmd, PSTR("main.vXMax.val=%d"), lOffsetX / 10);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[31] = (lOffsetX/10)/0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[32] = (lOffsetX/10)%0x100);
        sprintf_P(cmd, PSTR("main.vYMax.val=%d"), Y_MAX_POS * 10);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[33] = (Y_MAX_POS*10)/0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[34] = (Y_MAX_POS*10)%0x100);
        sprintf_P(cmd, PSTR("main.vZMax.val=%d"), Z_MAX_POS * 10);
        PRINTTJC(cmd);
        TERN_(ESP32_WIFI, wifi_printer_settings[35] = (Z_MAX_POS*10)/0x100);
        TERN_(ESP32_WIFI, wifi_printer_settings[36] = (Z_MAX_POS*10)%0x100);

        
        sprintf_P(cmd, PSTR("settings1.xM201X.val=%d"), planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM201Y.val=%d"), planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
        PRINTTJC(cmd);        
        sprintf_P(cmd, PSTR("settings1.xM201Z.val=%d"), planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
        PRINTTJC(cmd);        
        sprintf_P(cmd, PSTR("settings1.xM201E.val=%d"), planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
        PRINTTJC(cmd);

        sprintf_P(cmd, PSTR("settings1.xM203X.val=%d"), (uint16_t)planner.settings.max_feedrate_mm_s[X_AXIS]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM203Y.val=%d"), (uint16_t)planner.settings.max_feedrate_mm_s[Y_AXIS]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM203Z.val=%d"), (uint16_t)planner.settings.max_feedrate_mm_s[Z_AXIS]);
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM203E.val=%d"), (uint16_t)planner.settings.max_feedrate_mm_s[E_AXIS]);
        PRINTTJC(cmd); 

        sprintf_P(cmd, PSTR("settings1.xM301P.val=%d"), (uint32_t)(PID_PARAM(Kp, 0) * 100.0f + 0.5f));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM301I.val=%d"), (uint32_t)(unscalePID_i(PID_PARAM(Ki, 0)) * 100.0f + 0.5f));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM301D.val=%d"), (uint32_t)(unscalePID_d(PID_PARAM(Kd, 0)) * 100.0f + 0.5f));
        PRINTTJC(cmd);
        
        sprintf_P(cmd, PSTR("settings1.xM304P.val=%d"), (uint32_t)(thermalManager.temp_bed.pid.Kp * 100.0f + 0.5f));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings1.xM304I.val=%d"), (uint32_t)(unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100.0f + 0.5f));
        PRINTTJC(cmd); 
        sprintf_P(cmd, PSTR("settings1.xM304D.val=%d"), (uint32_t)(unscalePID_d(thermalManager.temp_bed.pid.Kd) * 100.0f + 0.5f));
        PRINTTJC(cmd);
        #if ENABLED(TL_LASER_ONLY)
        sprintf_P(cmd, PSTR("settings2.xM306S.val=%d"), (uint16_t)(tl_LASER_MAX_VALUE));
        #else
        sprintf_P(cmd, PSTR("settings2.xM306S.val=%d"), (uint16_t)(tl_E_MAX_TEMP));
        #endif
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings2.xM204P.val=%d"), (uint16_t)(planner.settings.acceleration));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings2.xM204T.val=%d"), (uint16_t)(planner.settings.travel_acceleration));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings2.xM204R.val=%d"), (uint16_t)(planner.settings.retract_acceleration));
        PRINTTJC(cmd);

        sprintf_P(cmd, PSTR("settings2.xM205X.val=%d"), (uint16_t)(planner.max_jerk.x * 100.0f + 0.5f));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings2.xM205Y.val=%d"), (uint16_t)(planner.max_jerk.y * 100.0f + 0.5f));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings2.xM205Z.val=%d"), (uint16_t)(planner.max_jerk.z * 100.0f + 0.5f));
        PRINTTJC(cmd);
        sprintf_P(cmd, PSTR("settings2.xM205E.val=%d"), (uint16_t)(planner.max_jerk.e * 100.0f + 0.5f));
        PRINTTJC(cmd);
        
    }else if(tl_TouchScreenType == 0){
        DWN_DELAY;
        DWN_Language(tl_languageID);
        DWN_DELAY;
        long lOffsetX = hotend_offset[1].x * 100;
        DWN_Data(0x6018, lOffsetX, 4);
        DWN_DELAY;

        long iSend = 0;
        for (int i = 0; i < 4; i++)
        {
            iSend = long(planner.settings.axis_steps_per_mm[i] * 100.0f);
            DWN_Data(0x6010 + i * 2, iSend, 4);
            DWN_DELAY;
        }

        long lOffsetY = hotend_offset[1].y * 100 + 500;
        long lOffsetZ = hotend_offset[1].z * 100;
        DWN_Data(0x6020, lOffsetY, 2);

        DWN_DELAY;
        DWN_Data(0x6021, lOffsetZ, 2);
        DWN_DELAY;

        /*
        DWN_Data(0x6023, tl_FAN2_VALUE, 2);
        DWN_DELAY;
        DWN_Data(0x6022, tl_FAN2_START_TEMP, 2);
        DWN_DELAY;
        */ //to be done

        DWN_Data(0x8015, 1, 2); //replace auto power off

        DWN_Data(0x8013, tl_ECO_MODE, 2);
        DWN_DELAY;

        #ifdef FILAMENT_RUNOUT_SENSOR
        DWN_Data(0x8014, runout.enabled, 2);
        #endif

        DWN_DELAY;
        iSend = b_PLR_MODULE_Detected + tl_languageID * 2;
        DWN_Data(0x8803, iSend, 2);
        DWN_Data(0x8806, b_PLR_MODULE_Detected, 2);
        delay(200);
    }
    TLVersion();
    //start_beeper(1,1);
}

void TlPageMain(){
    TJC_DELAY;
    if(tl_TouchScreenType == 1){
        TLSTJC_println("page main");
    }
    else if(tl_TouchScreenType == 0){
        DWN_Page(DWN_P_MAIN);
    }
    TJC_DELAY;
}

void TlIsPLR(){
    #if ENABLED(SDSUPPORT)
    if(tl_TouchScreenType == 1){
        TLSTJC_println("sleep=0");
        TLSTJC_println("page main");

        if (strlen(file_name_list[6]) > 3){
            if (!card.isMounted()) card.mount();
            if (card.isMounted()) {
                if(card.fileExists(file_name_list[6])){
                    TLSTJC_println("page printing");
                    TJCMessage(1, 6, 3, "M1003", "M1004", "", long_file_name_list[6]);
                }
            }
        }
    }else if(tl_TouchScreenType == 0){
        lLEDTimeoutCount = 0;
        DWN_Page(DWN_P_MAIN);
        if (strlen(file_name_list[6]) > 3){
            if (!card.isMounted()) card.mount();
            if (card.isMounted()) {
                if(card.fileExists(file_name_list[6])){
                }
            }
        }
    }
    #endif
}

void TJCMessage(const int FromPageID, const int ToPageID, const int MessageID, const char OkValue[], const char CancleValue[], const char StartValue[], const char Message[]){
    TJC_DELAY;
    TLSTJC_println("sleep=0");
    sprintf_P(cmd, PSTR("msgbox.vaFromPageID.val=%d"), FromPageID);
    TLSTJC_println(cmd);
    sprintf_P(cmd, PSTR("msgbox.vaToPageID.val=%d"), ToPageID);
    TLSTJC_println(cmd);
    sprintf_P(cmd, PSTR("msgbox.vaMID.val=%d"), MessageID);
    TLSTJC_println(cmd);
    sprintf_P(cmd, PSTR("msgbox.vtOKValue.txt=\"%s\""), OkValue);
    TLSTJC_println(cmd);
    sprintf_P(cmd, PSTR("msgbox.vtCancelValue.txt=\"%s\""), CancleValue);
    TLSTJC_println(cmd);
    sprintf_P(cmd, PSTR("msgbox.vtStartValue.txt=\"%s\""), StartValue);
    TLSTJC_println(cmd);
    sprintf_P(cmd, PSTR("msgbox.vtMS.txt=\"%s\""), Message);
    TLSTJC_println(cmd);
    TLSTJC_println("page msgbox");
}

void initTLScreen(){    
    stc_efm_unique_id UID_data = EFM_ReadUID();
    //char cmd1[64];
    
    sprintf_P(tl_hc_sn, "%08X%08X%08X", UID_data.uniqueID1, UID_data.uniqueID2, UID_data.uniqueID3);
    TLDEBUG_PRINTLN(tl_hc_sn);

    uint32_t flash_sn_data[5];
    flash_sn_data[0] = 0xFFFEFDFC;
    flash_sn_data[1] = UID_data.uniqueID1;
    flash_sn_data[2] = UID_data.uniqueID2;
    flash_sn_data[3] = UID_data.uniqueID3;
    flash_sn_data[4] = 0xFCFDFEFF;

    TLDEBUG_PRINTLN("Reading Flash...");
    bool canWrite = false;
    for(int i=0; i<5; i++){
        uint32_t flash_sn_data_0 = flash_read_one(FLASH_READ_ADDRESS + 0x04 * i);
        sprintf_P(cmd, "f_hc_sn_%d: 0x%08X", i, flash_sn_data_0);
        TLDEBUG_PRINTLN(cmd);
        if(flash_sn_data_0 != flash_sn_data[i]) canWrite = true;
    }

    if(canWrite){
        TLDEBUG_PRINTLN("Writing Flash...");
        uint32_t * flash_sn_data_w = flash_sn_data;
        flash_write(flash_sn_data_w);
    }

    #if ENABLED(TL_NO_SCREEN)
    tl_TouchScreenType = 1;
    #else
    tl_TouchScreenType = DETECT_TLS(); // check if it is tjc screen
    #endif

    if(tl_TouchScreenType == 1){
	    TLSTJC_printEmptyend();
        delay(100);
        TLSTJC_println("sleep=0");
        delay(100);
        TLDEBUG_PRINTLN("TL TJC Touch Screen Detected!");

    }else if(tl_TouchScreenType == 0){
        sprintf_P(cmd, "SN:%s", tl_hc_sn);
        DWN_Text(0x7280, 28, cmd);
        
        int iLoop = 0;
        bool bLRead = false;
        long lLRead = 0;
        while (!bLogoGot && iLoop < 5)
        {
            if (!bLRead)
            {
                readWriteDWNLogo(0);
                bLRead = true;
                lLRead = millis();
            }
            if (millis() - lLRead > 1000 && bLRead)
            {
                bLRead = false;
                iLoop++;
            }
            get_lcd_command(0);
            process_command_dwn();
            delay(50);
        }

        DWN_Page(DWN_P_LOADING);
        delay(100);        
        TLDEBUG_PRINTLN("TL DWN Touch Screen Detected!");
    }else if(tl_TouchScreenType == -1){
        //tl_TouchScreenType = 1;
        TLSTJC_printEmptyend();
        delay(100);
        TLSTJC_println("sleep=0");
        delay(100);
        kill("Tenlog touch screen not detected! ");
    }
}

void TlLoadingMessage(const char Message[], const int ShowType, const int DelayTime){
    if(tl_TouchScreenType == 1 && (ShowType==1 || ShowType==-1)){
        TLDEBUG_PRINTLN(Message);
        TLSTJC_println("sleep=0");
        TLSTJC_print("tStatus.txt=\"");
        TLSTJC_print(Message);
        TLSTJC_println("\"");
    }else if(tl_TouchScreenType == 0 && (ShowType==0 || ShowType==-1)){
        DWN_Text(0x7100, 20, Message);
    }
    delay(DelayTime);
}

float GCodelng(const char Header, const long FromPostion, long _command[], const bool isFirst=false)
{
    bool bFoundH = false;
    bool bIsEnd = false;
    float fRet = 0.0;
    float fNegative = 1.0;
    float fBs = 1.0f;

    long _tl_command[256];
    for(int i=0; i<256; i++){
        _tl_command[i] = _command[i];
    }

    for (int i = FromPostion; i < 255 && !bIsEnd; i++)
    {
        //0-9 . -
        if((_tl_command[i+1] < 58 && _tl_command[i+1] > 47) || _tl_command[i+1]==45 || _tl_command[i+1]==46)
        {
            if(bFoundH)
            { 
                if(_tl_command[i+1]==46 && fBs == 1.0f) 
                    fBs = 0.1f;
                else if(fBs == 1.0f){
                    fRet = fRet * 10.0f + (float)(_tl_command[i+1] - 48);
                }else if(fBs < 1.0f){
                    fRet = fRet + (float)(_tl_command[i+1] - 48) * fBs;
                    fBs = fBs * 0.1f;
                }
            }
            else if(_tl_command[i+1]==45)
                fNegative = -1.0;
            else
                fRet = (float)(_tl_command[i+1] - 48);
        }
        else if(bFoundH){
            return fRet * fNegative;
        }
        else{
            fRet = 0.0;
        }
        if(_tl_command[i+1] == 0x0A || _tl_command[i+1] == '\0') bIsEnd = true;

        if(isFirst){
            if(_tl_command[i] == Header && i == FromPostion) bFoundH = true;
        }else{
            if(_tl_command[i] == Header && i != FromPostion) {
                bFoundH = true;
                if(bIsEnd) return 0;
            }
        }
    }
    return -999.0;
}

void tlAbortPrinting(){
    #if ENABLED(SDSUPPORT)
    EXECUTE_GCODE("M107");
    #ifdef PRINT_FROM_Z_HEIGHT
    PrintFromZHeightFound = true;
    print_from_z_target = 0.0;
    #endif
    IF_DISABLED(NO_SD_AUTOSTART, card.autofile_cancel());
    card.endFilePrint(TERN_(SD_RESORT, true));

    queue.clear();
    quickstop_stepper();
    print_job_timer.abort();
    EXECUTE_GCODE("M106 S0");
    
    #if ENABLED(TL_LASER_ONLY)
        set_pwm_hw(0, 1000);
        //endstops.enable(true);
        //queue.inject_P(PSTR("G28 X"));
        //queue.enqueue_one_now(PSTR("G4 P10"));
        queue.enqueue_one_now(PSTR("M84"));
        queue.enqueue_one_now(PSTR("G4 P1"));
        queue.enqueue_one_now(PSTR("G4 P1"));
        queue.enqueue_one_now(PSTR("M5"));
        set_pwm_hw(0, 1000);
    #else
        queue.inject_P(PSTR("G28 XY"));
        queue.enqueue_one_now(PSTR("G92 Y0"));
        queue.enqueue_one_now(PSTR("M84"));
    #endif

    IF_DISABLED(SD_ABORT_NO_COOLDOWN, thermalManager.disable_all_heaters());
    EXECUTE_GCODE("M107");

    wait_for_heatup = false;
    TERN_(POWER_LOSS_RECOVERY_TL, if(plr_enabled) settings.plr_reset());
    disable_all_steppers();
    TLPrintingStatus = 0;

    TlPageMain();
		
	#if HAS_FAN
    thermalManager.setTargetHotend(0, 0);
    thermalManager.setTargetHotend(0, 1);
    thermalManager.setTargetBed(0);
	#endif
    #endif//SDSUPPORT
}

void TLFilamentRunout(){
    queue.inject_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
    my_sleep(1.5);
    //EXECUTE_GCODE("M25");
    //while(queue.has_commands_queued() || planner.movesplanned() > 1){
    //    idle();
    //}
    if(tl_TouchScreenType == 1){
        iBeepCount = 10;
        TLSTJC_println("main.vCC.val=1");
        //EXECUTE_GCODE("M25");
        my_sleep(1.5);
        TJCMessage(15, 15, 6, "G28 X", "G28 X", "M1031 O1", "");
    }
}

void SetBusyMoving(bool Busy){
    if(Busy){
        TLMoving = true;
        TLSTJC_println("main.vCC.val=1");
    }else{
        while(tl_busy_state){
            my_sleep(0.2);                
        }
        TLMoving = false;
        TLSTJC_println("main.vCC.val=0");
    }
	#if ENABLED(HAS_WIFI)
    if(wifi_connected)
        tenlog_status_update(false);
	#endif
}

int feed_rate_Pause = 0;
float zPos_Pause = 0.0;
float ePos_Pause = 0.0;

//M1031
void TJCPauseResumePrinting(bool PR, int FromPos){
    #ifdef PRINT_FROM_Z_HEIGHT
    if (!PrintFromZHeightFound)
        return;
    #endif
    
    long lO = GCodelng('O', 0, tl_command);
    if(PR) {
        //Pause
        if(lO == -999){
            //click pause from lcd
            EXECUTE_GCODE("M25");
            my_sleep(2.0);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            SetBusyMoving(true);
            TJC_DELAY;
        } 
        if(lO == 0 || lO == 1 || lO == -999){

            bool FilaRunout = false;
            if(lO == 1) FilaRunout = true;  //runout from lcd message box
            
            zPos_Pause = current_position[Z_AXIS];
            ePos_Pause = current_position[E_AXIS];
            feed_rate_Pause = feedrate_mm_s;

            TLSTJC_println("sleep=0");
            TLSTJC_println("reload.vaFromPageID.val=6");
            sprintf_P(cmd, PSTR("reload.sT1T2.txt=\"%d\""), active_extruder + 1);
            TLSTJC_println(cmd);
			#ifndef ELECTROMAGNETIC_VALUE
            sprintf_P(cmd, PSTR("reload.vaTargetTemp0.val=%d"), int(thermalManager.degTargetHotend(0) + 0.5f));
            TLSTJC_println(cmd);
            sprintf_P(cmd, PSTR("reload.vaTargetTemp1.val=%d"), int(thermalManager.degTargetHotend(1) + 0.5f));
            TLSTJC_println(cmd);
            sprintf_P(cmd, PSTR("reload.vaTargetBed.val=%d"), int(thermalManager.degTargetBed() + 0.5f));
            TLSTJC_println(cmd);
			#endif //!ELECTROMAGNETIC_VALUE
            #if ENABLED(DUAL_X_CARRIAGE)
            sprintf_P(cmd, PSTR("reload.vaMode.val=%d"), dual_x_carriage_mode);            
            TLSTJC_println(cmd);
            if (duplicate_extruder_x_offset != DEFAULT_DUPLICATION_X_OFFSET) {
                sprintf_P(cmd, PSTR("reload.vaMode2Offset.val=%d"), duplicate_extruder_x_offset);
                TLSTJC_println(cmd);
            } else{
                TLSTJC_println("reload.vaMode2Offset.val=-1");
            }
            #endif //DUAL_X_CARRIAGE

            if(FilaRunout){
                //planner.syn
				#if HAS_FAN
                thermalManager.setTargetHotend(0,0);
                thermalManager.setTargetHotend(0,1);
				#endif
            }            

            my_sleep(3.0);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.1);
            delay(100);
            float RaiseZ = zPos_Pause+30;                       
            if(RaiseZ > Z_LENGTH) RaiseZ = Z_LENGTH;
            sprintf_P(cmd, PSTR("G0 F6000 Z%f"), RaiseZ);
            queue.inject(cmd);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            my_sleep(0.2);
            delay(100);
            //queue.inject("G27");
            SetBusyMoving(false);
        }
    }else{
        //Resume
        TLPrintingStatus = 2;

        SetBusyMoving(true);
        delay(50);

        long lT = GCodelng('T', FromPos, tl_command);  //M1032 T0 H200 I0
        long lH = GCodelng('H', FromPos, tl_command);  //Temp 0 
        long lI = GCodelng('I', FromPos, tl_command);  //Temp 1
        sprintf_P(cmd, PSTR("M1032 T%i H%i I%i"), lT, lH, lI);
        TLDEBUG_PRINTLN(cmd);
        
        #if ENABLED(DUAL_X_CARRIAGE)
        if(dual_x_carriage_mode == DXC_DUPLICATION_MODE || DXC_MIRRORED_MODE == 3){
            EXECUTE_GCODE("G28 X");
            delay(100);
            my_sleep(2.5);
        }
        #endif

        if(lT == 0){
            if(lI > 0){
                sprintf_P(cmd, PSTR("M104 T1 S%i"), lI);
                EXECUTE_GCODE(cmd);
                delay(100);
            }
            EXECUTE_GCODE(PSTR("T0"));
            delay(100);
            if(lH > 0){
                sprintf_P(cmd, PSTR("M109 S%i"), lH);
                EXECUTE_GCODE(cmd);
                delay(100);
        }    
        }
        if(lT == 1){
            if(lH > 0){
                sprintf_P(cmd, PSTR("M104 T0 S%i"), lH);
                EXECUTE_GCODE(cmd);
                delay(100);
            }
            delay(100);
            EXECUTE_GCODE(PSTR("T1"));
            delay(100);
            if(lI > 0){
                sprintf_P(cmd, PSTR("M109 S%i"), lI);
                EXECUTE_GCODE(cmd);
                TLDEBUG_PRINTLN(cmd);
                delay(100);
            }
        }

        if(zPos_Pause > 0.0f){
            sprintf_P(cmd, PSTR("G1 Z%f F4500"), zPos_Pause);
            EXECUTE_GCODE(cmd);
            delay(100);
            //TLDEBUG_PRINTLN(cmd);
        }

        if(ePos_Pause > 0.0f){
            sprintf_P(cmd, PSTR("G92 E%f"), ePos_Pause);
            EXECUTE_GCODE(cmd);
            //TLDEBUG_PRINTLN(cmd);
            feedrate_mm_s = feed_rate_Pause;
            delay(100);
        }

        runout.reset();

        EXECUTE_GCODE(PSTR("M24"));
        //TLDEBUG_PRINTLN("M24");
        delay(100);
        
        SetBusyMoving(false);
        TLPrintingStatus = 1;
    }
}

void TLSDPrintFinished(){    
     
    #ifdef PRINT_FROM_Z_HEIGHT
    PrintFromZHeightFound = true;
    print_from_z_target = 0.0;
    #endif

    TERN_(POWER_LOSS_RECOVERY_TL, if(plr_enabled) settings.plr_reset());
    #if ENABLED(HWPWM)
    set_pwm_hw(0, 1000);
    #endif
    my_sleep(1.5);

    unsigned long stoptime = millis();
    long t = (stoptime - startPrintTime) / 1000;
    int hours, minutes;
    minutes = (t / 60) % 60;
    hours = t / 60 / 60;
 
    TLPrintingStatus = 0;

    tenlog_status_update(true);

    if(tl_TouchScreenType == 1){
        char time[10];
        sprintf_P(time, "%02d:%02d", hours, minutes);
        TJCMessage(1, 1, 1, "", "", "", time);
    }else if(tl_TouchScreenType == 0){

    }
    #if ENABLED(TL_BEEPER)
    start_beeper(16, 1);
    #endif
}

void load_filament(int LoadUnload, int TValue)
{    
    int iTempE = active_extruder;
    if (TValue != iTempE && TValue != -1){
        sprintf_P(cmd, PSTR("T%i"), TValue);
        EXECUTE_GCODE(cmd);
    }
    if (LoadUnload == 0){
        //Load
        float fEPos = current_position[E_AXIS];
        fEPos += 90;
        sprintf_P(cmd, PSTR("G1 E%f F400"), fEPos);
        EXECUTE_GCODE(cmd);
        delay(50);
        fEPos += 20;
        sprintf_P(cmd, PSTR("G1 E%f F200"), fEPos);
        EXECUTE_GCODE(cmd);
    }
    else if (LoadUnload == 1){
        //unload
        float fEPos = current_position[E_AXIS];
        fEPos += 30;
        sprintf_P(cmd, PSTR("G1 E%f F400"), fEPos);
        EXECUTE_GCODE(cmd);
        fEPos -= 120;
        sprintf_P(cmd, PSTR("G1 E%f F3000"), fEPos);
        EXECUTE_GCODE(cmd);        
    }
}

void process_command_dwn()
{
    /*
    char str_1[16], cmd[20], sC[10];
    if (tl_command[0] == 0x5A && tl_command[1] == 0xA5)
    { //Header Good
        if(lLEDTimeoutCount >= DWN_LED_TIMEOUT)
        {
            tl_command[0] = 0;
            tl_command[1] = 0;
            lLEDTimeoutCount = 0;
            DWN_LED(DWN_LED_ON);
            return;            
        }
        if (tl_command[2] == 0x03 && tl_command[3] == 0x82 && tl_command[4] == 0x4F && tl_command[5] == 0x4B)
        {
            bAtv = false;
            DWN_Page(0);
        }
        else if (tl_command[3] == 0x83)
        { 
            if (tl_command[4] == 0x10 && tl_command[5] == 0x32)
            {
                //get logo id
                bLogoGot = true;
                iOldLogoID = ConvertHexLong(tl_command, 4);
                showDWNLogo();
                delay(50);
            }
            else if (tl_command[4] == 0x60)
            {
                long lData = ConvertHexLong(tl_command, 2);
                switch ((int)tl_command[5])
                {
                case 0x00:
                case 0x02:
                    thermalManager.setTargetHotend(lData, tl_command[5] * 0.5);
                    break;
                case 0x04:
                    thermalManager.setTargetBed(lData);
                    break;
                case 0x06:
                {
                    feedrate_mm_s = 50;
                    float fX = (float)lData / 10.0f;
                    sprintf_P(cmd, PSTR("G0 X%s"), dtostrf(fX, 1, 3, str_1));
                    EXECUTE_GCODE(cmd);
                }
                    break;
                case 0x07:
                {
                    feedrate_mm_s = 50;
                    float fY = (float)lData / 10.0f;
                    sprintf_P(cmd, PSTR("G0 Y%s"), dtostrf(fY, 1, 3, str_1));
                    EXECUTE_GCODE(cmd);
                }
                    break;
                case 0x08:
                {
                    feedrate_mm_s = 5;
                    float fZ = (float)lData / 10.0f;
                    sprintf_P(cmd, PSTR("G0 Z%s"), dtostrf(fZ, 1, 3, str_1));
                    EXECUTE_GCODE(cmd);
                }
                    break;
                case 0x10:
                case 0x12:
                case 0x14:
                case 0x16:
                    lData = ConvertHexLong(tl_command, 4);
                    planner.settings.axis_steps_per_mm[((int)tl_command[5] - 0x10) / 2] = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    break;
                case 0x18:
                    lData = ConvertHexLong(tl_command, 4);
                    hotend_offset[1].x = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    break;
                case 0x20:                    
                    lData = ConvertHexLong(tl_command, 4);
                    hotend_offset[1].y = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    break;
                case 0x21:
                    lData = ConvertHexLong(tl_command, 4);
                    hotend_offset[1].z = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    break;
                case 0x23:
                    
                    //tl_FAN2_VALUE = lData;
                    //if (tl_FAN2_VALUE > 100)
                    //    tl_FAN2_VALUE = 80;
                    //EXECUTE_GCODE("M500");
                    
                    break;
                case 0x22:
                    
                    //tl_FAN2_START_TEMP = lData;
                    //EXECUTE_GCODE("M500");
                    
                    break;
                case 0x52:
                    {
                    long lS = lData;
                    char sS[10];
                    NULLZERO(sS);
                    sprintf_P(sS, PSTR("S%d "), lS);
                    sprintf_P(cmd, PSTR("M220 %s"), sS);
                    EXECUTE_GCODE(cmd);
                    }
                    break;
                case 0x41:
                    #ifdef PRINT_FROM_Z_HEIGHT
                    print_from_z_target = (float)lData / 10.0f;
                    #endif
                    break;
                case 0x0A:
                {
                    int iFan = (int)((float)lData / 100.0f * 256.0f + 0.5f);
                    if (iFan > 255)
                        iFan = 255;
                    char sS[10];
                    NULLZERO(sS);
                    long lS = iFan;
                    sprintf_P(sS, PSTR("S%d "), lS);
                    sprintf_P(cmd, PSTR("M106 %s"), sS);
                    EXECUTE_GCODE(cmd);
                }
                    break;
                case 0x70:
                    if (lData != iOldLogoID)
                    {
                        readWriteDWNLogo(lData);
                    }
                    break;
                case 0x66:
                    lData = ConvertHexLong(tl_command, 4);
                    lAtvCode = lData;
                    break;
                }
            }
            else if (tl_command[4] == 0x50 && tl_command[5] == 00)
            {
                long lData = ConvertHexLong(tl_command, 2);
                float fTemp = 0.0;
                switch (lData)
                {
                case 0x07:
                case 0x75:
                    #if HAS_FAN
                    if (thermalManager.fan_speed[0] > 0)
                        EXECUTE_GCODE("M107");
                    else
                        EXECUTE_GCODE("M106 S255");
                    #endif
                    break;
                case 0x02:
                case 0x17:
                case 0x87:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if (active_extruder == 0)
                        EXECUTE_GCODE("T1");
                    else
                        EXECUTE_GCODE("T0");
                    DWNMoving = false;
                   break;
                case 0x11:
                case 0x35:
                case 0x84:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    EXECUTE_GCODE("G4 P0.01");
                    EXECUTE_GCODE("G28 XYZ");
                    DWNMoving = false;
                    break;
                case 0x91:
                case 0x92:
                case 0x93:
                    sprintf_P(cmd, PSTR("M605 S%d"), lData-0x90);                    
                    EXECUTE_GCODE(cmd);
                    break;
                case 0x12:
                    EXECUTE_GCODE("M605 S1");
                    break;
                case 0x74:
                case 0x72:
                case 0x73:
                    {
                        long lS = lData - 0x72;
                        if(lS == 0){
                            //Pre heat ABS
                            thermalManager.setTargetHotend(PREHEAT_2_TEMP_HOTEND, 0);
                            thermalManager.setTargetHotend(PREHEAT_2_TEMP_HOTEND, 1);
                            thermalManager.setTargetBed(PREHEAT_2_TEMP_BED);
                        }else if(lS == 1){
                            //Pre heat PLA
                            thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 0);
                            thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 1);
                            thermalManager.setTargetBed(PREHEAT_1_TEMP_BED);
                        }else if(lS == 2){
                            //cooling...
                            thermalManager.setTargetHotend(0, 0);
                            thermalManager.setTargetHotend(0, 1);
                            thermalManager.setTargetBed(0);
                            disable_all_steppers();
                        }            
                    }
                    break;
                case 0x82:
                case 0x83:
                case 0x85:
                case 0x86:
                    {
                        if(DWNMoving) break;

                        DWNMoving = true;

                        long lX = 0;
                        long lY = 0;
                        if(lData == 0x82){
                            lX = 32;
                            lY = Y_MAX_POS - 53;
                        }else if(lData == 0x83){
                            lX = X2_MAX_POS - 81;
                            lY = Y_MAX_POS - 53;
                        }else if(lData == 0x85){
                            lX = 32;
                            lY = 25;
                        }else if(lData == 0x86){
                            lX = X2_MAX_POS - 81;
                            lY = 25;
                        }

                        EXECUTE_GCODE("G1 F4000");
                        my_sleep(0.2);
                        EXECUTE_GCODE("G1 Z5.0 F4000");
                        my_sleep(0.2);
                        sprintf_P(cmd, PSTR("G1 X%d Y%d"), lX, lY);
                        EXECUTE_GCODE(cmd);
                        my_sleep(0.2);
                        EXECUTE_GCODE("G1 Z0");
                        my_sleep(0.2);
                        DWNMoving = false;
                    }
                    break;
                case 0xC1://Move rate
                    iMoveRate = 1;
                    break;
                case 0xC2:
                    iMoveRate = 10;
                    break;
                case 0xC3:
                    iMoveRate = 100;
                    break;
                case 0x34:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 1)
                        EXECUTE_GCODE("T0");
                    fTemp = (float)iMoveRate / 10.0f + current_position[X_AXIS];
                    sprintf_P(sC, PSTR("X%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x33:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 1)
                        EXECUTE_GCODE("T0");
                    fTemp = (-1)*(float)iMoveRate / 10.0f + current_position[X_AXIS];
                    sprintf_P(sC, PSTR("X%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x31:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    fTemp = (-1)*(float)iMoveRate / 10.0f + current_position[Y_AXIS];
                    sprintf_P(sC, PSTR("Y%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x32:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    fTemp = (float)iMoveRate / 10.0f + current_position[Y_AXIS];
                    sprintf_P(sC, PSTR("Y%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x37:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    fTemp = (float)iMoveRate / 10.0f + current_position[Z_AXIS];
                    sprintf_P(sC, PSTR("Z%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x38:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    fTemp = (-1)*(float)iMoveRate / 10.0f + current_position[Z_AXIS];
                    sprintf_P(sC, PSTR("Z%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x39:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 0)
                        EXECUTE_GCODE("T1");
                    fTemp = (-1)*(float)iMoveRate / 10.0f + current_position[E_AXIS];
                    sprintf_P(sC, PSTR("E%s "), dtostrf(fTemp, 1, 1, str_1));
                    sprintf_P(cmd, PSTR("G1 %s F3000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x3A:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 0)
                        EXECUTE_GCODE("T1");
                    fTemp = (float)iMoveRate / 10.0f + current_position[E_AXIS];
                    sprintf_P(sC, PSTR("E%s "), dtostrf(fTemp, 1, 1, str_1));
                    sprintf_P(cmd, PSTR("G1 %s F3000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x3E:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 1)
                        EXECUTE_GCODE("T0");
                    fTemp = -1*(float)iMoveRate / 10.0f + current_position[E_AXIS];
                    sprintf_P(sC, PSTR("E%s "), dtostrf(fTemp, 1, 1, str_1));
                    sprintf_P(cmd, PSTR("G1 %s F3000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x3F:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 1)
                        EXECUTE_GCODE("T0");
                    fTemp = (float)iMoveRate / 10.0f + current_position[E_AXIS];
                    sprintf_P(sC, PSTR("E%s "), dtostrf(fTemp, 1, 1, str_1));
                    sprintf_P(cmd, PSTR("G1 %s F3000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x3B:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 0)
                        EXECUTE_GCODE("T1");
                    fTemp = -1*(float)iMoveRate / 10.0f + current_position[X_AXIS];
                    sprintf_P(sC, PSTR("X%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x3D:
                    if(DWNMoving) break;
                    DWNMoving = true;
                    if(active_extruder == 0)
                        EXECUTE_GCODE("T1");
                    fTemp = (float)iMoveRate / 10.0f + current_position[X_AXIS];
                    sprintf_P(sC, PSTR("X%s "), dtostrf(fTemp, 1, 1, str_1));                
                    sprintf_P(cmd, PSTR("G1 %s F6000"), sC);
                    EXECUTE_GCODE(cmd);
                    DWNMoving = false;
                    break;
                case 0x18:
                    thermalManager.setTargetHotend(0, 0);
                    thermalManager.setTargetHotend(0, 1);
                    thermalManager.setTargetBed(0);
                    disable_all_steppers();
                    break;
                case 0x29:
                case 0x2A:
                case 0x2B:
                case 0x2C:
                    tl_languageID = lData - 0x29 + 3;
                case 0x21:
                case 0x22:
                case 0x20:
                    if (lData == 0x20)
                        tl_languageID = 2;
                    else if (lData == 0x22 || lData == 0x21)
                        tl_languageID = lData - 0x21;

                    DWN_Data(0x8801, (dual_x_carriage_mode - 1) + tl_languageID * 3, 2);
                    DWN_Data(0x8803, b_PLR_MODULE_Detected + tl_languageID * 2, 2);
                    DWN_Language(tl_languageID);
                    EXECUTE_GCODE(PSTR("M500"));
                    break;
                case 0x25:
                    
                    //it is back from settings page. not in use any more
                    //if (!card.isFileOpen())
                    //{
                    //    DWN_Page(DWN_P_MAIN);
                    //}
                    //else
                    //{
                    //    DWN_Page(DWN_P_PRINTING);
                    //}
                    
                    break;
#ifdef HAS_PLR_MODULE
                case 0x23:
                    if (b_PLR_MODULE_Detected)
                    {
                        tl_AUTO_OFF = !tl_AUTO_OFF;
                        DWN_Data(0x8012, tl_AUTO_OFF, 2);
                        EXECUTE_GCODE(PSTR("M500"));
                    }
                    break;
#endif
#ifdef FILAMENT_RUNOUT_SENSOR
                case 0x28:
                    runout.enabled = !runout.enabled;
                    DWN_Data(0x8014, runout.enabled, 2);
                    EXECUTE_GCODE(PSTR("M500"));
                    break;
#endif
                case 0x27:
                    tl_ECO_MODE = !tl_ECO_MODE;
                    DWN_Data(0x8013, tl_ECO_MODE, 2);
                    EXECUTE_GCODE(PSTR("M500"));
                    break;
                case 0x24:
                    DWN_Message(MSG_RESET_DEFALT, "", false); //Reset
                    break;
                case 0xD1:
                    DWN_MessageHandler(true); //Msg OK
                    dwnMessageID = -1;
                    break;
                case 0xD2:
                    DWN_MessageHandler(false); //Msg Cancel
                    dwnMessageID = -1;
                    break;                    
                case 0xA6:
                    
                    //in page reload, not in use any more
                    //if (degTargetHotend(0) > 50)
                    //    command_M104(0, 0);
                    //else
                    //    command_M104(0, pause_T0T);
                    
                    break;
                case 0xA7:
                    
                    //if (degTargetHotend(0) > 50)
                    //    command_M104(1, 0);
                    //else
                    //    command_M104(1, pause_T1T);
                    
                    break;
                case 0xA1:
                    
                    //back from relaod page
                    //if (card.isFileOpen())
                    //{
                    //    DWN_Page(DWN_P_PRINTING);
                    //    if (pause_T0T1 == 1)
                    //        enquecommand_P(PSTR("T1"));
                    //    else
                    //        enquecommand_P(PSTR("T0"));

                    //    _delay_ms(100);

                    //    if (pause_BedT > 0)
                    //    {
                    //        String strCommand = "M140 S" + String(pause_BedT);
                    //        char *_Command = strCommand.c_str();
                    //        enquecommand(_Command);
                    //    }
                    //}
                    //else
                    //    DWN_Page(DWN_P_TOOLS);
                    
                    break;
                case 0xA2:
                    load_filament(0, 0);
                    break;
                case 0xA3:
                    load_filament(0, 1);
                    break;
                case 0xA4:
                    load_filament(1, 0);
                    break;
                case 0xA5:
                    load_filament(1, 1);
                    break;
                case 0x05:
                    EXECUTE_GCODE("M21");
                    delay(100);                
                    card.tl_ls();
                    break;
                case 0x58:
                    if (tl_print_page_id > 0)
                    {
                        tl_print_page_id--;
                        card.tl_ls();
                    }
                    break;
                case 0x59:
                    if (!dwn_is_last_page)
                    {
                        tl_print_page_id++;
                        card.tl_ls();
                    }
                    break;
                case 0x57:
                    #ifdef PRINT_FROM_Z_HEIGHT
                    if (print_from_z_target)
                        DWN_Page(DWN_P_TOOLS);
                    else
                    {
                        DWN_Page(DWN_P_MAIN);
                    }
                    #endif
                    break;
                case 0x51:
                case 0x52:
                case 0x53:
                case 0x54:
                case 0x55:
                case 0x56:
                    {
                        iPrintID = lData - 0x51;                    
                        if (long_file_name_list[iPrintID][0] != '\0'){
                            sprintf_P(cmd, "%s?", long_file_name_list[iPrintID]);
                            DWN_Message(MSG_START_PRINT_TL, cmd, false);
                        }
                    }
                    break;
                #ifdef PRINT_FROM_Z_HEIGHT
                case 0xB1:
                    print_from_z_target = 0.0;
                    DWN_Page(DWN_P_TOOLS);
                    break;
                case 0xB2:
                    if (print_from_z_target == 0.0f){
                        DWN_Message(MSG_INPUT_Z_HEIGHT, "", false);
                    }else{
                        card.tl_ls();
                        DWN_Page(DWN_P_SEL_FILE);
                    }
                    break;
                #endif
                case 0x63:
                    DWN_Message(MSG_STOP_PRINT_TL, "", false);
                    break;
                case 0x62:
                    //DWN_Pause(false);
                    break;
                case 0x15:
                case 0x64:                                        
                    if (!card.flag.sdprinting)
                    {
                        EXECUTE_GCODE(PSTR("M605 S1"));
                        delay(300);
                        EXECUTE_GCODE(PSTR("G28 X"));
                        delay(100);
                        if (card.isFileOpen() && !card.flag.sdprinting)
                        {
                            DWN_Page(DWN_P_RELOAD);
                        }
                    }
                    
                    break;
                case 0xF1:
                case 0xF2:
                {           
                    static long lLogoBTS;
                    if (lData == 0xF1)
                    {
                        lLogoBTS = millis();
                    }
                    else if (lData == 0xF2)
                    {
                        if (millis() - lLogoBTS <= 1500 && !card.flag.sdprinting)
                        {
                            DWN_VClick(5, 5);
                        }
                    }
                }
                break;
                case 0xE2:
                    
                    //long lCal = CalAtv(gsDeviceID);
                    //if (lCal == lAtvCode)
                    //{
                    //    DWN_Text(0x1002, 22, gsDeviceID, false);
                    //    _delay_ms(5);
                    //    DWN_NORFData(0x000002, 0x1002, 22, true);
                    //    _delay_ms(500);
                    //    DWN_Data(0x1022, lAtvCode, 4);
                    //    _delay_ms(5);
                    //    DWN_NORFData(0x000022, 0x1022, 2, true);
                    //    _delay_ms(500);
                    //    bAtv = true;
                    //}
                    //else
                    //{
                    //    DWN_Data(0x6066, 6666, 4);
                    //}
                    
                    break;
                }
            }
        }
        tl_command[0] = 0x00;
        tl_command[1] = 0x00;
    }
    */
}

void DWN_Message(const int MsgID, const char sMsg[], const bool PowerOff){
    dwnMessageID = MsgID;
    int iSend = dwnMessageID + tl_languageID * 13;
    if (dwnMessageID == 13)
        iSend = 3 + tl_languageID * 13;

    DWN_Data(0x9052, dwnMessageID, 2);
    DWN_Data(0x9050, iSend, 2);
    DWN_DELAY;
    DWN_Text(0x7000, 32, sMsg);
    DWN_DELAY;

    if (PowerOff == 0)
        iSend = 0;
    else
        iSend = PowerOff + tl_languageID;

    DWN_Data(0x8830, iSend, 2);
    DWN_DELAY;
    DWN_Page(DWN_P_MSGBOX);
}

int iPrintID = -1;
void DWN_MessageHandler(const bool ISOK){
    switch (dwnMessageID)
    {
    case MSG_RESET_DEFALT:
    {
        #if ENABLED(SDSUPPORT)
        if (card.isFileOpen()){
            DWN_Page(DWN_P_SETTING_PRINTING);
        }else{
            DWN_Page(DWN_P_SETTING_MAIN);
        }
        if (ISOK){
            EXECUTE_GCODE("M502");
        }
        #endif
    }
        break;
    case MSG_POWER_OFF:
        //if (ISOK)
        //    command_M81(false);
        //else
        //    DWN_Page(DWN_P_TOOLS);
        break;
    case MSG_START_PRINT_TL:
        if (ISOK)
        {
            #if ENABLED(SDSUPPORT)
            if (file_name_list[iPrintID][0] != '\0')
            {
                #if ENABLED(PRINT_FROM_Z_HEIGHT)
                    if (print_from_z_target > 0)
                        PrintFromZHeightFound = false;
                    else
                        PrintFromZHeightFound = true;
                #endif

                if (card.flag.sdprinting)
                {
                    planner.synchronize();
                    card.closefile();
                }

                sprintf_P(cmd, PSTR("M32 !%s"), file_name_list[iPrintID]);
                TLPrintingStatus = 1;
                settings.plr_fn_save(iPrintID);
                EXECUTE_GCODE(cmd);

                sprintf_P(cmd, PSTR("Printing %s"), long_file_name_list[iPrintID]);
                DWN_Text(0x7500, 32, cmd, true);
                DWN_DELAY;
                DWN_Page(DWN_P_PRINTING);
            }
            #endif
        }
        else
        {
            #if ENABLED(PRINT_FROM_Z_HEIGHT)
                if (print_from_z_target > 0)
                    DWN_Page(DWN_P_SEL_Z_FILE);
                else
                    DWN_Page(DWN_P_SEL_FILE);
            #endif
        }
        break;
    case MSG_PRINT_FINISHED:
        DWN_Page(DWN_P_MAIN);
        break;
    case MSG_STOP_PRINT_TL:
        if (ISOK)
        {
            DWN_Text(0x7000, 32, " Stopping, Pls wait...");
            tlAbortPrinting();
            //quickStop();
            //bHeatingStop = true;
            //enquecommand_P(PSTR("M1033"));
        }
        else
            DWN_Page(DWN_P_PRINTING);
        break;
    case MSG_INPUT_Z_HEIGHT:
        DWN_Page(DWN_P_PRINTZ);
        break;
    case MSG_NOZZLE_HEATING_ERROR:
    case MSG_NOZZLE_HIGH_TEMP_ERROR:
    case MSG_NOZZLE_LOW_TEMP_ERROR:
    case MSG_BED_HIGH_TEMP_ERROR:
    case MSG_BED_LOW_TEMP_ERROR:
        tlAbortPrinting();
        break;
    case MSG_FILAMENT_RUNOUT:
    /*
        enquecommand_P(PSTR("M605 S1"));
        _delay_ms(300);
        enquecommand_P(PSTR("G28 X"));
        _delay_ms(100);
        if (card.isFileOpen() && card.sdprinting == 0)
            DWN_Page(DWN_P_RELOAD);
    */
        break;
    case MSG_POWER_LOSS_DETECTED:
        /*
        bAtv = true;
        if (ISOK)
        {
            command_M1003();
        }
        else
        {
            DWN_Page(DWN_P_MAIN);
#if defined(POWER_LOSS_SAVE_TO_EEPROM)
            EEPROM_Write_PLR();
            EEPROM_PRE_Write_PLR();
#elif defined(POWER_LOSS_SAVE_TO_SDCARD)
            card.Write_PLR();
            card.PRE_Write_PLR();
#endif
        }
        */
        break;
    }
}

void Setting_ECO_MODE(){
    #if ENABLED(SDSUPPORT)
	#ifndef ELECTROMAGNETIC_VALUE
    static bool bECOSeted;
    static int iECOBedT;
    if (current_position[Z_AXIS] >= ECO_HEIGHT && !bECOSeted && card.flag.sdprinting && tl_ECO_MODE == 1)
    {
        iECOBedT = thermalManager.degTargetBed();
        thermalManager.setTargetBed(0);
        bECOSeted = true;
    }
    else if (current_position[Z_AXIS] >= ECO_HEIGHT && card.flag.sdprinting && tl_ECO_MODE == 0 && bECOSeted && iECOBedT > 0)
    {
        thermalManager.setTargetBed(iECOBedT);
        bECOSeted = false;
    }

    if (current_position[Z_AXIS] <= ECO_HEIGHT && bECOSeted)
    {
        bECOSeted = false;
    }
	#endif
    #endif
}

void tenlog_screen_update_dwn()
{
#ifdef SUPPORT_DWN
    //if (!bAtv)
    //    return;
    if (current_position[X_AXIS] < 0)
        DWN_Data(0x6006, (current_position[X_AXIS] * 10.0f + 0x10000), 2);
    else
        DWN_Data(0x6006, current_position[X_AXIS] * 10.0f, 2);
    DWN_DELAY;

    if (current_position[Y_AXIS] < 0)
        DWN_Data(0x6007, (current_position[Y_AXIS] * 10.0f + 0x10000), 2);
    else
        DWN_Data(0x6007, current_position[Y_AXIS] * 10.0f, 2);
    DWN_DELAY;

    if (current_position[Z_AXIS] < 0)
        DWN_Data(0x6008, (current_position[Z_AXIS] * 10.0f + 0x10000), 2);
    else
        DWN_Data(0x6008, current_position[Z_AXIS] * 10.0f, 2);
    DWN_DELAY;

    DWN_Data(0x8000, thermalManager.isHeatingHotend(0), 2);
    DWN_DELAY;

    DWN_Data(0x8002, thermalManager.isHeatingHotend(1), 2);
    DWN_DELAY;

    DWN_Data(0x8004, thermalManager.isHeatingBed(), 2);
    DWN_DELAY;

    DWN_Data(0x6000, int(thermalManager.degTargetHotend(0) + 0.5f), 2);
    DWN_DELAY;

    DWN_Data(0x6001, int(thermalManager.degHotend(0) + 0.5f), 2);
    DWN_DELAY;

    DWN_Data(0x6002, int(thermalManager.degTargetHotend(1) + 0.5f), 2);
    DWN_DELAY;

    DWN_Data(0x6003, int(thermalManager.degHotend(1) + 0.5f), 2);
    DWN_DELAY;

    DWN_Data(0x6004, int(thermalManager.degTargetBed() + 0.5f), 2);
    DWN_DELAY;

    DWN_Data(0x6005, int(thermalManager.degBed() + 0.5f), 2);
    DWN_DELAY;

    //BOF For old version UI
    #if HAS_FAN
    int iFan = (int)((float)thermalManager.fan_speed[0] / 256.0f * 100.0f + 0.5f);
    #else
    int iFan = 0;
    #endif
    
    #if HAS_FAN
    if (thermalManager.fan_speed[0] == 0)
        DWN_Data(0x8006, 0, 2);
    else
        DWN_Data(0x8006, 1, 2);
    #endif

    DWN_DELAY;
    DWN_Data(0x600A, iFan, 2);
    DWN_DELAY;
    
    DWN_Data(0x602A, iMoveRate, 2);
    DWN_DELAY;
   
    DWN_Data(0x6052, feedrate_percentage, 2);
    DWN_DELAY;

    char sTime[10] = "-- :--";
    int iTimeS = 0;
    int iPercent = 0;
    #if ENABLED(SDSUPPORT)
    if (card.flag.sdprinting)
    {
        uint16_t time = millis() / 60000 - startPrintTime / 60000;
        
        sprintf_P(sTime, "%02d :%02d", (time / 60),(time % 60));

        iPercent = card.percentDone();
        DWN_Data(0x6051, iPercent, 2);
        DWN_DELAY;
        DWN_Data(0x8820, iPercent, 2);
        DWN_DELAY;
    }
    else
    {
        DWN_Data(0x6051, 0, 2);
        DWN_DELAY;
        DWN_Data(0x8820, 0, 2);
        DWN_DELAY;
        iPercent = 0;
        iTimeS = 1;
    }
    #endif

    DWN_Data(0x8840, card.flag.sdprinting + tl_languageID * 3, 2);
    DWN_DELAY;
    DWN_Data(0x8842, card.flag.sdprinting, 2);
    DWN_DELAY;

    DWN_Text(0x7540, 8, sTime);
    DWN_DELAY;

    DWN_Data(0x8841, iTimeS, 2);
    DWN_DELAY;
    
    Setting_ECO_MODE();
    
    static int siCM;
    int iCM;

    #if ENABLED(DUAL_X_CARRIAGE)
    if (dual_x_carriage_mode == 2)
        iCM = 3;
    else if (dual_x_carriage_mode == 3)
        iCM = 4;
    else if (dual_x_carriage_mode == 1){
        static bool bAPMNozzle;
        bAPMNozzle = !bAPMNozzle;
        if (active_extruder == 0 && bAPMNozzle)
            iCM = 1;
        else if (active_extruder == 1 && bAPMNozzle)
            iCM = 2;
        else if (!bAPMNozzle)
            iCM = 0;
    }
    if (siCM != iCM){
        DWN_Data(0x8800, iCM, 2);
        DWN_DELAY;
    }
    siCM = iCM;

    int iMode = (dual_x_carriage_mode - 1) + tl_languageID * 3;
    DWN_Data(0x8801, iMode, 2);
    DWN_Data(0x8804, (dual_x_carriage_mode - 1), 2);
    DWN_DELAY;
    #endif

    int iAN = active_extruder + tl_languageID * 2;
    DWN_Data(0x8802, iAN, 2); // is for UI V1.3.6
    DWN_Data(0x8805, active_extruder, 2);
    DWN_DELAY;
    
    
    if (gsM117 != "" && gsM117 != "Printing...")
    { 
        //Do not display "Printing..."
        char * sPrinting = "";
        static int icM117;

        if (icM117 > 0)
        {
            icM117--;
        }

        if (icM117 == 0)
        {
            sPrinting = gsM117;
            icM117 = 60;
        }
        else if (icM117 == 30)
        {
            sPrinting = gsPrinting;
        }

        if (icM117 == 30 || icM117 == 0 || icM117 == 60)
        { //Switch message every 30 secounds
            DWN_Text(0x7500, 32, sPrinting, true);
            DWN_DELAY;
        }
    }
    */
    #ifdef PRINT_FROM_Z_HEIGHT
    DWN_Data(0x6041, (long)(print_from_z_target * 10.0f), 2);
    DWN_DELAY;
    #endif

    if (lLEDTimeoutCount <= DWN_LED_TIMEOUT)
    {
        lLEDTimeoutCount++;
    }

    if (lLEDTimeoutCount == DWN_LED_TIMEOUT)
    {
        DWN_LED(DWN_LED_OFF);
        lLEDTimeoutCount++;
        if (iDWNPageID != DWN_P_MAIN && !card.isFileOpen())
        {
            DWN_Page(DWN_P_MAIN);
        }
        else if (iDWNPageID != DWN_P_PRINTING && card.isFileOpen())
        {
            DWN_Page(DWN_P_PRINTING);
        }
    }

    /*
    if (iBeepCount >= 0)
    {
    #if (BEEPER > 0)
        if (iBeepCount % 2 == 1)
        {
            WRITE(BEEPER, BEEPER_ON);
        }
        else
        {
            WRITE(BEEPER, BEEPER_OFF);
        }
    #endif
        iBeepCount--;
    }
    if (!bInited)
    {
        //Init_TLScreen_dwn();
        bInited = true;
    }733

    */
#endif //support dwn
}

void tenlog_status_update(bool isTJC)
{
    #if ENABLED(TL_NO_SCREEN)
    if(isTJC) return;
    #endif

    int32_t lnAll = 0;
    const int16_t ln0  = current_position[X_AXIS] * 10.0f; 
    const int16_t ln1  = current_position[Y_AXIS] * 10.0f;
    const int16_t ln2  = current_position[Z_AXIS] * 10.0f;
    const int16_t ln3  = 0;
		
	#ifdef ELECTROMAGNETIC_VALUE
    const int16_t ln4  = 0;
    const int16_t ln5  = 0;
    const int16_t ln6  = 0;
    const int16_t ln7  = 0;
    const int8_t ln8  = 0;
    const int8_t ln9  = 0;		
	#else
    const int16_t ln4  = int(thermalManager.degTargetHotend(0) + 0.5f);
    const int16_t ln5  = int(thermalManager.degHotend(0) + 0.5f);
    const int16_t ln6  = int(thermalManager.degTargetHotend(1) + 0.5f);
    const int16_t ln7  = int(thermalManager.degHotend(1) + 0.5f);
    const int8_t ln8  = int(thermalManager.degTargetBed() + 0.5f);
    const int8_t ln9  = int(thermalManager.degBed() + 0.5f);
	#endif
		
    #if HAS_FAN
    const int8_t ln10 = (float)thermalManager.fan_speed[active_extruder] * 100.0f / 256.0f + 0.5f;
    #else
    const int8_t ln10 = 0;
    #endif    
    const int16_t ln11 = feedrate_percentage;

    int8_t ln12 = 0;
    int8_t ln13 = 0;
    #if ENABLED(SDSUPPORT)
    ln12 = card.flag.sdprinting;
    if(TLPrintingStatus == 2)
        ln12 = TLPrintingStatus;
    if(file_uploading)
        ln12 = 0;

    ln13 = card.percentDone();
    if(ln12 == 0)
        ln13 = 0;
    if(card.isFileOpen() && ln12 == 0){
        ln13 = 0;
    }
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)
    const int8_t ln14 = active_extruder;
    const int8_t ln15 = dual_x_carriage_mode;
    #else
    const int8_t ln14 = 0;
    const int8_t ln15 = 0;
    #endif

    char cTime[10];
    NULLZERO(cTime);
    uint16_t ln16 = 0;
    uint8_t lnSeconds = 0;
    if(ln12 == 0)
    {
        sprintf_P(cTime, "%s", "00:00");
    }else{
        ln16 = (millis() - startPrintTime) / 60000;//分钟数
        lnSeconds = ((millis() - startPrintTime) / 1000) - ln16 * 60;//秒钟数
        uint16_t time_h = ln16/60;
        uint16_t time_m = ln16%60;
        
        sprintf_P(cTime, PSTR("%02d:%02d"), time_h, time_m);
    }

    int8_t ln17 = 0;
    #if ENABLED(SDSUPPORT)
    ln17 = card.isFileOpen();
    if(ln17 == 0 && TLPrintingStatus == 2)
        ln17 = 1;
    if(file_uploading)
        ln17 = 0;
    #endif

	#if HAS_HOTEND
    const int8_t ln18 = thermalManager.isHeatingHotend(0);
    const int8_t ln19 = thermalManager.isHeatingHotend(1);
    const int8_t ln20 = thermalManager.isHeatingBed();
	#else
    const int8_t ln18 = 0;
    const int8_t ln19 = 0;
    const int8_t ln20 = 0;
	#endif


    uint8_t e0_flow = planner.flow_percentage[0];
    #if ENABLED(SINGLE_HEAD)
        uint8_t e1_flow = 0;
    #else
        uint8_t e1_flow = planner.flow_percentage[1];
    #endif

    if(e0_flow < 0 || e0_flow > 250) e0_flow = 100;
    if(e1_flow < 0 || e1_flow > 250) e1_flow = 100;
    //TLDEBUG_PRINTLNPAIR("E0 Flow", e0_flow, "E1 Flow", e1_flow);    

    #if ENABLED(HAS_WIFI)
    if((wifi_connected || wifiFirstSend < 2000) && !isTJC){
        wifiFirstSend ++;
        
        ZERO(wifi_printer_status);

        wifi_printer_status[0] = (ln0 + 1000) / 0x100;
        wifi_printer_status[1] = (ln0 + 1000) % 0x100;    // ^~^
        
        wifi_printer_status[2] = ln1 / 0x100;
        wifi_printer_status[3] = ln1 % 0x100;
        
        wifi_printer_status[4] = ln2 / 0x100;
        wifi_printer_status[5] = ln2 % 0x100;
        
        #ifndef ELECTROMAGNETIC_VALUE
        wifi_printer_status[6] = (uint16_t)(thermalManager.degTargetHotend(0) * 100.0f) / 0x100;
        wifi_printer_status[7] = (uint16_t)(thermalManager.degTargetHotend(0) * 100.0f) % 0x100;
        
        wifi_printer_status[8] = (uint16_t)(thermalManager.degHotend(0) * 100.0f) / 0x100;
        wifi_printer_status[9] = (uint16_t)(thermalManager.degHotend(0) * 100.0f) % 0x100;
        
        wifi_printer_status[10] = (uint16_t)(thermalManager.degTargetHotend(1) * 100.0f) / 0x100;
        wifi_printer_status[11] = (uint16_t)(thermalManager.degTargetHotend(1) * 100.0f) % 0x100;
        
        wifi_printer_status[12] = (uint16_t)(thermalManager.degHotend(1) * 100.0f) / 0x100;
        wifi_printer_status[13] = (uint16_t)(thermalManager.degHotend(1) * 100.0f) % 0x100;
        
        wifi_printer_status[14] = (uint16_t)(thermalManager.degTargetBed() * 100.0f) / 0x100;
        wifi_printer_status[15] = (uint16_t)(thermalManager.degTargetBed() * 100.0f) % 0x100;

        wifi_printer_status[16] = (uint16_t)(thermalManager.degBed() * 100.0f) / 0x100;
        wifi_printer_status[17] = (uint16_t)(thermalManager.degBed() * 100.0f) % 0x100;
        #else
        wifi_printer_status[6] = 0;
        wifi_printer_status[7] = 0;

        wifi_printer_status[8] = 0;
        wifi_printer_status[9] = 0;

        wifi_printer_status[10] = 0;
        wifi_printer_status[11] = 0;

        wifi_printer_status[12] = 0;
        wifi_printer_status[13] = 0;

        wifi_printer_status[14] = 0;
        wifi_printer_status[15] = 0;

        wifi_printer_status[16] = 0;
        wifi_printer_status[17] = 0;
       #endif
        
        wifi_printer_status[18] = ln10;
        wifi_printer_status[19] =  (uint16_t)ln11 % 0x100; //19 & 33 = feedrate per

        wifi_printer_status[20] = ln12;
        wifi_printer_status[21] = ln13;
        wifi_printer_status[22] = ln14;
        wifi_printer_status[23] = ln15;

        wifi_printer_status[24] = ln16 / 0x100; //分钟数
        wifi_printer_status[25] = ln16 % 0x100;
        
        wifi_printer_status[26] = ln17;
        wifi_printer_status[27] = ln18;
        wifi_printer_status[28] = ln19;
        wifi_printer_status[29] = ln20;
        wifi_printer_status[30] = TLMoving?1:0;
        wifi_printer_status[31] = tl_print_page_id;
        wifi_printer_status[32] = tl_print_file_id;

        wifi_printer_status[33] =  (uint16_t)ln11 / 0x100; //19 & 33 = feedrate per
        wifi_printer_status[34] =  sd_OK; //SD OK
        wifi_printer_status[35] =  lnSeconds; //SD OK
        
        WIFI_TX_Handler(0x07);

        if(!wifi_resent && wifi_connected){
           #if ENABLED(SDSUPPORT)
            card.tl_ls(true);
            WIFI_TX_Handler(0x08);
            #endif
            wifi_resent = true;
            tlSendSettings(true);
        }

        bool SettingsSent = true;
        for (uint16_t i=0; i<WIFI_MSG_LENGTH; i++){
            if(wifi_printer_settings[i] > 0){
                SettingsSent = false;
                break;
            }
        }

        if((!SettingsSent) && wifi_connected){
            WIFI_TX_Handler(0x09);
            ZERO(wifi_printer_settings);
        }
    }
    #endif //HAS_WIFI

    if(isTJC){
        char printerStatus[100];
        char V25[9] = "V:2.2.25";
        if(strlen(tl_tjc_ver)<8){
            TLTJC_GetTJCVersion();
        }
        if(strcmp_P(tl_tjc_ver,V25)>0) {       
            lnAll = ln0+ln1+ln2+ln3+ln4+ln5+ln6+ln7+ln8+ln9+ln10+ln11+ln12+ln13+ln14+ln15+ln17+ln18+ln19+ln20+e0_flow+e1_flow;
            sprintf_P(printerStatus, PSTR("%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s|%d|%d|%d|%d|%d|%d|%d|"), 
                ln0, ln1, ln2, ln3, ln4, ln5, ln6, ln7, ln8, ln9, ln10, ln11, ln12, ln13, ln14, ln15, cTime, ln17, ln18, ln19, ln20, e0_flow, e1_flow, lnAll%0xFF);
            //TLDEBUG_PRINTLN("V2226");
        }else{
            lnAll = ln0+ln1+ln2+ln3+ln4+ln5+ln6+ln7+ln8+ln9+ln10+ln11+ln12+ln13+ln14+ln15+ln17+ln18+ln19+ln20;
            sprintf_P(printerStatus, PSTR("%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s|%d|%d|%d|%d|%d|"), 
                ln0, ln1, ln2, ln3, ln4, ln5, ln6, ln7, ln8, ln9, ln10, ln11, ln12, ln13, ln14, ln15, cTime, ln17, ln18, ln19, ln20, lnAll%0xFF);
            //TLDEBUG_PRINTLN("V2225");
        }
        TLSTJC_print("main.sStatus.txt=\"");
        TLSTJC_print(printerStatus);
        TLSTJC_println("\"");
        TJC_DELAY;
        TLSTJC_println("click btReflush,0");
        Setting_ECO_MODE();
    }

    /*
    if (gsM117 != "" && gsM117 != "Printing...")
    { //Do not display "Printing..."
        static int icM117;

        if (icM117 > 0)
        {
            icM117--;
        }

        if (icM117 == 0)
        {
            delay(50);
            sprintf_P(temp, PSTR("printing.tM117.txt=\"\"")
            TLSTJC_printconst(F("printing.tM117.txt=\""));            
            String strM117 = "" + gsM117 + "";
            const char *strM1170 = strM117.c_str();
            TLSTJC_println(strM1170);
            TLSTJC_println("\""));
            icM117 = 60;
        }
        else if (icM117 == 30)
        {
            delay(50);
            TLSTJC_println("printing.tM117.txt=\"\""));
            delay(50);
        }
    }

    delay(50);
    TLSTJC_println("click btReflush,0"));

    if (iBeepCount >= 0)
    {

#if (BEEPER > 0)
        if (iBeepCount % 2 == 1)
        {
            WRITE(BEEPER, BEEPER_ON);
        }
        else
        {
            WRITE(BEEPER, BEEPER_OFF);
        }
#endif
        iBeepCount--;
    }
    if (!bInited)
    {
        Init_TLScreen_tjc();
        bInited = true;
    }
    */
}


void process_command_gcode(long _tl_command[]) {
    //Translate Gcode from TJC controller to the original gcode handler.

    long lFrom[10] = {0}; //10 lines.
    ZERO(lFrom);

    long lP = 1;
    long lLen = 0;
    for(uint16_t i=0; i<256; i++){
        /*
        if(i < 25 && _tl_command[i]>0){
            sprintf(cmd, "#%d", _tl_command[i]);
            TLDEBUG_PRINTLN(cmd);
        }
        */

        if(_tl_command[i] == 0x0A)
        {
            lFrom[lP] = i + 1;
            lP++;
        }
        if(_tl_command[i] == '\0')
            break;
        lLen++;
    }
    

    if(lLen > 0) {
        for(int i=0; i<lP; i++) {

            int iFrom = lFrom[i];
            long lG = GCodelng('G', iFrom, _tl_command, true);
            long lT = GCodelng('T', iFrom, _tl_command, true);
            long lM = GCodelng('M', iFrom, _tl_command, true);

            sprintf_P(cmd, PSTR("G%d T%d M%d "), lG, lT, lM);
            TLDEBUG_PRINTLN(cmd);
            
            if(lG == 1 || lG == 0) {
                //G1
                SetBusyMoving(true);
                long lM = GCodelng('M', iFrom, _tl_command);
                long lF = GCodelng('F', iFrom, _tl_command);
                float fX = GCodelng('X', iFrom, _tl_command);
                float fY = GCodelng('Y', iFrom, _tl_command);
                float fZ = GCodelng('Z', iFrom, _tl_command);
                long lE = GCodelng('E', iFrom, _tl_command);
                long lR = GCodelng('R', iFrom, _tl_command);
                
                long lRate = 1;
                long lMode = 0;
                if(lR != -999) lRate = lR;
                if(lM != -999) lMode = lM;

                char sX[10],sY[10],sZ[10],sE[10],sF[10];
                NULLZERO(sX);
                NULLZERO(sY);
                NULLZERO(sZ);
                NULLZERO(sE);
                NULLZERO(sF);
                
                float fTemp = 0.0;
                if(fX > -999.0)
                {
                    fTemp = fX / (float)lRate + current_position[X_AXIS] * lMode;
                    sprintf_P(sX, PSTR("X%s "), dtostrf(fTemp, 1, 1, str_1));                
                }
                if(fY > -999.0)
                {
                    fTemp = fY / (float)lRate + current_position[Y_AXIS] * lMode;
                    sprintf_P(sY, PSTR("Y%s "), dtostrf(fTemp, 1, 1, str_1));                
                }
                if(fZ > -999.0)
                {
                    fTemp = fZ / (float)lRate + current_position[Z_AXIS] * lMode;
                    sprintf_P(sZ, PSTR("Z%s "), dtostrf(fTemp, 1, 1, str_1));                
                }
                if(lE != -999)
                {
                    fTemp = (float)lE / (float)lRate;
                    float fEPos = current_position[E_AXIS] * lM;
                    fEPos += fTemp;
                    sprintf_P(sE, PSTR("E%f "), fEPos);
                }
                if(lF != -999)
                {
                    sprintf_P(sF, PSTR("F%d "), lF);                
                }
                feedrate_mm_s = lF / 60;
                sprintf_P(cmd, PSTR("G%d %s%s%s%s%s"), lG, sF, sX, sY, sZ, sE);
                EXECUTE_GCODE(cmd);
                SetBusyMoving(false);
            }else if(lM == 1586 || lM == 1587){
                long lS = GCodelng('S', iFrom, _tl_command);
                #if ENABLED(TL_STEPTEST)
                    if(lM == 1586){
                        //M1586 老范挤出机测试
                        if(lS == 1) {
                            WRITE(XX_ENABLE_PIN, 0);
                            WRITE(XX_DIR_PIN, 0);
                            set_pwm_hw(2, 255, UN_TLTS);
                        }else if(lS == 2){
                            WRITE(XX_ENABLE_PIN, 0);
                            WRITE(XX_DIR_PIN, 1);
                            set_pwm_hw(2, 255, UN_TLTS);
                        }else if(lS == 3){
                            WRITE(XX_ENABLE_PIN, 1);
                            WRITE(XX_DIR_PIN, 0);
                            set_pwm_hw(0, 255, UN_TLTS);
                        }
                        sprintf_P(cmd, PSTR("M%d S%d"), lM, lS);
                        TLDEBUG_PRINTLN(cmd);
                    }else if(lM == 1587){
                        //M1587 老范挤出机测试
                        STEPTEST_HZ = lS;
                        EXECUTE_GCODE(PSTR("M500"));
                        pwm_init();
                        set_pwm_hw(1, 255, UN_TLTS);
                        sprintf_P(cmd, PSTR("M%d S%d"), lM, lS);
                        TLDEBUG_PRINTLN(cmd);
                        sprintf_P(cmd, PSTR("M%d S%d"), lM, lS);
                        TLDEBUG_PRINTLN(cmd);
                    }
                #endif
            } else if(lM == 201 || lM == 203 || lM == 204 || lM == 205 || lM == 301 || lM == 304){
                //M201 XYZE M203 XYZE M204 PTR M205 XYZE M301 PID M304 PID

                float fX = GCodelng('X', iFrom, _tl_command);
                float fY = GCodelng('Y', iFrom, _tl_command);
                float fZ = GCodelng('Z', iFrom, _tl_command);
                float fE = GCodelng('E', iFrom, _tl_command);
                float fT = GCodelng('T', iFrom, _tl_command);
                float fP = GCodelng('P', iFrom, _tl_command);
                float fI = GCodelng('I', iFrom, _tl_command);
                float fD = GCodelng('D', iFrom, _tl_command);
                int32_t lR = GCodelng('R', iFrom, _tl_command);

                uint32_t lRate = 1;                    
                if(lM != 204 && lR != -999) lRate = lR;

                char sX[16],sY[16],sZ[16],sE[16],sP[16],sI[16],sD[16],sR[16],sT[16];
                NULLZERO(sX);
                NULLZERO(sY);
                NULLZERO(sZ);
                NULLZERO(sE);
                NULLZERO(sP);
                NULLZERO(sI);
                NULLZERO(sD);
                NULLZERO(sR);
                NULLZERO(sT);

                uint8_t PageNo = 1;
                if(lM == 204 || lM == 205) PageNo = 2;
                char chrA[2];
                NULLZERO(chrA);

                float fTemp = 0.0;
                if(fX > -999.0) {
                    fTemp = fX / (float)lRate;
                    sprintf_P(sX, PSTR("X%.2f "), fTemp);
                    chrA[0] = 'X';
                }
                if(fY > -999.0) {
                    fTemp = fY / (float)lRate;
                    sprintf_P(sY, PSTR("Y%.2f "), fTemp);
                    chrA[0] = 'Y';
                }
                if(fZ > -999.0) {
                    fTemp = fZ / (float)lRate;
                    sprintf_P(sZ, PSTR("Z%.2f "), fTemp);
                    chrA[0] = 'Z';
                }
                if(fE > -999.0) {
                    fTemp = fE / (float)lRate;
                    sprintf_P(sE, PSTR("E%.2f "), fTemp);
                    chrA[0] = 'E';
                }
                if(fP > -999.0) {
                    fTemp = fP / (float)lRate;
                    sprintf_P(sP, PSTR("P%.2f "), fTemp);
                    chrA[0] = 'P';
                }
                if(fI > -999.0) {
                    fTemp = fI / (float)lRate;
                    sprintf_P(sI, PSTR("I%.2f "), fTemp);
                    chrA[0] = 'I';
                }
                if(fD > -999.0) {
                    fTemp = fD / (float)lRate;
                    sprintf_P(sD, PSTR("D%.2f "), fTemp);
                    chrA[0] = 'D';
                }
                if(fT > -999.0) {
                    fTemp = fT / (float)lRate;
                    sprintf_P(sT, PSTR("T%.2f "), fTemp);
                    chrA[0] = 'T';
                }
                if(lR > -999 && lM == 204){
                    sprintf_P(sR, PSTR("R%d "), lR);
                    chrA[0] = 'R';
                    fTemp = lR;
                }
                sprintf_P(cmd, "settings%d.xM%d%s.val=%d", PageNo, lM, chrA, (int32_t)(fTemp * (float)lRate));
                TLSTJC_println(cmd);
                sprintf_P(cmd, "Temp:%.2f, Rate:%d", fTemp, lRate);
                TLDEBUG_PRINTLN(cmd);  

                sprintf_P(cmd, PSTR("M%d %s%s%s%s%s%s%s%s%s"), lM, sX, sY, sZ, sE, sP, sI, sD, sT, sR);
                EXECUTE_GCODE(cmd);
                EXECUTE_GCODE(PSTR("M500"));

            } else if(lG == 28){
                //G28
                SetBusyMoving(true);
                long lX = GCodelng('X', iFrom, _tl_command);
                long lY = GCodelng('Y', iFrom, _tl_command);
                long lZ = GCodelng('Z', iFrom, _tl_command);

                char sX[10],sY[10],sZ[10];
                NULLZERO(sX);
                NULLZERO(sY);
                NULLZERO(sZ);

                if(lX > -999) {
                    sX[0] = 'X';
                    //sprintf_P(sX, PSTR("X"));
                    sprintf_P(cmd, PSTR("G%d X\\n"), lG);
                }
                if(lY > -999) {
                    sY[0] = 'Y';
                    sprintf_P(cmd, PSTR("G%d Y\\n"), lG);
                    //sprintf_P(sY, PSTR("Y"));                
                }
                if(lZ > -999) {
                    sZ[0] = 'Z';
                    sprintf_P(cmd, PSTR("G%d Z\\n"), lG);
                    //sprintf_P(sZ, PSTR("Z"));                
                }

                if(lX == -999 && lY == -999 && lZ == -999)
                sprintf_P(cmd, PSTR("G%d \\n"), lG);
                
                //TLDEBUG_PRINTLN(cmd);
                EXECUTE_GCODE(cmd);
                SetBusyMoving(false);
            }else if(lT == 0 || lT == 1){                    
                //T0 , T1
                SetBusyMoving(true);
                sprintf_P(cmd, PSTR("T%d"), lT);
                //TLDEBUG_PRINTLN(cmd);
                EXECUTE_GCODE(cmd);
                SetBusyMoving(false);
            }else if(lM == 1022){
                //M1022
                SetBusyMoving(true);
                long lS = GCodelng('S', iFrom, _tl_command);
                long lT = GCodelng('T', iFrom, _tl_command);
                if((lS == 0 || lS == 1) && lT > -999){
                    load_filament(lS, lT);
                }else if(lS == 2 || lS == 3 || lS == 4 || lS == 5){
                    long lX = 0;
                    long lY = 0;
                    if(lS == 5){
                        lX = 32;
                        lY = Y_MAX_POS - 53;
                    }else if(lS == 4){
                        lX = X_BED_SIZE - 28;
                        lY = Y_MAX_POS - 53;
                    }else if(lS == 2){
                        lX = 32;
                        lY = 25;
                    }else if(lS == 3){
                        lX = X_BED_SIZE - 28;
                        lY = 25;
                    }
                    EXECUTE_GCODE("G1 F9000");
                    my_sleep(0.8);
                    EXECUTE_GCODE("G1 Z5.0 F9000");
                    my_sleep(0.8);
                    sprintf_P(cmd, PSTR("G1 X%d Y%d"), lX, lY);
                    EXECUTE_GCODE(cmd);
                    my_sleep(0.8);
                    EXECUTE_GCODE("G1 Z0");
                    my_sleep(0.8);
                }else if(lS == 6){
                    EXECUTE_GCODE("G28 XYZ");
                    my_sleep(2.0);
                    //EXECUTE_GCODE("G1 X-50.0 Y0");
                    //my_sleep(1.0);
                }
                SetBusyMoving(false);
            }else if(lM == 104 || lM == 221){
                //M104 //M221
                long lTT = GCodelng('T', iFrom, _tl_command);
                long lS = GCodelng('S', iFrom, _tl_command);
                char sT[10],sS[10];
                NULLZERO(sT);
                NULLZERO(sS);
                if(lTT > -999) {
                    sprintf_P(sT, PSTR("T%d "), lTT);                
                }
                if(lS > -999) {
                    sprintf_P(sS, PSTR("S%d "), lS);                
                }
                sprintf_P(cmd, PSTR("M%d %s%s"), lM, sT, sS);
                EXECUTE_GCODE(cmd);
            }else if(lM == 140 || lM == 220 || lM == 18){
                //M220 //M140 //M18
                long lS = GCodelng('S', iFrom, _tl_command);
                char sS[10];
                NULLZERO(sS);
                if(lS > -999) {
                    sprintf_P(sS, PSTR("S%d "), lS);
                }
                sprintf_P(cmd, PSTR("M%d %s"), lM, sS);
                EXECUTE_GCODE(cmd);
            }else if(lM == 106){
                //M106
                float fS = GCodelng('S', iFrom, _tl_command);
                uint8_t lR = GCodelng('R', iFrom, _tl_command);
                char sS[10], sR[10];
                NULLZERO(sS);
                NULLZERO(sR);
                float fR = 1.0f;
                if(lR == 1){
                    fR = 2.55f;
                }
                uint16_t lS = 0;
                if(fS > -999.0) {
                    lS = (uint16_t)((float)(fS * fR) + 0.5f);
                    sprintf_P(sS, PSTR("S%d "), lS);                
                }
                sprintf_P(cmd, PSTR("M%d %s"), lM, sS);
                EXECUTE_GCODE(cmd);
            }else if(lM == 107){
                //M107
                sprintf_P(cmd, PSTR("M%d"), lM);
                EXECUTE_GCODE(cmd);
            }else if(lM == 21){
                sprintf_P(cmd, PSTR("M%d"), lM);
                EXECUTE_GCODE(cmd);
            }else if(lM == 32){
                //M32
                long lF = GCodelng('F', iFrom, _tl_command);
                uint8_t cmdLength = 0;
                for(uint8_t i=0; i<256; i++){
                    if(_tl_command[i] == 0x00){
                        cmdLength = i;
                        break;
                    }
                }
                NULLZERO(cmd); 
                if(lF > 0 && cmdLength < 8){
                    //print from lcd
                    sprintf_P(cmd, PSTR("M%d !%s"), lM, file_name_list[lF-1]);
                    tl_print_file_id=lF;
                }else{
                    //print from wifi
                    char fileNameP[13];
                    NULLZERO(fileNameP);
                    for(int i=0; i<12; i++){
                        fileNameP[i] = _tl_command[iFrom + 4 + i];
                    }
                    if(strlen(fileNameP) > 2){
                        sprintf_P(cmd, PSTR("M%d !%s"), lM, fileNameP);
                    }
                }
                #if ENABLED(SDSUPPORT)
                if(strlen(cmd)){  
                    TLPrintingStatus = 1;
                    settings.plr_fn_save(lF-1);
                    TLDEBUG_PRINTLN(cmd);
                    #if ENABLED(TL_LASER_ONLY)
                    EXECUTE_GCODE("G92 X-20 Y5");
                    #endif
                    EXECUTE_GCODE(cmd);
                }
                #endif
            }else if(lM == 321){
                #if ENABLED(TL_LASER_ONLY)
                    //M321 pre select print file name
                    //print from wifi
                    NULLZERO(pre_print_file_name);
                    for(int i=0; i<12; i++){
                        pre_print_file_name[i] = _tl_command[iFrom + 4 + i];
                    }
                    if(strlen(pre_print_file_name) > 2){
                        #if ENABLED(TL_BEEPER)
                        start_beeper(100,0);
                        #endif
                        //sprintf_P(cmd, PSTR("M%d !%s"), lM, fileNameP);
                    }
                #endif  //TL_LASER_ONLY
            }else if(lM == 19){
                //M19
                tl_print_page_id = GCodelng('S', iFrom, _tl_command);
                bool wifi = false;
                if(GCodelng('R', iFrom, _tl_command) == 1) wifi=true;
                #if ENABLED(SDSUPPORT)                    
                card.tl_ls(wifi);
                #endif
            }else if(lM == 1001){
                //M1001
                tl_languageID = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1017){
                //M1017 TJC sleep
                tl_Sleep = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1024){
                //M1024
                tl_ECO_MODE = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1520){
                //M1520
                tl_THEME_ID = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1521){
                //M1521 Light On Off
                tl_Light = GCodelng('S', iFrom, _tl_command);
                command_M1521(tl_Light);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 413){
                //M413 PLR
                plr_enabled = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1023){
                //M1023
                #ifdef FILAMENT_RUNOUT_SENSOR
                runout.enabled = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
                #endif
            }else if(lM == 1018){
                //M1018
                #if ENABLED(TL_LASER_ONLY)
                tl_LASER_MAX_VALUE= GCodelng('S', iFrom, _tl_command);
                #else
                tl_E_MAX_TEMP = GCodelng('S', iFrom, _tl_command);
                thermalManager.hotend_maxtemp[0] = tl_E_MAX_TEMP;
                thermalManager.hotend_maxtemp[1] = tl_E_MAX_TEMP;
                sprintf_P(cmd, PSTR("main.vTempMax.val=%d"), tl_E_MAX_TEMP - HOTEND_OVERSHOOT);
                TLSTJC_println(cmd);
                #endif
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1016){
                //M1016
                tl_C_FAN_SPEED = GCodelng('S', iFrom, _tl_command);
                tl_C_FAN_CHANGED = true;
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1015){
                //M1015
                tl_E_FAN_SPEED = GCodelng('S', iFrom, _tl_command);
                tl_E_FAN_CHANGED = true;
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1014){
                //M1014
                tl_E_FAN_START_TEMP = GCodelng('S', iFrom, _tl_command);
                tl_E_FAN_CHANGED = true;
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 502){
                //M502 reset EEPROM
                //TLTJC_GetTJCVersion();
                //printf(cmd, "Company ID:%d", tl_com_ID);
                //TLDEBUG_PRINTLN(cmd);
                EXECUTE_GCODE(PSTR("M502"));
                delay(5);
                EXECUTE_GCODE(PSTR("M500"));
                delay(5);
                EXECUTE_GCODE(PSTR("M501"));
            }else if(lM == 92){
                //M92
                long lRate = 1; //M92
                long lR = GCodelng('R', iFrom, _tl_command);
                if(lR > -999) lRate = lR;

                float fX = GCodelng('X', iFrom, _tl_command);
                float fY = GCodelng('Y', iFrom, _tl_command);
                float fZ = GCodelng('Z', iFrom, _tl_command);
                float fE = GCodelng('E', iFrom, _tl_command);

                char sX[10],sY[10],sZ[10],sE[10];

                NULLZERO(sX);
                NULLZERO(sY);
                NULLZERO(sZ);
                NULLZERO(sE);
                
                float fTemp = 0.0;
                if(fX > -999.0){
                    fTemp = fX / (float)lRate ;
                    sprintf_P(sX, PSTR("X%s "), dtostrf(fTemp, 1, 2, str_1));                
                }
                if(fY > -999.0){
                    fTemp = fY / (float)lRate ;
                    sprintf_P(sY, PSTR("Y%s "), dtostrf(fTemp, 1, 2, str_1));                
                }
                if(fZ > -999.0){
                    fTemp = fZ / (float)lRate ;
                    sprintf_P(sZ, PSTR("Z%s "), dtostrf(fTemp, 1, 2, str_1));                
                }
                if(fE != -999.0){
                    fTemp = fE / (float)lRate ;
                    sprintf_P(sE, PSTR("E%s "), dtostrf(fTemp, 1, 2, str_1));                
                }

                sprintf_P(cmd, PSTR("M%d %s%s%s%s"), lM, sX, sY, sZ, sE);
                EXECUTE_GCODE(cmd);
                EXECUTE_GCODE(PSTR("M500"));
                
            }else if(lM == 605){
                //M605
                char sX[10],sR[10],sS[10];
                
                long lX = GCodelng('X', iFrom, _tl_command);
                NULLZERO(sX);
                if(lX > -999) {
                    sprintf_P(sX, PSTR("X%d "), lX);
                }
                NULLZERO(sR);
                long lR = GCodelng('R', iFrom, _tl_command);
                if(lR > -999) {
                    sprintf_P(sR, PSTR("R%d "), lR);
                }
                
                NULLZERO(sS);
                long lS = GCodelng('S', iFrom, _tl_command);
                if(lS > -999) {
                    sprintf_P(sS, PSTR("S%d "), lS);
                }
                sprintf_P(cmd, PSTR("M%d %s%s%s"), lM, sS, sX, sR);
                EXECUTE_GCODE(cmd);
            }else if(lM == 1031){                
                //pause from lcd or runout //M1031
                TJCPauseResumePrinting(true, 0);
            }else if(lM == 1032){
                //Resume from lcd M1032
                TJCPauseResumePrinting(false, iFrom);
            }else if(lM == 1033){
                //M1033
                //abortSDPrinting
                tlAbortPrinting();
            }else if(lM == 1035){
                                    #if ENABLED(ESP32_WIFI)
                //M1035
                //resend wifi
                wifi_resent = false;
                                    #endif
            }else if(lM == 1021){
                //Pre heat //M1021
                #ifndef ELECTROMAGNETIC_VALUE
                long lS = GCodelng('S',iFrom, _tl_command);
                if(lS == 0){
                    //Pre heat ABS
                    thermalManager.setTargetHotend(PREHEAT_2_TEMP_HOTEND, 0);
                    thermalManager.setTargetHotend(PREHEAT_2_TEMP_HOTEND, 1);
                    thermalManager.setTargetBed(PREHEAT_2_TEMP_BED);
                }else if(lS == 1){
                    //Pre heat PLA
                    thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 0);
                    thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 1);
                    thermalManager.setTargetBed(PREHEAT_1_TEMP_BED);
                }else if(lS == 2){
                    thermalManager.setTargetHotend(0, 0);
                    thermalManager.setTargetHotend(0, 1);
                    thermalManager.setTargetBed(0);
                    disable_all_steppers();
                }
                #endif
            }else if(lM == 1011 || lM == 1012 || lM == 1013){
                #if (EXTRUDERS==2)
                    //hoten offset 1 M1011 M1012 M1013
                    uint32_t lR = GCodelng('R',iFrom, _tl_command);
                    uint32_t lS = GCodelng('S',iFrom, _tl_command);
                    
                    if(lR == 100 && lS > 0 ){
                        
                        long lAxis = lM - 1011;
                        float fOffset = (float)lS / (float)lR;
                        if(lAxis == X_AXIS)
                            hotend_offset[1].x = fOffset;
                        else if(lAxis == Y_AXIS){
                            fOffset = fOffset - 5.0f;
                            hotend_offset[1].y = fOffset;
                        }
                        else if(lAxis == Z_AXIS){
                            fOffset = fOffset;
                            hotend_offset[1].z = fOffset;
                        }
                        EXECUTE_GCODE(PSTR("M500"));
                        hotendOffsetChanged = true;
                    }
                #endif //(EXTRUDERS==2)
            }else if(lM == 1004){
                #if ENABLED(SDSUPPORT)
                //M1004
                settings.plr_reset();//cancel power loss resume.
                #endif
            }else if(lM == 1003){
                #if ENABLED(SDSUPPORT)
                //M1003
                settings.plr_recovery();//do power loss resume.
                #endif
            }else if(lM == 1040){
                //M1040
                #ifdef PRINT_FROM_Z_HEIGHT
                float fValue = GCodelng('S',iFrom, _tl_command);
                if (fValue == 0){
                    PrintFromZHeightFound = true;
                }else{
                    print_from_z_target = fValue / 10.0f;
                    PrintFromZHeightFound = false;
                }
                TLDEBUG_PRINTLNPAIR("M1040 S", fValue);
                #endif //PRINT_FROM_Z_HEIGHT
            }else if (lM == 1050){
                //M1050
                int8_t KillFlag = GCodelng('S',iFrom, _tl_command);
            }else if(lM > 1499 && lM < 1599){
                //1500-1510
                if(lM == 1550){//M1550
                    delay(100);
                    sprintf_P(cmd, "tUID.txt=\"UID:%s\"", tl_hc_sn);
                    TLSTJC_println(cmd);
                    TLDEBUG_PRINTLN(cmd);
                }
                #if ENABLED(HAS_WIFI)
                    if(lM == 1501){
                        //M1501
                        int _wifi_mode = GCodelng('S', iFrom, _tl_command);
                        if(_wifi_mode != wifi_mode){
                            wifi_mode = GCodelng('S', iFrom, _tl_command);
                            EXECUTE_GCODE(PSTR("M500"));
                            SPI_resent_wifi_info();
                        }
                    }else if(lM == 1504){
                        //M1504
                        http_port = GCodelng('S', iFrom, _tl_command);
                        sprintf_P(cmd, PSTR("M%d S%d"), lM, http_port);                        
                        EXECUTE_GCODE(PSTR("M500"));
                        SPI_resent_wifi_info();
                    }else if(lM == 1502){
                        //M1502
                        NULLZERO(wifi_ssid);
                        for(int i=0; i<20; i++){
                            wifi_ssid[i] = _tl_command[iFrom + 6 + i];
                            if(wifi_ssid[i]==10){
                                wifi_ssid[i] = '\0';
                                break;
                            }
                        }
                        sprintf_P(cmd, PSTR("M%d %s"), lM, wifi_ssid);
                        EXECUTE_GCODE(PSTR("M500"));
                        SPI_resent_wifi_info();
                    }else if(lM == 1503){
                        //M1503
                        NULLZERO(wifi_pswd);
                        for(int i=0; i<20; i++){
                            wifi_pswd[i] = _tl_command[iFrom + 6 + i]; 
                            if(wifi_pswd[i]==10){
                                wifi_pswd[i] = '\0';
                                break;
                            }
                        }
                        sprintf_P(cmd, PSTR("M%d %s"), lM, wifi_pswd);                        
                        EXECUTE_GCODE(PSTR("M500"));
                        SPI_resent_wifi_info();
                    }else if(lM == 1505){
                        //M1505 access code
                        NULLZERO(wifi_acce_code);
                        for(int i=0; i<20; i++){
                            wifi_acce_code[i] = _tl_command[iFrom + 6 + i]; 
                            if(wifi_acce_code[i]==10){
                                wifi_acce_code[i] = '\0';
                                break;
                            }
                        }
                        sprintf_P(cmd, PSTR("M%d %s"), lM, wifi_acce_code); 
                        EXECUTE_GCODE(PSTR("M500"));
                        SPI_resent_wifi_info();
                    }else if(lM == 1506 || lM == 1507 || lM == 1508){
                        //M1506 wifi_gateway //M1506 M1507 M1508
                        for(int i=0; i<4; i++){
                            wifi_ip_settings[4* (lM - 1506) + i] = 0x00;
                        }
                        uint8_t uIP = 0;
                        uint8_t pointCount = 0;
                        for(int i=0; i<20; i++){
                            if(_tl_command[iFrom + 6 + i]=='.'){
                                wifi_ip_settings[4* (lM - 1506) + pointCount] = uIP;
                                pointCount++;
                                uIP = 0;
                            }
                            if(_tl_command[iFrom + 6 + i] >= '0' && _tl_command[iFrom + 6 + i]<='9'){
                                uIP = uIP * 10 + _tl_command[iFrom + 6 + i] - '0';
                            }
                        }
                        sprintf_P(cmd, PSTR("M%d %d.%d.%d.%d"), lM, wifi_ip_settings[4* (lM - 1506) + 0],wifi_ip_settings[4* (lM - 1506) + 1],wifi_ip_settings[4* (lM - 1506) + 2],wifi_ip_settings[4* (lM - 1506) + 3]);
                        TLDEBUG_PRINTLN(cmd);
                        EXECUTE_GCODE(PSTR("M500"));
                        SPI_resent_wifi_info();
                    }else if(lM == 1510){
                        //M1510
                        SPI_RestartWIFI();
                    }
                #endif //Has wifi
            }else if(lM == 290){
                //M290 babysetp
                float fZ = GCodelng('Z', iFrom, _tl_command);
                if(fZ > -999.0)
                {
                    sprintf_P(cmd, PSTR("M%d Z%s"), lM, dtostrf(fZ, 1, 3, str_1));                
                    EXECUTE_GCODE(cmd);
                }

            }else if(lM == 30){
                //M30 delete file from sd card.
                char fileNameP[13];
                NULLZERO(fileNameP);
                for(int i=0; i<12; i++){
                    fileNameP[i] = _tl_command[iFrom + 4 + i];
                }
                if(strlen(fileNameP) > 4){
                    sprintf_P(cmd, PSTR("M%d %s"), lM, fileNameP);
                }
                #if ENABLED(SDSUPPORT)
                if(strlen(cmd)){                        
                    TLDEBUG_PRINTLN(cmd);
                    EXECUTE_GCODE(cmd);
                    uint32_t wait_start = millis();
                    while (millis()-wait_start < 300) //delay 300ms
                    {
                        watchdog_refresh();
                    }
                    card.tl_ls(true);
                    sd_OK = 2;
                }
                #endif
            }
            delay(100);
        }//iLoop //lines in this command group
    }
}

void tenlog_screen_update()
{    
    static unsigned long lLastUpdateTime;
    if(millis() - lLastUpdateTime < 1000)
    {
        return;
    }

    lLastUpdateTime=millis();
    if(tl_TouchScreenType == 0)
        tenlog_screen_update_dwn();
    else if(tl_TouchScreenType == 1)
        tenlog_status_update(true);
        
}

void tl_wifi_idle()
{   
    if(file_uploading){
        wifi_upload_write_data();
        WIFI_TX_Handler(0x0C);
        delay(1);
        return;
    }
    static unsigned long lWLastUpdateTime;
    if(millis() - lWLastUpdateTime < wifi_update_interval)
        return;
    
    lWLastUpdateTime=millis();
    tenlog_status_update(false);
}


void tenlog_command_handler()
{
    if(tl_TouchScreenType == 0)
    {
        get_lcd_command(0);
        process_command_dwn();
    }
    else if(tl_TouchScreenType == 1)
    {
        get_lcd_command(1);
        process_command_gcode(tl_command);
    }
}

#if ENABLED(TL_BEEPER)
void button_light_handler(){
    static bool btLight;
    static uint32_t lastLightTime;
    uint16_t beeper_time = 500;
    if(millis() - lastLightTime < beeper_time ){
        return;
    }
    lastLightTime = millis();
    if(!card.flag.sdprinting){
        set_pwm_hw(0, 1000);
    }
    btLight = ! btLight;
    if(pre_print_file_name[0] == 0x00){
        WRITE(TL_BUTTON_LIGHT_PIN, 1);
    }else if(card.isFileOpen()){
        WRITE(TL_BUTTON_LIGHT_PIN, 1);
    }else{
        WRITE(TL_BUTTON_LIGHT_PIN, btLight);
    }
}

void tl_beeper_handler(){
    static bool beeper_state;
    static uint32_t lastBeeperTime;
    uint16_t beeper_time = 250;
    if(beeper_type == 1) beeper_time = 500;
    if(millis() - lastBeeperTime < beeper_time ){
        return;
    }

    if(beeper_count==0 && beeper_state) {
        //TLDEBUG_PRINTLN("Beeper off");
        BEEPER_OFF;
        beeper_state = false;
    }
    if(beeper_count >0){
        if(beeper_count % 2 == 1 && !beeper_state){
            //TLDEBUG_PRINTLN("Beeper on");
            BEEPER_ON;
            beeper_state = true;
        } 
        else if(beeper_count % 2 == 0 && beeper_state){
            //TLDEBUG_PRINTLN("Beeper off");
            BEEPER_OFF;
            beeper_state = false;
        }
        beeper_count--;
        lastBeeperTime = millis();
    }
}   
void start_beeper(uint8_t count, uint8_t type){
    beeper_count=count;
    beeper_type=type;
}
#endif

#
void tl_sd_abort_on_endstop_hit(){  //only for x & y
    #if ENABLED(TL_SD_ABORT_ON_ENDSTOP_HIT)
        static uint8_t last_hitX;
        static uint8_t last_hitY;
        static uint8_t last_hitZ;
        uint8_t hitX = READ(X_STOP_PIN);
        uint8_t hitY = READ(Y_STOP_PIN);
        uint8_t hitZ = READ(Z_STOP_PIN);
        if(hitX == last_hitX && hitY == last_hitY && hitZ == last_hitZ) return;
        last_hitZ = hitZ;
        last_hitX = hitX;
        last_hitY = hitY;
        if(hitX == 0 || hitY == 0 || hitZ == 0){
            if(card.flag.sdprinting){
                TLDEBUG_PRINTLN("EndStop Hited!!"); //by zyf
                tlAbortPrinting();
                #if ENABLED(TL_BEEPER)
                start_beeper(32, 0);
                #endif
            }
        }
    #endif
}

void CheckLaserFan(){
  #if ENABLED(TL_LASER_ONLY)  //by zyf auto laser fan 
    static bool LaserStatus=false;
    if(millis() - last_laser_time < LASER_FAN_DELAY && millis() > LASER_FAN_DELAY){
        if(!LaserStatus){
            WRITE(LASER_FAN_PIN, 1);
            LaserStatus = true;
        }
    }else{
        if(LaserStatus){
            WRITE(LASER_FAN_PIN, 0);
            LaserStatus = false;
        }
    }
    #endif
  }

/*
void read_blt(){
    static uint32_t temp ;
    if(millis() - temp < 500)return;
    temp = millis();
    uint8_t r = READ(Z_STOP_PIN);
    char cmd[32];
    sprintf(cmd, "BLTouch: %d", r);
    TLDEBUG_PRINTLN(cmd);
}
*/

void TL_idle(){    
    #if ENABLED(TL_BEEPER)
        button_light_handler();
        tl_beeper_handler(); 
    #endif
    tenlog_command_handler();
    tenlog_screen_update();
    #if ENABLED(HAS_WIFI)
        tl_wifi_idle();
    #endif
    tl_sd_abort_on_endstop_hit();
    CheckLaserFan();
}

#define CEND  0xFF
void TLSTJC_println(const char s[])
{
    #if DISABLED(TL_NO_SCREEN)
    TL_LCD_SERIAL.print(s);
    TLSTJC_printend();
    #endif
}

void TLSTJC_print(const char s[])
{
    #if DISABLED(TL_NO_SCREEN)
    TL_LCD_SERIAL.print(s);
    #endif
}

void TLSTJC_printend()
{    
    #if DISABLED(TL_NO_SCREEN)
    TL_LCD_SERIAL.write(CEND);
    TL_LCD_SERIAL.write(CEND);
    TL_LCD_SERIAL.write(CEND);
    TJC_DELAY;
    #endif
}

void TLSTJC_printEmptyend()
{
    #if DISABLED(TL_NO_SCREEN)
    TL_LCD_SERIAL.write(0x00);
    TLSTJC_printend();
    #endif
}

bool MTLSERIAL_available()
{
    return TL_LCD_SERIAL.available();
}

char MTLSERIAL_read()
{
    return TL_LCD_SERIAL.read();
}

#define DWN_HEAD0 0x5A
#define DWN_HEAD1 0xA5
#define DWN_WRITE 0x82
#define DWN_READ 0x83

void TenlogScreen_begin(long boud)
{
    TL_LCD_SERIAL.begin(boud);
}

void DWN_Page(int ID)
{
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x07);
    TL_LCD_SERIAL.write(DWN_WRITE);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(0x84);
    TL_LCD_SERIAL.write(0x5A);
    TL_LCD_SERIAL.write(0x01);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(ID);
    iDWNPageID = ID;
}

void DWN_Text(long ID, int Len, const char s[], bool Center)
{
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(3 + Len);
    TL_LCD_SERIAL.write(0x82);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;

    TL_LCD_SERIAL.write(ID0);
    TL_LCD_SERIAL.write(ID1);

    int iLen = strlen(s);

    if (iLen > Len - 2){
        //s = s.substring(0, Len - 2);
        TL_LCD_SERIAL.print(s);
        TL_LCD_SERIAL.write(0xFF);
        TL_LCD_SERIAL.write(0xFF);
    }else{
        if(!Center){
            TL_LCD_SERIAL.print(s);
            for (int i = 0; i < Len - iLen - 2; i++){
                TL_LCD_SERIAL.print(" ");
            }
            TL_LCD_SERIAL.write(0xFF);
            TL_LCD_SERIAL.write(0xFF);
        }else{
            int Count = 0;
            for (int i = 0; i < (Len - iLen - 2) / 2; i++){
                TL_LCD_SERIAL.print(" ");                
                Count++;
            }
            TL_LCD_SERIAL.print(s);
            Count += iLen;
            for (int i = 0; i < Len - Count - 2; i++){
                TL_LCD_SERIAL.print(" ");                
            }
            TL_LCD_SERIAL.write(0xFF);
            TL_LCD_SERIAL.write(0xFF);
        }
    }
}

void DWN_Language(int ID)
{
    DWN_Change_Icon(0x90, 0x40, ID);
    DWN_Change_Icon(0x80, 0x10, !ID);
    DWN_Change_Icon(0x80, 0x11, ID);
}

void DWN_Change_Icon(int IID0, int IID1, int ID){
    //5A A5 05 82 50 31 00 01
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x05);
    TL_LCD_SERIAL.write(0x82);
    TL_LCD_SERIAL.write(IID0);
    TL_LCD_SERIAL.write(IID1);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(ID);
}

void DWN_Get_Ver(){
    //5A A5 04 83 00 0F 01
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x04);
    TL_LCD_SERIAL.write(0x83);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(0x0F);
    TL_LCD_SERIAL.write(0x01);
}

void DWN_NORFData(long NorID, long ID, int Length, bool WR){
    int iWR = 0x5A;
    if (WR)
        iWR = 0xA5;
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x0B);
    TL_LCD_SERIAL.write(0x82);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(0x08);
    TL_LCD_SERIAL.write(iWR);
    int Nor0 = NorID / 0x10000;
    NorID = NorID % 0x10000;
    int Nor1 = NorID / 0x100;
    int Nor2 = NorID % 0x100;
    TL_LCD_SERIAL.write(Nor0);
    TL_LCD_SERIAL.write(Nor1);
    TL_LCD_SERIAL.write(Nor2);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    TL_LCD_SERIAL.write(ID0);
    TL_LCD_SERIAL.write(ID1);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(Length);
}

void DWN_VClick(int X, int Y)
{
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x0B);
    TL_LCD_SERIAL.write(0x82);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(0xD4);
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(0x04);
    TL_LCD_SERIAL.write(X / 0x100);
    TL_LCD_SERIAL.write(X % 0x100);
    TL_LCD_SERIAL.write(Y / 0x100);
    TL_LCD_SERIAL.write(Y % 0x100);
}

void DWN_RData(long ID, int DataLen)
{
    int iLen = 4;
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(iLen);
    TL_LCD_SERIAL.write(DWN_READ);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    TL_LCD_SERIAL.write(ID0);
    TL_LCD_SERIAL.write(ID1);
    TL_LCD_SERIAL.write(DataLen);
}

void DWN_Data(long ID, long Data, int DataLen)
{
    int iLen = 3 + DataLen;
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(iLen);
    TL_LCD_SERIAL.write(DWN_WRITE);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    TL_LCD_SERIAL.write(ID0);
    TL_LCD_SERIAL.write(ID1);

    if (DataLen == 4){
        if (Data > 0x1000000){
            TL_LCD_SERIAL.write(Data / 0x1000000);
            Data = Data % 0x1000000;
        }else{
            TL_LCD_SERIAL.write(0x00);
        }

        if (Data > 0x10000){
            TL_LCD_SERIAL.write(Data / 0x10000);
            Data = Data % 0x10000;
        }else{
            TL_LCD_SERIAL.write(0x00);
        }
    }

    if (Data > 0x100){
        TL_LCD_SERIAL.write(Data / 0x100);
        TL_LCD_SERIAL.write(Data % 0x100);
    }else{
        TL_LCD_SERIAL.write(0x00);
        TL_LCD_SERIAL.write(Data);
    }
}

void DWN_LED(int LED){
    TL_LCD_SERIAL.write(DWN_HEAD0);
    TL_LCD_SERIAL.write(DWN_HEAD1);
    TL_LCD_SERIAL.write(0x05);
    TL_LCD_SERIAL.write(DWN_WRITE);
    TL_LCD_SERIAL.write(0x00);
    TL_LCD_SERIAL.write(0x82);
    TL_LCD_SERIAL.write(LED);
    TL_LCD_SERIAL.write(LED);
}

#if ENABLED(POWER_LOSS_RECOVERY_TL)

void _outage() {    
    // Disable all heaters to reduce power loss
    WRITE(HEATER_0_PIN, 0);
    WRITE(HEATER_1_PIN, 0);
    WRITE(HEATER_BED_PIN, 0);
    // Save,
    if(IS_SD_PRINTING()){
        uint32_t sdPos = card.getIndex();
        settings.plr_save(sdPos, active_extruder);
    }

    if(tl_TouchScreenType == 1){
        TLSTJC_println("page shutdown");
    }else if(tl_TouchScreenType == 0){
        DWN_Page(DWN_P_SHUTDOWN);
    }
    kill(GET_TEXT(MSG_OUTAGE_RECOVERY));
}

//first get a high volt to start check.. 
void plr_outage() {
    static bool canCheck;
    if (plr_enabled){        
        if(READ(POWER_LOSS_PIN) == !POWER_LOSS_STATE && !canCheck){
            canCheck = true;
        }
        if(READ(POWER_LOSS_PIN) == POWER_LOSS_STATE && canCheck){
            _outage();
        }
        if(IS_SD_PRINTING()){
            uint32_t sdPos = card.getIndex();
            #if ENABLED(DUAL_X_CARRIAGE)
            uint8_t _dual_x_carriage_mode = dual_x_carriage_mode;
            float _duplicate_extruder_x_offset = duplicate_extruder_x_offset;
            if(_duplicate_extruder_x_offset == DEFAULT_DUPLICATION_X_OFFSET) _duplicate_extruder_x_offset = 0.0;
            #else
            float _duplicate_extruder_x_offset = 0.0;
            uint8_t _dual_x_carriage_mode = 0;
            #endif
            #ifndef ELECTROMAGNETIC_VALUE
            settings.plr_pre_save(sdPos, thermalManager.degTargetBed(), thermalManager.degTargetHotend(0), thermalManager.degTargetHotend(1), _dual_x_carriage_mode, _duplicate_extruder_x_offset, uint16_t(feedrate_mm_s * 60.0f));
            #endif
        }
    }
}

/*
void plr_outage_test(){
    if (plr_enabled){
        _outage();
    }
}
*/
void plr_setup() {
    #if ENABLED(POWER_LOSS_PULLUP)
        SET_INPUT_PULLUP(POWER_LOSS_PIN);
    #elif ENABLED(POWER_LOSS_PULLDOWN)
        SET_INPUT_PULLDOWN(POWER_LOSS_PIN);
    #else
        SET_INPUT(POWER_LOSS_PIN);
    #endif
}

#endif //POWER_LOSS_RECOVERY_TL

void my_sleep(float time){
    unsigned long now_time = millis();
    while(millis() - now_time > time * 1000){
        idle();
        planner.synchronize();          // Wait for planner moves to finish!      
    } 
}

uint8_t TLTJC_GetLastPage(){
    uint8_t lastPageID = 0;
    TLSTJC_println("sendme");
    delay(200);
    get_lcd_command(1);
    if(tl_command[0]==0x66 && tl_command[2]==0xFF && tl_command[3]==0xFF && tl_command[4]==0xFF){
    lastPageID = tl_command[1];
    TLDEBUG_PRINTLNPAIR("Page=", lastPageID);
    }
    return lastPageID;
}

void TLTJC_GetTJCVersion(){
    ZERO(tl_tjc_ver);
    TLSTJC_println("prints loading.tUIVer.txt,0");
    delay(200);
    get_lcd_command(1);
    for(int i=0;i<10;i++){
        tl_tjc_ver[i]=tl_command[i];
    }

    TLSTJC_println("prints main.vComID.val,0");
    delay(200);
    get_lcd_command(1);
    if(tl_command[1]==0x00 && tl_command[2]==0x00)
    {
        tl_com_ID = tl_command[0];
    }
}

void SyncFanSpeed(){
  #if HAS_FAN
    #if ENABLED(DUAL_X_CARRIAGE)      
        if (idex_is_duplicating()){ 
            thermalManager.set_fan_speed(0, thermalManager.common_fan_speed);
            thermalManager.set_fan_speed(1, thermalManager.common_fan_speed);
        }
        else{
            thermalManager.set_fan_speed(active_extruder, thermalManager.common_fan_speed);      
            thermalManager.set_fan_speed(1 - active_extruder, 0);      
        }
    #else
        thermalManager.set_fan_speed(0, thermalManager.common_fan_speed);
    #endif
    
    #if ENABLED(LASER_SYNCHRONOUS_M106_M107)
    planner.buffer_sync_block(BLOCK_FLAG_SYNC_FANS);
    #endif
  #endif
}

uint32_t flash_read_one(uint32_t address){
    return *(uint32_t *)address;
}

void flash_read(uint32_t *readData){
    uint32_t dataIndex;
    for(dataIndex=0; dataIndex<8; dataIndex++){
        readData[dataIndex] = flash_read_one(FLASH_READ_ADDRESS + dataIndex * 4);
    }
}

void flash_write(uint32_t *buffer){
    unsigned int i=0u;
    uint32_t u32Addr;
    EFM_Unlock();
    EFM_FlashCmd(Enable);
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));
    u32Addr = FLASH_READ_ADDRESS;
    for(i=0u; i<8; i++){
        EFM_SingleProgram(u32Addr, *(buffer+i));
        u32Addr += 4u;
    }
    EFM_Lock();
}

void flash_earea(){
    EFM_Unlock();
    EFM_FlashCmd(Enable);
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));
    EFM_SectorErase(FLASH_READ_ADDRESS);
    EFM_Lock();
}

void command_M1521(int8_t Status){
    if(Status) WRITE(LED_PIN, 1); else WRITE(LED_PIN, 0);
}



#endif  //TENLOG_TOUCH_LCD
