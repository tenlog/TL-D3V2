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
int8_t tl_FAN2_VALUE = 80;
int8_t tl_FAN2_START_TEMP = 80;
int8_t tl_ECO_MODE = 0;

int dwnMessageID = -1;
long lLEDTimeoutCount = 0;

bool DWNMoving = false;

int tl_print_page_id = 0;
int iDWNPageID = 0;

char gsM117[30] = "";
char gsPrinting[30] = "";

long tl_command[256] = {0};
bool bLogoGot = false;

int tenlog_status_update_delay;
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

bool plr_enabled = true;
int tl_TouchScreenType = 0;

char file_name_list[7][13]={""};
char long_file_name_list[7][27]={""};
char m117_str[15] = {""};
char tl_sn[32] = {""};

bool dwn_is_last_page = false;

char cmd[32], str_1[16];

int TLPrintingStatus = 0;

#define TL_LCD_SERIAL LCD_SERIAL

#ifdef PRINT_FROM_Z_HEIGHT
bool PrintFromZHeightFound = true;
float print_from_z_target = 0.0;
#endif

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

void get_command(int ScreenType=1)
{
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
                TLDEBUG_LNPAIR("Trying TJC... ", iTryCount);
                delay(50);
                TenlogScreen_end();
                delay(50);
                TenlogScreen_begin(lBaud);
                delay(50);
                TLSTJC_printEmptyend();
                delay(50);
                sprintf_P(cmd, PSTR("tStatus.txt=\"Shake hands... %d\""), (iTryCount+1)/2);
                TLSTJC_println(cmd);                
                delay(50);
                TLSTJC_println("connect");
                delay(50);
                lScreenStart = millis();
                delay(100);
                CanTry = false;
            }

            get_command(1);

            char SerialNo[256];
            int SLoop = 0;
            NULLZERO(SerialNo);
            int iDHCount = 0;
            for(int i=0; i<100; i++){
                if(tl_command[i] == '\0') break;
                if(tl_command[i] == ',') iDHCount++;
                if(iDHCount > 6) break;
                if(iDHCount > 4 && iDHCount < 6){
                    if((tl_command[i] > 47 && tl_command[i] < 58) || (tl_command[i] > 64 && tl_command[i] < 71)){
                        SerialNo[SLoop] = tl_command[i];
                        SLoop++;
                    }
                }
            }
            if(SerialNo[0] != '\0')
                TLDEBUG_LNPAIR("Serial.. ", SerialNo);

            delay(50);

            if (strlen(SerialNo) == 16){
                TLSTJC_print("loading.sDI.txt=\"");
                TLSTJC_print(SerialNo);
                TLSTJC_println("\"");
                delay(50);
                TLSTJC_println("click btA,0");
                delay(50);
                TLSTJC_println("bkcmd=0");
                if(lBaud == 9600){
                    delay(50);
                    TLSTJC_println("bauds=115200");
                    delay(50);
                    lBaud = 115200;
                    iTryCount--;
                }else {
                    bCheckDone = true;
                    return 1;
                }
            }

        }else if(TryType == 0){
            if(CanTry){
                TLDEBUG_LNPAIR("Trying DWN... ", iTryCount);
                delay(50);
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

            get_command(0);
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
     tl_FAN2_VALUE = 80;
    tl_FAN2_START_TEMP = 80;
    tl_ECO_MODE = 0;
}

void TLVersion(){
    if(tl_TouchScreenType == 1){
        sprintf_P(cmd, PSTR("about.tVer.txt=\"%s V%s.%s\""), TL_MODEL_STR, SHORT_BUILD_VERSION, TL_SUBVERSION);
        TLSTJC_println(cmd);
    }else if(tl_TouchScreenType == 0){
        sprintf_P(cmd, PSTR("%s V%s.%s"), TL_MODEL_STR, SHORT_BUILD_VERSION, TL_SUBVERSION);
        DWN_Text(0x7200, 32, cmd);
    }
    delay(20);
}

void tlInitSetting(){
    if(tl_TouchScreenType == 1)
    {
        NULLZERO(cmd);
        sprintf_P(cmd, PSTR("main.vLanguageID.val=%d"), tl_languageID);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.nSleep.val=%d"), tl_Sleep);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.cECOMode.val=%d"), tl_ECO_MODE);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.nF2t.val=%d"), tl_FAN2_START_TEMP);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.nF2s.val=%d"), tl_FAN2_VALUE);
        TLSTJC_println(cmd);
        delay(20);

        long lX = planner.settings.axis_steps_per_mm[X_AXIS] * 100;
        long lY = planner.settings.axis_steps_per_mm[Y_AXIS] * 100;
        long lZ = planner.settings.axis_steps_per_mm[Z_AXIS] * 100;
        long lE = planner.settings.axis_steps_per_mm[E_AXIS] * 100;

        sprintf_P(cmd, PSTR("setting.xXs.val=%d"), lX);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.xYs.val=%d"), lY);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.xZs.val=%d"), lZ);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.xEs.val=%d"), lE);
        TLSTJC_println(cmd);
        delay(20);

        #ifdef FILAMENT_RUNOUT_SENSOR
        if(runout.enabled)
            TLSTJC_println("setting.cFilaSensor.val=1");
        else
            TLSTJC_println("setting.cFilaSensor.val=0");
        #endif
        delay(20);

        long lOffsetX = hotend_offset[1].x * 100;
        long lOffsetY = hotend_offset[1].y * 100 + 500;
        long lOffsetZ = hotend_offset[1].z * 100 + 200;

        sprintf_P(cmd, PSTR("setting.xX2.val=%d"), lOffsetX);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.xY2.val=%d"), lOffsetY);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("setting.xZ2.val=%d"), lOffsetZ);
        TLSTJC_println(cmd);
        delay(20);
        
        sprintf_P(cmd, PSTR("main.vXMax.val=%d"), lOffsetX / 10);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("main.vYMax.val=%d"), Y_MAX_POS * 10);
        TLSTJC_println(cmd);
        delay(20);
        sprintf_P(cmd, PSTR("main.vZMax.val=%d"), Z_MAX_POS * 10);
        TLSTJC_println(cmd);
        delay(20);        
    }else if(tl_TouchScreenType == 0){
        delay(10);
        DWN_Language(tl_languageID);
        delay(10);
        long lOffsetX = hotend_offset[1].x * 100;
        DWN_Data(0x6018, lOffsetX, 4);
        delay(10);

        long iSend = 0;
        for (int i = 0; i < 4; i++)
        {
            iSend = long(planner.settings.axis_steps_per_mm[i] * 100.0f);
            DWN_Data(0x6010 + i * 2, iSend, 4);
            delay(10);
        }

        long lOffsetY = hotend_offset[1].y * 100 + 500;
        long lOffsetZ = hotend_offset[1].z * 100 + 200;
        DWN_Data(0x6020, lOffsetY, 2);

        delay(10);
        DWN_Data(0x6021, lOffsetZ, 2);
        delay(10);

        DWN_Data(0x6023, tl_FAN2_VALUE, 2);
        delay(10);
        DWN_Data(0x6022, tl_FAN2_START_TEMP, 2);
        delay(10);

        DWN_Data(0x8015, 1, 2); //replace auto power off

        DWN_Data(0x8013, tl_ECO_MODE, 2);
        delay(10);

        #ifdef FILAMENT_RUNOUT_SENSOR
        DWN_Data(0x8014, runout.enabled, 2);
        #endif

        delay(10);
        iSend = b_PLR_MODULE_Detected + tl_languageID * 2;
        DWN_Data(0x8803, iSend, 2);
        DWN_Data(0x8806, b_PLR_MODULE_Detected, 2);
        delay(200);
    }
    TLVersion();
}

void TlPageMain(){
    delay(20);
    if(tl_TouchScreenType == 1){
        TLSTJC_println("page main");
    }
    else if(tl_TouchScreenType == 0){
        DWN_Page(DWN_P_MAIN);
    }
}

void TlIsPLR(){
    if(tl_TouchScreenType == 1){
        TLSTJC_println("sleep=0");
        delay(20);
        TLSTJC_println("page main");
        delay(20);

        if (strlen(file_name_list[6]) > 3){
            if (!card.isMounted()) card.mount();
            if (card.isMounted()) {
                if(card.fileExists(file_name_list[6])){
                    TLSTJC_println("page printing");
                    delay(10);
                    TLSTJC_println("msgbox.vaFromPageID.val=1");
                    delay(10);
                    TLSTJC_println("msgbox.vaToPageID.val=6");
                    delay(10);
                    TLSTJC_println("msgbox.vtOKValue.txt=\"M1003\"");
                    delay(10);
                    TLSTJC_println("msgbox.vtCancelValue.txt=\"M1004\"");
                    delay(10);

                    TLSTJC_print("msgbox.vtMS.txt=\"");
                    TLSTJC_print(long_file_name_list[6]);
                    TLSTJC_println("\"");
                    delay(10);
                    TLSTJC_println("msgbox.vaMID.val=3");
                    delay(10);
                    TLSTJC_println("page msgbox");
                    delay(10);
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
}

void initTLScreen(){    
    stc_efm_unique_id UID_data = EFM_ReadUID();
    //char cmd1[64];
    sprintf_P(tl_sn, "SN:%08X%08X%08X", UID_data.uniqueID1, UID_data.uniqueID2, UID_data.uniqueID3);
    TLDEBUG_LNPGM(tl_sn);

    tl_TouchScreenType = DETECT_TLS(); // check if it is tjc screen
    if(tl_TouchScreenType == 1){
	    TLSTJC_printEmptyend();
        delay(100);
        TLSTJC_println("sleep=0");
        delay(100);
        TLDEBUG_LNPGM("TL TJC Touch Screen Detected!");
    }else if(tl_TouchScreenType == 0){
        DWN_Text(0x7280, 28, tl_sn);
        
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
            get_command(0);
            process_command_dwn();
            delay(50);
        }

        DWN_Page(DWN_P_LOADING);
        delay(100);        
        TLDEBUG_LNPGM("TL DWN Touch Screen Detected!");
    }else if(tl_TouchScreenType == -1){
        //tl_TouchScreenType = 1;
        TLSTJC_printEmptyend();
        delay(100);
        TLSTJC_println("sleep=0");
        delay(100);
        //TLDEBUG_LNPGM("Tenlog touch screen not detected! ");
        kill("Tenlog touch screen not detected! ");
    }
}

void TlLoadingMessage(const char Message[], const int ShowType, const int DelayTime){
    if(tl_TouchScreenType == 1 && (ShowType==1 || ShowType==-1)){
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

    for (int i = FromPostion; i < 254 && !bIsEnd; i++)
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

void TLAbortPrinting(){
    
    #ifdef PRINT_FROM_Z_HEIGHT
    PrintFromZHeightFound = true;
    print_from_z_target = 0.0;
    #endif
    
    IF_DISABLED(NO_SD_AUTOSTART, card.autofile_cancel());
    card.endFilePrint(TERN_(SD_RESORT, true));

    queue.clear();
    quickstop_stepper();

    print_job_timer.abort();
    IF_DISABLED(SD_ABORT_NO_COOLDOWN, thermalManager.disable_all_heaters());

    wait_for_heatup = false;
    TERN_(POWER_LOSS_RECOVERY_TL, settings.plr_reset());

    #ifdef EVENT_GCODE_SD_ABORT
    
    queue.inject_P(PSTR(EVENT_GCODE_SD_ABORT));
    queue.enqueue_one_now(PSTR("G92 Y0"));
    queue.enqueue_one_now(PSTR("M107"));
    queue.enqueue_one_now(PSTR("M84"));
    TLPrintingStatus = 0;
    #endif
    if(tl_TouchScreenType == 0)
        DWN_Page(DWN_P_MAIN);
}

void TLFilamentRunout(){
    EXECUTE_GCODE("M25");
    //while(queue.has_commands_queued() || planner.movesplanned() > 1){
    //    idle();
    //}
    if(tl_TouchScreenType == 1){
        iBeepCount = 10;
        TLSTJC_println("sleep=0");
        delay(50);
        TLSTJC_println("main.vCC.val=1");
        delay(50);
        TLSTJC_println("msgbox.vaMID.val=6");
        delay(50);
        TLSTJC_println("msgbox.vaFromPageID.val=15");
        delay(50);
        TLSTJC_println("msgbox.vaToPageID.val=15");
        delay(50);
        TLSTJC_println("msgbox.vtOKValue.txt=\"M1034\"");
        delay(50);
        TLSTJC_println("msgbox.vtCancelValue.txt=\"M1034\"");
        delay(50);
        TLSTJC_println("msgbox.vtStartValue.txt=\"M1031 O1\"");        
        TLSTJC_println("page msgbox");
        delay(50);
    }
}

int feed_rate_Pause = 0;
float zPos_Pause = 0.0;
float ePos_Pause = 0.0;

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
            TLSTJC_println("main.vCC.val=1");
            delay(50);
        } 
        if(lO == 0 || lO == 1 || lO == -999){
            bool FilaRunout = false;
            if(lO == 1) FilaRunout = true;  //runout from lcd
            
            zPos_Pause = current_position[Z_AXIS];
            ePos_Pause = current_position[E_AXIS];
            feed_rate_Pause = feedrate_mm_s;

            TLSTJC_println("reload.vaFromPageID.val=6");
            delay(50);

            sprintf_P(cmd, PSTR("reload.sT1T2.txt=\"%d\""), active_extruder + 1);
            TLSTJC_println(cmd);
            //TLDEBUG_LNPGM(cmd);
            delay(50);

            sprintf_P(cmd, PSTR("reload.vaTargetTemp0.val=%d"), int(thermalManager.degTargetHotend(0) + 0.5f));
            TLSTJC_println(cmd);
            //TLDEBUG_LNPGM(cmd);
            delay(50);

            sprintf_P(cmd, PSTR("reload.vaTargetTemp1.val=%d"), int(thermalManager.degTargetHotend(1) + 0.5f));
            TLSTJC_println(cmd);
            //TLDEBUG_LNPGM(cmd);
            delay(50);

            sprintf_P(cmd, PSTR("reload.vaTargetBed.val=%d"), int(thermalManager.degTargetBed() + 0.5f));
            TLSTJC_println(cmd);
            //TLDEBUG_LNPGM(cmd);
            delay(50);
            
            sprintf_P(cmd, PSTR("reload.vaMode.val=%d"), dual_x_carriage_mode);
            TLSTJC_println(cmd);
            //TLDEBUG_LNPGM(cmd);
            delay(50);

            if (duplicate_extruder_x_offset != DEFAULT_DUPLICATION_X_OFFSET) {
                sprintf_P(cmd, PSTR("reload.vaMode2Offset.val=%d"), duplicate_extruder_x_offset);
                TLSTJC_println(cmd);
                //TLDEBUG_LNPGM(cmd);
                delay(50);
            } else{
                TLSTJC_println("reload.vaMode2Offset.val=-1");
                //TLDEBUG_LNPGM("reload.vaMode2Offset.val=-1");
            }

            if(FilaRunout){
                thermalManager.setTargetHotend(0,0);
                thermalManager.setTargetHotend(0,1);
            }

            //while (planner.has_blocks_queued() || planner.cleaning_buffer_counter) idle();
            queue.inject("G27");
            TLDEBUG_LNPGM("inject G27");

            my_sleep(3.0);
            TLSTJC_println("main.vCC.val=0");
            delay(50);
            //TLPrintingStatus = 0;
        }
    }else{
        //Resume
        TLPrintingStatus = 2;

        TLSTJC_println("main.vCC.val=1");
        delay(50);

        long lT = GCodelng('T', FromPos, tl_command);  //M1032 T0 H200 I0
        long lH = GCodelng('H', FromPos, tl_command);  //Temp 0 
        long lI = GCodelng('I', FromPos, tl_command);  //Temp 1
        sprintf_P(cmd, PSTR("M1032 T%i H%i I%i"), lT, lH, lI);
        //TLDEBUG_LNPGM(cmd);
        
        if(dual_x_carriage_mode == DXC_DUPLICATION_MODE || DXC_MIRRORED_MODE == 3){
            //TLDEBUG_LNPGM("G28 X");
            EXECUTE_GCODE("G28 X");
            delay(100);
            my_sleep(2.5);
        }

        if(lT == 0){
            if(lI > 0){
                sprintf_P(cmd, PSTR("M104 T1 S%i"), lI);
                EXECUTE_GCODE(cmd);
                //TLDEBUG_LNPGM(cmd);
                delay(100);
            }
            EXECUTE_GCODE(PSTR("T0"));
            delay(100);
            if(lH > 0){
                sprintf_P(cmd, PSTR("M109 S%i"), lH);
                EXECUTE_GCODE(cmd);
                //TLDEBUG_LNPGM(cmd);
                delay(100);
        }    
        }
        if(lT == 1){
            if(lH > 0){
                sprintf_P(cmd, PSTR("M104 T0 S%i"), lH);
                EXECUTE_GCODE(cmd);
                //TLDEBUG_LNPGM(cmd);
                delay(100);
            }
            delay(100);
            EXECUTE_GCODE(PSTR("T1"));
            delay(100);
            if(lI > 0){
                sprintf_P(cmd, PSTR("M109 S%i"), lI);
                EXECUTE_GCODE(cmd);
                //TLDEBUG_LNPGM(cmd);
                delay(100);
            }
        }

        sprintf_P(cmd, PSTR("G1 Z%f F4500"), zPos_Pause);
        EXECUTE_GCODE(cmd);
        delay(100);
        //TLDEBUG_LNPGM(cmd);

        sprintf_P(cmd, PSTR("G92 E%f"), ePos_Pause);
        EXECUTE_GCODE(cmd);
        //TLDEBUG_LNPGM(cmd);
        feedrate_mm_s = feed_rate_Pause;
        delay(100);

        runout.reset();

        EXECUTE_GCODE(PSTR("M24"));
        //TLDEBUG_LNPGM("M24");
        delay(100);
        
        TLSTJC_println("main.vCC.val=0");
        delay(50);
        TLPrintingStatus = 1;
    }
}

void TLSDPrintFinished(){    
     
    #ifdef PRINT_FROM_Z_HEIGHT
    PrintFromZHeightFound = true;
    print_from_z_target = 0.0;
    #endif

    TERN_(POWER_LOSS_RECOVERY_TL, settings.plr_reset());
    my_sleep(1.5);


    unsigned long stoptime = millis();
    long t = (stoptime - startPrintTime) / 1000;
    int hours, minutes;
    minutes = (t / 60) % 60;
    hours = t / 60 / 60;
 
    TLPrintingStatus = 0;

    if(tl_TouchScreenType == 1){
        TLSTJC_println("sleep=0");
        TLSTJC_println("msgbox.vaFromPageID.val=1");
        TLSTJC_println("msgbox.vaToPageID.val=1");
        TLSTJC_println("msgbox.vtOKValue.txt=\"\"");
        TLSTJC_println("msgbox.vaMID.val=1");

        sprintf_P(cmd, "msgbox.vtMS.txt=\"%02d:%02d\"", hours, minutes);
        TLSTJC_println(cmd);
        TLSTJC_println("page msgbox");
    }else if(tl_TouchScreenType == 0){

    }
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
        fEPos += 20;
        sprintf_P(cmd, PSTR("G1 E%f F200"), fEPos);
        EXECUTE_GCODE(cmd);        
    }
    else if (LoadUnload == 1){
        //unload
        float fEPos = current_position[E_AXIS];
        fEPos += 30;
        sprintf_P(cmd, PSTR("G1 E%f F500"), fEPos);
        EXECUTE_GCODE(cmd);
        fEPos -= 120;
        sprintf_P(cmd, PSTR("G1 E%f F3000"), fEPos);
        EXECUTE_GCODE(cmd);        
    }
}

void process_command_gcode(long _tl_command[]) {
    //Translate Gcode from TJC controller to the original gcode handler.

    long lFrom[10] = {0}; //10 lines.
    ZERO(lFrom);
    //memset(lFrom, 0, sizeof(lFrom));

    long lP = 1;
    long lLen = 0;
    for(long i=1; i<256; i++){
        if(_tl_command[i] == 0x0A)
        {
            lFrom[lP] = i + 1;
            lP++;
        }
        if(_tl_command[i] == '\0')
            break;
        lLen++;
    }

    if(lLen > 1) {
        for(int i=0; i<lP; i++) {

            int iFrom = lFrom[i];
            long lG = GCodelng('G', iFrom, _tl_command, true);
            long lT = GCodelng('T', iFrom, _tl_command, true);
            long lM = GCodelng('M', iFrom, _tl_command, true);            
            
            if(lG == 1 || lG == 0) {
                TLSTJC_println("main.vCC.val=1");
                delay(50);
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
                    sprintf_P(sE, PSTR("E%s "), dtostrf(fTemp, 1, 1, str_1));                
                }
                if(lF != -999)
                {
                    sprintf_P(sF, PSTR("F%d"), lF);                
                }
                feedrate_mm_s = lF / 60;
                sprintf_P(cmd, PSTR("G%d %s%s%s%s%s"), lG, sF, sX, sY, sZ, sE);
                EXECUTE_GCODE(cmd);
                TLSTJC_println("main.vCC.val=0");
                delay(50);
            } else if(lG == 28){
                //G28                
                TLSTJC_println("main.vCC.val=1");
                delay(50);

                long lX = GCodelng('X', iFrom, _tl_command);
                long lY = GCodelng('Y', iFrom, _tl_command);
                long lZ = GCodelng('Z', iFrom, _tl_command);

                char sX[10],sY[10],sZ[10];
                NULLZERO(sX);
                NULLZERO(sY);
                NULLZERO(sZ);

                if(lX > -999) {
                    sprintf_P(sX, PSTR("X%d "), lX);
                }
                if(lY > -999) {
                    sprintf_P(sY, PSTR("Y%d "), lY);                
                }
                if(lZ > -999) {
                    sprintf_P(sZ, PSTR("Z%d "), lZ);                
                }

                sprintf_P(cmd, PSTR("G%d %s%s%s"), lG, sX, sY, sZ);
                TLDEBUG_LNPGM(cmd);
                EXECUTE_GCODE(cmd);                    
                my_sleep(1.5);
                TLSTJC_println("main.vCC.val=0");
                delay(50);

            }else if(lT == 0 || lT == 1){
                sprintf_P(cmd, PSTR("T%d"), lT);
                EXECUTE_GCODE(cmd);
            }else if(lM == 1022){
                TLSTJC_println("main.vCC.val=1");
                delay(50);
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
                        lX = X2_MAX_POS - 81;
                        lY = Y_MAX_POS - 53;
                    }else if(lS == 2){
                        lX = 32;
                        lY = 25;
                    }else if(lS == 3){
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
                }else if(lS == 6){
                    EXECUTE_GCODE("G28 XYZ");
                    my_sleep(1.0);
                    //EXECUTE_GCODE("G1 X-50.0 Y0");
                    //my_sleep(1.0);
                }
                TLSTJC_println("main.vCC.val=0");
                delay(50);
            }else if(lM == 104){
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
                TLDEBUG_LNPGM(cmd);
            }else if(lM == 140 || lM == 220){
                //M220 //M140
                long lS = GCodelng('S', iFrom, _tl_command);
                char sS[10];
                NULLZERO(sS);
                if(lS > -999) {
                    sprintf_P(sS, PSTR("S%d "), lS);
                }
                sprintf_P(cmd, PSTR("M%d %s"), lM, sS);
                EXECUTE_GCODE(cmd);
            }else if(lM == 106){
                float fS = GCodelng('S', iFrom, _tl_command);
                long lR = GCodelng('R', iFrom, _tl_command);
                char sS[10], sR[10];
                NULLZERO(sS);
                NULLZERO(sR);
                float fR = 1.0f;
                if(lR == 1){
                    fR = 2.55f;
                }
                if(fS > -999) {
                    long lS = fS * fR;
                    sprintf_P(sS, PSTR("S%d "), lS);                
                }
                sprintf_P(cmd, PSTR("M%d %s"), lM, sS);                
                EXECUTE_GCODE(cmd);
            }else if(lM == 107){
                sprintf_P(cmd, PSTR("M%d"), lM);
                EXECUTE_GCODE(cmd);
            }else if(lM == 21){
                sprintf_P(cmd, PSTR("M%d"), lM);
                TLDEBUG_LNPGM("M21");
                EXECUTE_GCODE(cmd);
            }else if(lM == 32){
                //M32
                long lF = GCodelng('F', iFrom, _tl_command);
                //long lF = 2;
                if(lF > 0){
                    sprintf_P(cmd, PSTR("M%d %s"), lM, file_name_list[lF-1]);
                    TLPrintingStatus = 1;
                    settings.plr_fn_save(lF-1);
                    EXECUTE_GCODE(cmd);
                }
            }else if(lM == 19){
                tl_print_page_id = GCodelng('S', iFrom, _tl_command);
                TLDEBUG_LNPGM("M19");
                card.tl_ls();
            }else if(lM == 1001){
                tl_languageID = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1017){
                tl_Sleep = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1024){
                tl_ECO_MODE = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1015){
                tl_FAN2_START_TEMP = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 1023){
                #ifdef FILAMENT_RUNOUT_SENSOR
                runout.enabled = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
                #endif
            }else if(lM == 1014){
                tl_FAN2_VALUE = GCodelng('S', iFrom, _tl_command);
                EXECUTE_GCODE(PSTR("M500"));
            }else if(lM == 502){
                EXECUTE_GCODE(PSTR("M502"));
                delay(5);
                EXECUTE_GCODE(PSTR("M500"));
                delay(5);
                EXECUTE_GCODE(PSTR("M501"));
            }else if(lM == 92){
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
                TLDEBUG_LNPGM(cmd);
                EXECUTE_GCODE(cmd);
            }else if(lM == 1031){
                //pause from lcd or runout //M1031
                TJCPauseResumePrinting(true, 0);
            }else if(lM == 1032){
                //Resume from lcd
                TJCPauseResumePrinting(false, iFrom);
            }else if(lM == 1033){
                //M1033
                //abortSDPrinting
                TLAbortPrinting();
            }else if(lM == 1021){
                //Pre heat //M1021
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
            }else if(lM == 1011 || lM == 1012 || lM == 1013){
                //hoten offset 1 M1011 M1012 M1013
                long lR = GCodelng('R',iFrom, _tl_command);
                long lS = GCodelng('S',iFrom, _tl_command);
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
                        fOffset = fOffset - 2.0f;
                        hotend_offset[1].z = fOffset;
                    }
                    EXECUTE_GCODE(PSTR("M500"));
                }
            }else if(lM == 1004){
                settings.plr_reset();//cancel power loss resume.
            }else if(lM == 1003){
                //M1003
                settings.plr_recovery();//do power loss resume.
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
                #endif //PRINT_FROM_Z_HEIGHT
            }else if(lM > 1499 && lM < 1510){
                #if ENABLED(HAS_WIFI)
                if(lM == 1501){
                    int _wifi_ena = GCodelng('S', iFrom, _tl_command);
                    if(_wifi_ena != wifi_ena){
                        wifi_ena = GCodelng('S', iFrom, _tl_command);
                        EXECUTE_GCODE(PSTR("M500"));
                    }
                }else if(lM == 1504){                    
                    http_port = GCodelng('S', iFrom, _tl_command);
                    EXECUTE_GCODE(PSTR("M500"));
                }else if(lM == 1502){
                    NULLZERO(wifi_ssid);
                    for(int i=0; i<20; i++){
                        wifi_ssid[i] = _tl_command[iFrom + 6 + i];
                        if(wifi_ssid[i]==10){
                            wifi_ssid[i] = '\0';
                            break;
                        }
                    }
                    EXECUTE_GCODE(PSTR("M500"));
                }else if(lM == 1503){
                    NULLZERO(wifi_pswd);
                    for(int i=0; i<20; i++){
                        wifi_pswd[i] = _tl_command[iFrom + 6 + i]; 
                        if(wifi_pswd[i]==10){
                            wifi_pswd[i] = '\0';
                            break;
                        }
                    }
                    EXECUTE_GCODE(PSTR("M500"));
                }else if(lM == 1505){
                    if(wifi_ena == 0){
                        #if ENABLED(ESP8266_WIFI)
                        esp_wifi_init();
                        #endif
                        TLSTJC_println("page wifisetting");
                    }else{
                        TLSTJC_println("page loading");
                        kill("Restart to apply wifi! ");                
                    }
                }
                #endif
            }
            delay(100);
        }//iLoop
    }
}

void process_command_dwn()
{
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
                    tlInitSetting();
                    break;
                case 0x18:
                    lData = ConvertHexLong(tl_command, 4);
                    hotend_offset[1].x = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    tlInitSetting();
                    break;
                case 0x20:                    
                    lData = ConvertHexLong(tl_command, 4);
                    hotend_offset[1].y = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    tlInitSetting();
                    break;
                case 0x21:
                    lData = ConvertHexLong(tl_command, 4);
                    hotend_offset[1].z = (float)lData / 100.0f;
                    EXECUTE_GCODE("M500");
                    tlInitSetting();
                    break;
                case 0x23:
                    
                    tl_FAN2_VALUE = lData;
                    if (tl_FAN2_VALUE > 100)
                        tl_FAN2_VALUE = 80;
                    EXECUTE_GCODE("M500");
                    tlInitSetting();
                    break;
                case 0x22:
                    tl_FAN2_START_TEMP = lData;
                    EXECUTE_GCODE("M500");
                    tlInitSetting();
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
                    if (thermalManager.fan_speed[0] > 0)
                        EXECUTE_GCODE("M107");
                    else
                        EXECUTE_GCODE("M106 S255");
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
                    tlInitSetting();
                    break;
                case 0x25:
                    /*
                    //it is back from settings page. not in use any more
                    if (!card.isFileOpen())
                    {
                        DWN_Page(DWN_P_MAIN);
                    }
                    else
                    {
                        DWN_Page(DWN_P_PRINTING);
                    }
                    */
                    break;
#ifdef HAS_PLR_MODULE
                case 0x23:
                    if (b_PLR_MODULE_Detected)
                    {
                        tl_AUTO_OFF = !tl_AUTO_OFF;
                        DWN_Data(0x8012, tl_AUTO_OFF, 2);
                        EXECUTE_GCODE(PSTR("M500"));
                        tlInitSetting();
                    }
                    break;
#endif
#ifdef FILAMENT_RUNOUT_SENSOR
                case 0x28:
                    runout.enabled = !runout.enabled;
                    DWN_Data(0x8014, runout.enabled, 2);
                    EXECUTE_GCODE(PSTR("M500"));
                    tlInitSetting();
                    break;
#endif
                case 0x27:
                    tl_ECO_MODE = !tl_ECO_MODE;
                    DWN_Data(0x8013, tl_ECO_MODE, 2);
                    EXECUTE_GCODE(PSTR("M500"));
                    tlInitSetting();
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
                    /*
                    //in page reload, not in use any more
                    if (degTargetHotend(0) > 50)
                        command_M104(0, 0);
                    else
                        command_M104(0, pause_T0T);
                    */
                    break;
                case 0xA7:
                    /*
                    if (degTargetHotend(0) > 50)
                        command_M104(1, 0);
                    else
                        command_M104(1, pause_T1T);
                    */
                    break;
                case 0xA1:
                    /*
                    //back from relaod page
                    if (card.isFileOpen())
                    {
                        DWN_Page(DWN_P_PRINTING);
                        if (pause_T0T1 == 1)
                            enquecommand_P(PSTR("T1"));
                        else
                            enquecommand_P(PSTR("T0"));

                        _delay_ms(100);

                        if (pause_BedT > 0)
                        {
                            String strCommand = "M140 S" + String(pause_BedT);
                            char *_Command = strCommand.c_str();
                            enquecommand(_Command);
                        }
                    }
                    else
                        DWN_Page(DWN_P_TOOLS);
                    */
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
                    //TLDEBUG_LNPGM("List files");
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
                    /*
                    long lCal = CalAtv(gsDeviceID);
                    if (lCal == lAtvCode)
                    {
                        DWN_Text(0x1002, 22, gsDeviceID, false);
                        _delay_ms(5);
                        DWN_NORFData(0x000002, 0x1002, 22, true);
                        _delay_ms(500);
                        DWN_Data(0x1022, lAtvCode, 4);
                        _delay_ms(5);
                        DWN_NORFData(0x000022, 0x1022, 2, true);
                        _delay_ms(500);
                        bAtv = true;
                    }
                    else
                    {
                        DWN_Data(0x6066, 6666, 4);
                    }
                    */
                    break;
                }
            }
        }
        tl_command[0] = 0x00;
        tl_command[1] = 0x00;
    }
}

void DWN_Message(const int MsgID, const char sMsg[], const bool PowerOff){
    dwnMessageID = MsgID;
    int iSend = dwnMessageID + tl_languageID * 13;
    if (dwnMessageID == 13)
        iSend = 3 + tl_languageID * 13;

    DWN_Data(0x9052, dwnMessageID, 2);
    DWN_Data(0x9050, iSend, 2);
    delay(10);
    DWN_Text(0x7000, 32, sMsg);
    delay(10);

    if (PowerOff == 0)
        iSend = 0;
    else
        iSend = PowerOff + tl_languageID;

    DWN_Data(0x8830, iSend, 2);
    delay(10);
    DWN_Page(DWN_P_MSGBOX);
}

int iPrintID = -1;
void DWN_MessageHandler(const bool ISOK){
    switch (dwnMessageID)
    {
    case MSG_RESET_DEFALT:
    {
        if (card.isFileOpen()){
            DWN_Page(DWN_P_SETTING_PRINTING);
        }else{
            DWN_Page(DWN_P_SETTING_MAIN);
        }
        if (ISOK){
            EXECUTE_GCODE("M502");
            tlInitSetting();
        }
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
            if (file_name_list[iPrintID][0] != '\0')
            {

                if (print_from_z_target > 0)
                    PrintFromZHeightFound = false;
                else
                    PrintFromZHeightFound = true;

                if (card.flag.sdprinting)
                {
                    planner.synchronize();
                    card.closefile();
                }

                sprintf_P(cmd, PSTR("M32 %s"), file_name_list[iPrintID]);
                TLPrintingStatus = 1;
                settings.plr_fn_save(iPrintID);
                EXECUTE_GCODE(cmd);

                sprintf_P(cmd, PSTR("Printing %s"), long_file_name_list[iPrintID]);
                DWN_Text(0x7500, 32, cmd, true);
                delay(10);
                DWN_Page(DWN_P_PRINTING);
            }
        }
        else
        {
            if (print_from_z_target > 0)
                DWN_Page(DWN_P_SEL_Z_FILE);
            else
                DWN_Page(DWN_P_SEL_FILE);
        }
        break;
    case MSG_PRINT_FINISHED:
        DWN_Page(DWN_P_MAIN);
        break;
    case MSG_STOP_PRINT_TL:
        if (ISOK)
        {
            DWN_Text(0x7000, 32, " Stopping, Pls wait...");
            TLAbortPrinting();
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
        TLAbortPrinting();
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

void TL_idle()
{
    tenlog_command_handler();
    tenlog_screen_update();
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

void tenlog_command_handler()
{
    if(tl_TouchScreenType == 0)
    {
        get_command(0);
        process_command_dwn();
    }
    else if(tl_TouchScreenType == 1)
    {
        get_command(1);
        process_command_gcode(tl_command);
    }
}

void Setting_ECO_MODE(){
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

}

char * tenlog_status_update(bool isTJC)
{
    const long ln0  = current_position[X_AXIS] * 10.0f; 
    const long ln1  = current_position[Y_AXIS] * 10.0f;
    const long ln2  = current_position[Z_AXIS] * 10.0f;
    const long ln3  = current_position[E_AXIS] * 10.0f;
    const long ln4  = int(thermalManager.degTargetHotend(0) + 0.5f);
    const long ln5  = int(thermalManager.degHotend(0) + 0.5f);
    const long ln6  = int(thermalManager.degTargetHotend(1) + 0.5f);
    const long ln7  = int(thermalManager.degHotend(1) + 0.5f);
    const long ln8  = int(thermalManager.degTargetBed() + 0.5f);
    const long ln9  = int(thermalManager.degBed() + 0.5f);
    const long ln10 = (float)thermalManager.fan_speed[0] * 100.0f / 256.0f + 0.5f;
    const long ln11 = feedrate_percentage;
    
    long ln12 = card.flag.sdprinting;
    if(TLPrintingStatus == 2)
        ln12 = TLPrintingStatus;

    long ln13 = card.percentDone();
    if(ln12 == 0)
        ln13 = 0;
    if(card.isFileOpen() && ln12 == 0){
        ln13 = 0;
    }

    const long ln14 = active_extruder;
    const long ln15 = dual_x_carriage_mode;

    char cTime[10];
    NULLZERO(cTime);
    if(ln12 == 0)
    {
        sprintf_P(cTime, "%s", "00:00");
    }else{
        uint16_t time_mm = millis() / 60000 - startPrintTime / 60000;
        uint16_t time_h = time_mm/60;
        uint16_t time_m = time_mm%60;
        sprintf_P(cTime, PSTR("%02d:%02d"), time_h, time_m);
    }

    long ln17 = card.isFileOpen();
    if(ln17 == 0 && TLPrintingStatus == 2){
        ln17 = 1;
    }
    const long ln18 = thermalManager.isHeatingHotend(0);
    const long ln19 = thermalManager.isHeatingHotend(1);
    const long ln20 = thermalManager.isHeatingBed();

    #if DISABLED(HAS_WIFI)
    char wifi_status[100] = "";
    #endif
    sprintf_P(wifi_status, PSTR("%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s|%d|%d|%d|%d|"), 
        ln0, ln1, ln2, ln3, ln4, ln5, ln6, ln7, ln8, ln9, ln10, ln11, ln12, ln13, ln14, ln15, cTime, ln17, ln18, ln19, ln20);

    if(isTJC){
        TLSTJC_print("main.sStatus.txt=\"");
        TLSTJC_print(wifi_status);
        TLSTJC_println("\"");
        delay(5);
        TLSTJC_println("click btReflush,0");
        Setting_ECO_MODE();
    }
    return wifi_status;

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

void tenlog_screen_update_dwn()
{

    //if (!bAtv)
    //    return;
    if (current_position[X_AXIS] < 0)
        DWN_Data(0x6006, (current_position[X_AXIS] * 10.0f + 0x10000), 2);
    else
        DWN_Data(0x6006, current_position[X_AXIS] * 10.0f, 2);
    delay(5);

    if (current_position[Y_AXIS] < 0)
        DWN_Data(0x6007, (current_position[Y_AXIS] * 10.0f + 0x10000), 2);
    else
        DWN_Data(0x6007, current_position[Y_AXIS] * 10.0f, 2);
    delay(5);

    if (current_position[Z_AXIS] < 0)
        DWN_Data(0x6008, (current_position[Z_AXIS] * 10.0f + 0x10000), 2);
    else
        DWN_Data(0x6008, current_position[Z_AXIS] * 10.0f, 2);
    delay(5);

    DWN_Data(0x8000, thermalManager.isHeatingHotend(0), 2);
    delay(5);

    DWN_Data(0x8002, thermalManager.isHeatingHotend(1), 2);
    delay(5);

    DWN_Data(0x8004, thermalManager.isHeatingBed(), 2);
    delay(5);

    DWN_Data(0x6000, int(thermalManager.degTargetHotend(0) + 0.5f), 2);
    delay(5);

    DWN_Data(0x6001, int(thermalManager.degHotend(0) + 0.5f), 2);
    delay(5);

    DWN_Data(0x6002, int(thermalManager.degTargetHotend(1) + 0.5f), 2);
    delay(5);

    DWN_Data(0x6003, int(thermalManager.degHotend(1) + 0.5f), 2);
    delay(5);

    DWN_Data(0x6004, int(thermalManager.degTargetBed() + 0.5f), 2);
    delay(5);

    DWN_Data(0x6005, int(thermalManager.degBed() + 0.5f), 2);
    delay(5);

    //BOF For old version UI
    int iFan = (int)((float)thermalManager.fan_speed[0] / 256.0f * 100.0f + 0.5f);

    if (thermalManager.fan_speed[0] == 0)
        DWN_Data(0x8006, 0, 2);
    else
        DWN_Data(0x8006, 1, 2);

    delay(5);
    DWN_Data(0x600A, iFan, 2);
    delay(5);
    
    DWN_Data(0x602A, iMoveRate, 2);
    delay(5);
   
    DWN_Data(0x6052, feedrate_percentage, 2);
    delay(5);

    char sTime[10] = "-- :--";
    int iTimeS = 0;
    int iPercent = 0;
    if (card.flag.sdprinting)
    {
        uint16_t time = millis() / 60000 - startPrintTime / 60000;
        
        sprintf_P(sTime, "%02d :%02d", (time / 60),(time % 60));

        iPercent = card.percentDone();
        DWN_Data(0x6051, iPercent, 2);
        delay(5);
        DWN_Data(0x8820, iPercent, 2);
        delay(5);
    }
    else
    {
        DWN_Data(0x6051, 0, 2);
        delay(5);
        DWN_Data(0x8820, 0, 2);
        delay(5);
        iPercent = 0;
        iTimeS = 1;
    }

    DWN_Data(0x8840, card.flag.sdprinting + tl_languageID * 3, 2);
    delay(5);
    DWN_Data(0x8842, card.flag.sdprinting, 2);
    delay(5);

    DWN_Text(0x7540, 8, sTime);
    delay(5);

    DWN_Data(0x8841, iTimeS, 2);
    delay(5);
    
    Setting_ECO_MODE();
    
    static int siCM;
    int iCM;

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
        delay(5);
    }
    siCM = iCM;

    int iMode = (dual_x_carriage_mode - 1) + tl_languageID * 3;
    DWN_Data(0x8801, iMode, 2);
    DWN_Data(0x8804, (dual_x_carriage_mode - 1), 2);
    delay(5);

    int iAN = active_extruder + tl_languageID * 2;
    DWN_Data(0x8802, iAN, 2); // is for UI V1.3.6
    DWN_Data(0x8805, active_extruder, 2);
    delay(5);
    
    /*
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
            delay(5);
        }
    }
    */
    #ifdef PRINT_FROM_Z_HEIGHT
    DWN_Data(0x6041, (long)(print_from_z_target * 10.0f), 2);
    delay(5);
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
    }
}

#define CEND  0xFF
void TLSTJC_println(const char s[])
{
    TL_LCD_SERIAL.print(s);
    TLSTJC_printend();
}

void TLSTJC_print(const char s[])
{
    TL_LCD_SERIAL.print(s);
}

void TLSTJC_printend()
{
    TL_LCD_SERIAL.write(CEND);
    TL_LCD_SERIAL.write(CEND);
    TL_LCD_SERIAL.write(CEND);
}

void TLSTJC_printEmptyend()
{
    TL_LCD_SERIAL.write(0x00);
    TLSTJC_printend();
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
        settings.plr_save(sdPos, thermalManager.degTargetHotend(0), thermalManager.degTargetHotend(1), active_extruder, thermalManager.fan_speed[0], current_position.z, current_position.e);
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
            float _duplicate_extruder_x_offset = duplicate_extruder_x_offset;
            if(_duplicate_extruder_x_offset == DEFAULT_DUPLICATION_X_OFFSET) _duplicate_extruder_x_offset = 0.0;
            settings.plr_pre_save(sdPos, thermalManager.degTargetBed(), dual_x_carriage_mode, _duplicate_extruder_x_offset, uint16_t(feedrate_mm_s * 60.0f));
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
        //delay(10);
        idle();
    } 
    planner.synchronize();          // Wait for planner moves to finish!      
}

#endif  //TENLOG_TOUCH_LCD
