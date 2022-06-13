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
#pragma once

#if ENABLED(TENLOG_TOUCH_LCD)

#include "../../inc/MarlinConfig.h"
#include "../../core/serial.h"

//Setting for DWIN touch screen
#define DWN_P_LOADING 21
#define DWN_P_MAIN 41
#define DWN_P_TOOLS 43
#define DWN_P_ABOUT 31
#define DWN_P_SETTING_MAIN 45
#define DWN_P_SETTING_PRINTING 69
#define DWN_P_MOVE 47
#define DWN_P_SEL_Z_FILE 49
#define DWN_P_SEL_FILE 70
#define DWN_P_PRINTING 0x33
#define DWN_P_TEMP 53
#define DWN_P_MODE 57
#define DWN_P_RELOAD 59
#define DWN_P_SHUTDOWN 61
#define DWN_P_PRINTZ 63
#define DWN_P_MSGBOX 14

#define DWN_TXT_VERSION 0x10
#define DWN_TXT_LOADING 0x00
#define DWN_TXT_FILE0 0x51
#define DWN_TXT_PRINTFILE 0x20
#define DWN_TXT_PERCENT 0x21

#define DWN_LED_ON 74
#define DWN_LED_OFF 03
#define DWN_LED_TIMEOUT 300

#define MSG_START_PRINT_TL 0
#define MSG_PRINT_FINISHED 1
#define MSG_POWER_OFF 2
#define MSG_POWER_LOSS_DETECTED 3
#define MSG_RESET_DEFALT 4
#define MSG_STOP_PRINT_TL 5
#define MSG_FILAMENT_RUNOUT 6
#define MSG_INPUT_Z_HEIGHT 7

#define MSG_NOZZLE_HEATING_ERROR 8
#define MSG_NOZZLE_HIGH_TEMP_ERROR 9
#define MSG_NOZZLE_LOW_TEMP_ERROR 10
#define MSG_BED_HIGH_TEMP_ERROR 11
#define MSG_BED_LOW_TEMP_ERROR 12
#define MSG_TL_PRINT_PAUSED 13

#define ECO_HEIGHT 5

#define NULLZERO(a) memset(a,'\0',sizeof(a))

#ifdef TL_DEBUG
#define TLDEBUG_PRINTLNPAIR      SERIAL_ECHOLNPAIR
#define TLDEBUG_PRINT         SERIAL_ECHOPGM_P
#define TLDEBUG_PRINTLN       SERIAL_ECHOLNPGM_P
#else
#define TLDEBUG_PRINTLNPAIR(...)      NOOP
#define TLDEBUG_PRINT(...)         NOOP
#define TLDEBUG_PRINTLN(...)       NOOP
#endif
#define TL_ECHO SERIAL_CHAR

#define EXECUTE_GCODE(X)  gcode.process_subcommands_now(X)

//void tenlog_status_screen();
void TenlogScreen_begin(long boud);
void TenlogScreen_end();
void TJCMessage(const int FromPageID, const int ToPageID, const int MessageID, const char OkValue[], const char CancleValue[], const char Message[]);


void DWN_MessageHandler(bool ISOK);
void DWN_Message(int MsgID, const char sMsg[], bool PowerOff);

void DWN_LED(int LED) ;
void DWN_Page(int ID);
void DWN_Text(long ID, int Len, const char s[], bool Center = false);
void DWN_Language(int ID);
void DWN_Data(long ID, long Data, int DataLen);
void process_command_dwn();
void DWN_NORFData(long NorID, long ID, int Lenth, bool WR);
void DWN_RData(long ID, int DataLen);
void DWN_VClick(int X, int Y);
void DWN_Change_Icon(int IID0, int IID1, int ID);
void DWN_Get_Ver();

void TL_idle();
void tenlog_command_handler();
void tenlog_screen_update();
void tenlog_screen_update_dwn();
char * tenlog_status_update(bool isTJC);
void process_command_gcode(long _tl_command[]);
void get_command(int ScreenType=1);

//void Init_TLScreen_tjc();
//void Init_TLScreen_dwn();

//void CheckTempError_tjc();

void TLSTJC_println(const char s[]);
void TLSTJC_print(const char s[]);
void TLSTJC_printend();
void TLSTJC_printEmptyend();

void TlLoadingMessage(const char Message[], const int ShowType=-1, const int DelayTime=500);
void TlPageMain();
void TlIsPLR();

//void process_commands();
bool MTLSERIAL_available();
char MTLSERIAL_read();

void initTLScreen();
void tlInitSetting();
void tlResetEEPROM();

void TLFilamentRunout();
void TLSDPrintFinished();

void plr_setup();
void plr_outage();
//void plr_outage_test();

void my_sleep(float time);

extern bool plr_enabled;

extern unsigned long startPrintTime;

extern int tl_print_page_id;
extern int iDWNPageID;
extern int tl_TouchScreenType;
extern bool DWNMoving;

extern bool dwn_is_last_page;
extern char file_name_list[7][13];
extern char long_file_name_list[7][27];
extern char m117_str[15];
extern char tl_sn[32];
extern long tl_command[256];
extern char printer_status[100];

extern bool hotendOffsetChanged;

//static float feedrate = 1500.0, next_feedrate, saved_feedrate;

extern int iPrintID;
//extern char * gsM117;
//extern char * gsPrinting;

extern int8_t tl_languageID;
extern int8_t tl_Sleep;
extern int8_t tl_E2_FAN_START_TEMP;
extern int8_t tl_E1_FAN_START_TEMP;
extern int8_t tl_ECO_MODE;
extern int8_t tl_THEME_ID;
extern int8_t tl_Light;

extern int dwnMessageID;
extern long lLEDTimeoutCount;

extern bool bLogoGot;

extern int iBeepCount ;
extern bool b_PLR_MODULE_Detected ;
extern int iMoveRate ;
extern bool bInited ;
extern bool bHeatingStop ;

extern bool bAtvGot0 ;
extern bool bAtvGot1;
extern bool bAtv;
extern int iOldLogoID;
extern long lAtvCode;
extern int TLPrintingStatus;

#ifdef PRINT_FROM_Z_HEIGHT
extern bool PrintFromZHeightFound;
extern float print_from_z_target;
#endif

#endif  //TENLOG_TOUCH_LCD
