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

//#if TL_HIGH_SPEED
    #define TL_HIGHSPEED
//#endif

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

#define TJC_DELAY delay(3)
#define DWN_DELAY delay(5)

#ifdef TL_DEBUG
#define TLDEBUG_PRINTLNPAIR      SERIAL_ECHOLNPAIR
#define TLDEBUG_PRINTPAIR       SERIAL_ECHOPAIR
#define TLDEBUG_PRINT         SERIAL_ECHOPGM_P
#define TLDEBUG_PRINTLN       SERIAL_ECHOLNPGM_P
#define TLDEBUG_ECHO         SERIAL_ECHO
#define TLDEBUG_POS               SERIAL_POS
#define TLDEBUG_XYZ               SERIAL_XYZ
#else
#define TLDEBUG_PRINTPAIR(...)    NOOP
#define TLDEBUG_PRINTLNPAIR(...)      NOOP
#define TLDEBUG_PRINT(...)         NOOP
#define TLDEBUG_PRINTLN(...)       NOOP
#define TLDEBUG_ECHO(...)         NOOP
#define TLDEBUG_POS(...)            NOOP
#define TLDEBUG_XYZ(...)            NOOP
#endif

#define TL_ECHO SERIAL_CHAR
#define TLECHO_PRINTLNPAIR      SERIAL_ECHOLNPAIR
#define TLECHO_PRINT         SERIAL_ECHOPGM_P
#define TLECHO_PRINTLN       SERIAL_ECHOLNPGM_P

#define EXECUTE_GCODE(X)  gcode.process_subcommands_now(X)
#define ENQUEUE_GCODE(X)  queue.enqueue_one_now(X)

//void tenlog_status_screen();
void TenlogScreen_begin(long boud);
void TenlogScreen_end();
void TJCMessage(const int FromPageID, const int ToPageID, const int MessageID, const char OkValue[], const char CancleValue[], const char StartValue[], const char Message[]);

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
void process_command_gcode(long _tl_command[]);
void get_lcd_command(int ScreenType=1);

void TLSTJC_println(const char s[]);
void TLSTJC_print(const char s[]);
void TLSTJC_printend();
void TLSTJC_printEmptyend();
uint8_t TLTJC_GetLastPage();

void TlLoadingMessage(const char Message[], const int ShowType=-1, const int DelayTime=500);
void TlPageMain();
void TlIsPLR();

void tlAbortPrinting();

//void process_commands();
bool MTLSERIAL_available();
char MTLSERIAL_read();

void initTLScreen();
void tlSendSettings(bool only_wifi);
void tlResetEEPROM();

void TLFilamentRunout();
void TLSDPrintFinished();

void plr_setup();
void plr_outage();
//void plr_outage_test();
void tenlog_status_update(bool isTJC);
void command_M1521(int8_t Status);
void my_sleep(float time);
void SyncFanSpeed();
void TLTJC_GetTJCVersion();
#if DISABLED(TL_L)
void TJCPauseResumePrinting(bool PR, bool isRunOut);
#endif

//flash read write
#define FLASH_READ_ADDRESS 0x00070000
uint32_t flash_read_one(uint32_t address);
void flash_read(uint32_t *readData);
void flash_write(uint32_t *buffer);
void flash_earea();

#if ENABLED(POWER_LOSS_RECOVERY_TL)
extern bool plr_enabled;
#endif
extern unsigned long startPrintTime;

extern int tl_print_page_id;
extern int tl_print_file_id;
extern int iDWNPageID;
extern int tl_TouchScreenType;
extern bool DWNMoving;
extern bool TLMoving;
extern bool tl_busy_state;

extern bool dwn_is_last_page;
extern char file_name_list[7][13];
extern char long_file_name_list[7][27];
extern char m117_str[15];
extern char tl_hc_sn[25];
extern char tl_tjc_sn[18];
extern char tl_tjc_ver[10];
extern long tl_command[256];

#if ENABLED(TL_BEEPER)
extern uint8_t beeper_count;
extern uint8_t beeper_type;
#define BEEPER_ON WRITE(TL_BEEPER_PIN, 1)
#define BEEPER_OFF WRITE(TL_BEEPER_PIN, 0)
void start_beeper(uint8_t count, uint8_t type);
void tl_beeper_handler();
#endif

extern bool hotendOffsetChanged;
extern bool plr1stZ;
extern bool plr1stE;

//static float feedrate = 1500.0, next_feedrate, saved_feedrate;

extern int iPrintID;
//extern char * gsM117;
//extern char * gsPrinting;

extern int8_t tl_languageID;
extern int8_t tl_Sleep;
extern int8_t tl_E_FAN_SPEED;
extern int8_t tl_C_FAN_SPEED;
extern bool tl_E_FAN_CHANGED;
extern bool tl_C_FAN_CHANGED;
extern int8_t tl_E_FAN_START_TEMP;
extern int16_t tl_LASER_MAX_VALUE;
#if HAS_HOTEND
    extern int16_t tl_E_MAX_TEMP;
#endif
extern int8_t tl_ECO_MODE;
extern int8_t tl_THEME_ID;
extern int8_t tl_Light;
extern float tl_Z_HOME_POS;

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

extern uint32_t wifi_update_interval;
extern uint8_t sd_OK;

#if ENABLED(TL_L)
    extern char pre_print_file_name[13];
    extern uint32_t last_laser_time;
    extern uint16_t laser_power;
    #define LASER_FAN_DELAY 180000
#endif

#if ENABLED(TL_STEPTEST)
    extern uint32_t STEPTEST_HZ;
#endif

#if ENABLED(CONVEYOR_BELT)
    extern uint32_t belt_print_end_time_start;
    extern uint8_t  belt_next_print_file_no;
    void belt_idle();
    void belt_start();
    void belt_stop();
    extern char shortFileNameSearched[13];
#endif

extern uint8_t tl_com_ID;
extern char TJCModelNo[64];
extern float E_Pos_read;
float CalDelay(float Length, uint16_t uSpeed);

#if ENABLED(BLTOUCH)
    #if ENABLED(Z_MIN_ENDSTOP_PROBE_OFFSET)
    extern bool BLTouch_G28;
    #endif
    extern bool G29_AFTER_G28;
#endif

#if ENABLED(TL_GRBL)
    extern char grbl_arg[30];
    extern bool isHoming;
    extern uint32_t Homing_start;
    extern uint8_t tlStopped;
    extern bool weakLaserOn;
    extern float LaserPowerG1;
    extern bool grbl_hold;
    void grbl_idle();
    void grbl_report_status(bool isIDLE=false);
    extern bool wait_ok;
    extern uint32_t last_G01;
    extern bool grbl_no_button;
    extern bool grbl_laser_off;
#endif

#if ENABLED(TL_X)
    #define DEFAULT_EXCHANGE_FILA_LENGTH    50
    #define DEFAULT_EXTRA_FILA_LENGTH       25
    #define DEFAULT_RETRACT_LENGTH          8
    #define DEFAULT_SWEEPING_TIMES          2
    #define DEFAULT_SWEEPING_SPEED          9000
    #define DEFAULT_EXCHANGE_SPEED          2400
    #define DEFAULT_EXTRA_SPEED             200
    #define DEFAULT_RETRACT_SPEED           2000

    extern int8_t old_xe_atv_0;
    extern int8_t old_xe_atv_1;
    extern int8_t tl_xe_atv;
    extern bool xe_ena;
    
    extern uint16_t exchange_fila_length;
    extern uint16_t extra_fila_length;
    extern uint16_t retract_fila_length;
    extern uint16_t sweeping_times;

    extern uint16_t sweeping_speed;
    extern uint16_t exchange_speed;
    extern uint16_t extra_speed;
    extern uint16_t retract_fila_speed;
#endif

#if ENABLED(TL_V)
    extern int8_t tl_ve_atv;
#endif

#if ENABLED(RSA_TEST)
    void rsa_test();
#endif


#endif  //TENLOG_TOUCH_LCD
