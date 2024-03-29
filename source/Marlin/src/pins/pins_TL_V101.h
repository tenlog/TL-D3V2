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

#ifndef HC32F46x
  #error "Oops! Select an HC32F46x board in 'options > c/c++->defines.'"
#endif

#define BOARD_INFO_NAME      "TENLOG v3.1.1"
#define DEFAULT_MACHINE_NAME "TENLOG 3D"

#define BOARD_NO_NATIVE_USB
/* LED0 Port/Pin definition */
#define  LED0          PB7 // TEST

//#define OLD_PIN

/*
 * SDIO Pins
 */
#define BOARD_SDIO_D0 			PC8
#define BOARD_SDIO_D1 			PC9
#define BOARD_SDIO_D2 			PC10
#define BOARD_SDIO_D3 			PC11
#define BOARD_SDIO_CLK 			PC12
#define BOARD_SDIO_CMD 		  PD2
#define BOARD_SDIO_DET 			PE14

// USARTS

#define BOARD_USART2_TX_PIN     PA2// debug
#define BOARD_USART2_RX_PIN     PA3

#define BOARD_USART1_TX_PIN     PC4// LCD PC4
#define BOARD_USART1_RX_PIN     PC5

//#define BOARD_USART3_TX_PIN   PB8// WIFI
//#define BOARD_USART3_RX_PIN   PB9

//
// EEPROM
//
#define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
//#define SDCARD_EEPROM_EMULATION

#ifdef IIC_BL24CXX_EEPROM
  #if defined(TL_W) || defined(TL_L)
    #define IIC_EEPROM_SDA       PD12 
    #define IIC_EEPROM_SCL       PD13 
  #else
    #define IIC_EEPROM_SDA       PA12
    #define IIC_EEPROM_SCL       PA11
  #endif
  #define MARLIN_EEPROM_SIZE               0x800  // 2Kb (24C16)
#endif

//
// Limit Switches
//

#if defined(DUAL_X_CARRIAGE)
  #define X_MAX_PIN          PH2// X+
  #define Y_STOP_PIN         PC3// y-
  #define X_STOP_PIN         PC13// x-
  #define Z_STOP_PIN         PC15// Z-
  #if defined(TL_M3)
    #define Z_MAX_PIN        PC15//
    //PC14 to hw pwm for bl touch
  #elif defined(BLTOUCH)
    //#define Z_MAX_PIN        PC15// 14Z+ //不能有这行，否则Z回零第一次触碰之后不会抬高。很诡异的错误！
    //PC14 to hw pwm for bl touch
  #else
    #define Z_MAX_PIN        PC14// 14Z+
  #endif
#elif defined(TL_L)
  #define X_MIN_PIN           PC13// x-
  #define X_MAX_PIN           PE11// x+
  #define Y_STOP_PIN          PC3// y-
  #define Y_MAX_PIN           PE9// y+
  #define Z_STOP_PIN          PC15// Z-
  //#define Z_MAX_PIN         PC15// Z-
#elif defined(BLTOUCH) && defined(SINGLE_HEAD)
  #if defined(TL_S) 
    #define X_MAX_PIN         PH2// x+
  #elif defined(TL_W)
    #define X_MAX_PIN        PC13// x+
  #else
    #define X_MAX_PIN         PH2// x+
  #endif
  #define Y_STOP_PIN         PC3// y-
  #define Z_STOP_PIN         PC15// Z- 
  //PC14 to hw pwm for bl touch
#else
  #define X_STOP_PIN         PC13// x-
  #define Y_STOP_PIN         PC3// y-
  #define Z_STOP_PIN         PC15// Z-
  #define Z_MAX_PIN          PC15// Z-
#endif

#ifdef TL_L
  #define LASER_FAN_PIN       PD3
  #define RESET_PIN           PA15 
  #define TL_BUTTON_START_PIN       PC14
  #define TL_BUTTON_HOME_PIN        PE6
  #define TL_BUTTON_LIGHT_PIN PD8
#endif

#ifdef TL_BEEPER
  #define TL_BEEPER_PIN   PE3//PC3
#endif

//
// Filament Runout Sensor
//
#if defined(TL_W) 
  #define FIL_RUNOUT_PIN                    PA0   // "Pulled-high"  
#elif defined(TL_L)
  #define FIL_RUNOUT_PIN                    PD0   // "Pulled-high"
#else
  #define FIL_RUNOUT_PIN                    PA15   // "Pulled-high" 
#endif

//
// Steppers
//

#ifdef TL_SPI_DRIVE
  //#define X_CS_PIN          PA9
  #define Y_CS_PIN          PB15
  //#define Z_CS_PIN          PB15
  //#define X2_CS_PIN         PB14
  //#define Z2_CS_PIN         PC6

  #define TMC_SW_MOSI       PA7
  #define TMC_SW_MISO       PB0
  #define TMC_SW_SCK        PA6
#endif

#if defined(TL_STEPTEST)
  #define X_ENABLE_PIN       PB8
  #define XX_ENABLE_PIN      PB2
  #define XX_STEP_PIN        PA9
  #define XX_DIR_PIN         PA8
#else
  #define X_ENABLE_PIN     PB2
  #if !defined(SINGLE_HEAD)
    #define X2_ENABLE_PIN  X_ENABLE_PIN
    #define X2_STEP_PIN    PA9
    #define X2_DIR_PIN     PA8
  #endif
#endif

#define Y_ENABLE_PIN       X_ENABLE_PIN
#define Z_ENABLE_PIN       X_ENABLE_PIN
#define Z2_ENABLE_PIN      X_ENABLE_PIN
#define E0_ENABLE_PIN      X_ENABLE_PIN
#define E1_ENABLE_PIN      X_ENABLE_PIN

#ifdef TL_S
  #define X_STEP_PIN         PA9
  #define X_DIR_PIN          PA8
#else
  #define X_STEP_PIN         PC7
  #define X_DIR_PIN          PC6
#endif

#define Y_STEP_PIN         PB15
#define Y_DIR_PIN          PB14

#define Z_STEP_PIN         PE13
#define Z_DIR_PIN          PE12



#ifdef TL_W
  #define E0_STEP_PIN        PB13
  #define E0_DIR_PIN         PB12
  #define E1_STEP_PIN        PB4
  #define E1_DIR_PIN         PB3
  #define Z2_STEP_PIN        PB6
  #define Z2_DIR_PIN         PB5
#elif !defined(TL_L)
  #define Z2_STEP_PIN        PB13
  #define Z2_DIR_PIN         PB12
  #ifdef TL_X
    #define E0_STEP_PIN        PB6
    #define E0_DIR_PIN         PB5
  #else
    #define E0_STEP_PIN        PB4
    #define E0_DIR_PIN         PB3
  #endif
  #define E1_STEP_PIN        PB6
  #define E1_DIR_PIN         PB5
#endif

//FANS
#ifdef ELECTROMAGNETIC_VALUE
  #define ELECTROMAGNETIC_VALUE_ON            1
  #define ELECTROMAGNETIC_VALUE_OFF           0
  #define ELECTROMAGNETIC_VALUE_0_PIN         PA0
  //#define ELECTROMAGNETIC_VALUE_1_PIN       FAN3_PIN
  #define ELECTROMAGNETIC_VALUE_MOTO0_PIN     PA5
  #define ELECTROMAGNETIC_VALUE_LED0_PIN      PA1
  //#define ELECTROMAGNETIC_VALUE_LED1_PIN    HEATER_1_PIN

  #define FAN_PIN               -1   
  #define FAN1_PIN              -1   
  #define HEATER_0_PIN          -1   
  #define HEATER_1_PIN          -1   
  #define FAN2_PIN              -1 
  #define FAN3_PIN              -1   
#else
  //#define FAN_PIN               PD0   //FAN  FC1 now use hw pwm
  //#define FAN1_PIN              PE8   //FC2 use hw pwm

  #if defined(TL_W)
    #define FAN2_PIN              PD3   //FAN2 FZ1
    #define FAN3_PIN              PE7   //FZ2
  #elif defined(TL_L)
    #define FAN_LASER_PIN         PD3   //FAN2 For laser
    #define COOLANT_MIST_PIN      PE7
  #else
    #define FAN2_PIN              PA0   //FAN2 FZ1  //PA0
    #define FAN3_PIN              PE7   //FZ2
  #endif

  #ifndef TL_L
    #define HEATER_BED_PIN     PE10 
    #define HEATER_0_PIN          PA5   // HEATER0
    #define HEATER_1_PIN          PA4   // HEATER1
  #endif
#endif

#define LED_PIN                 PE2   //LED 控制管脚
#ifdef TL_W
  #define FAN4_PIN                PC3   
  #define CHAMEBER_PIN            FAN4_PIN   //机箱风扇口
#elif defined(TL_L)

#else
  #define FAN4_PIN                PE3   
  #define CHAMEBER_PIN            FAN4_PIN   //机箱风扇口
#endif
//
// Temperature Sensors
//
#ifndef TL_L
  #define TEMP_0_PIN		     PC1
  #define TEMP_1_PIN		     PC2    //PC2
  #define TEMP_BED_PIN       PC0
#endif

#define  ADC_CHANNEL_COUNT 3u

//
// Heaters / Fans
//

#if PIN_EXISTS(FAN2) || PIN_EXISTS(FAN_LASER_PIN)
  #define FAN_SOFT_PWM
#endif

//#define REPRINT_PIN         PE11             
#define POWEROFF_PIN        PB10
//#define BREAK_PIN           PA15

//
// SD Card
//
#define SD_DETECT_PIN       BOARD_SDIO_DET
#define SDCARD_CONNECTION   ONBOARD
#define ONBOARD_SPI_DEVICE  1
#define ONBOARD_SD_CS_PIN   PE4   // SDSS
#define SDIO_SUPPORT
#define NO_SD_HOST_DRIVE    // This board's SD is only seen by the printer

//Spi wifi
#if defined(ESP32_WIFI)
  #define SCK_PIN           PA6
  #define NSS_PIN           PB1
  #define MOSI_PIN          PA7
  #define MISO_PIN          PB0
#endif

#ifdef CONVEYOR_BELT
  #define BELT_STEP_PIN     PB0
  #define BELT_DIR_PIN      PB1
  #define BELT_ENABLE_PIN   PA6 //?PA7
#endif
