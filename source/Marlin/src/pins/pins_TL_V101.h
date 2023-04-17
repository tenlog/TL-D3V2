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

//#define BOARD_USART3_TX_PIN     PB8// WIFI
//#define BOARD_USART3_RX_PIN     PB9

//
// EEPROM
//
#define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
//#define SDCARD_EEPROM_EMULATION

#ifdef IIC_BL24CXX_EEPROM
  #ifdef TENLOG_LW
    #define IIC_EEPROM_SDA       PD13
    #define IIC_EEPROM_SCL       PD12
  #elif defined(TL_LASER_ONLY)
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
#define X_STOP_PIN         PC13// x-

#if defined(TENLOG_LW) || defined(TL_LASER_ONLY)
  //#define X_MAX_PIN        PE5// X+
  #define Y_STOP_PIN         PH2// y-
#else
  #define X_MAX_PIN          PH2// X+
  #define Y_STOP_PIN         PC3// y-
#endif

#define Z_STOP_PIN         PC15// Z- LW-采样

#if defined(TENLOG_M3) || defined(TENLOG_LW) || defined(TL_LASER_ONLY)
  #define Z_MAX_PIN          PC15// Z+ PC14
#else
  #define Z_MAX_PIN          PC14// Z+ PC14 //LW-吐舌头
#endif

#if ENABLED(TL_LASER_ONLY)
  #define RESET_PIN           PA15
  #define TL_BUTTON_PIN       PC14
  #define TL_BUTTON_LIGHT_PIN PD8
#endif

#if ENABLED(BLTOUCH)
  #define SERVO0_PIN Z_MAX_PIN
#endif

#if ENABLED(TL_BEEPER)
  #define TL_BEEPER_PIN PC3
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #if defined(TENLOG_LW) || defined(TENLOG_LASER_ONLY)
    #define FIL_RUNOUT_PIN                    PD0   // "Pulled-high"
    //#define FIL_RUNOUT2_PIN                   PE6   // "Pulled-high"
  #else
    #define FIL_RUNOUT_PIN                    PA15   // "Pulled-high" 
  #endif
#endif

//
// Steppers
//
#define X_ENABLE_PIN       PB2
#define X_STEP_PIN         PC7
#define X_DIR_PIN          PC6

#define X2_ENABLE_PIN      X_ENABLE_PIN
#define X2_STEP_PIN        PA9   //PA9
#define X2_DIR_PIN         PA8   //PA8

#define Y_ENABLE_PIN       X_ENABLE_PIN
#define Y_STEP_PIN         PB15
#define Y_DIR_PIN          PB14

#define Z_ENABLE_PIN       X_ENABLE_PIN

#define Z_STEP_PIN         PE13
#define Z_DIR_PIN          PE12
#define HEATER_BED_PIN     PE10 

#define Z2_ENABLE_PIN       X_ENABLE_PIN
#define Z2_STEP_PIN         PB13
#define Z2_DIR_PIN          PB12

#define E0_ENABLE_PIN      X_ENABLE_PIN
#define E0_STEP_PIN        PB4  //PB4
#define E0_DIR_PIN         PB3  //PB3
#define E1_ENABLE_PIN      X_ENABLE_PIN
#define E1_STEP_PIN        PB6
#define E1_DIR_PIN         PB5



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
  #ifdef TENLOG_LW
    //#define FAN_PIN               PD0   //FAN  FC1
    #define FAN2_PIN              PD3   //FAN2 FZ1
  #else
    //#define FAN_PIN               PD0   //FAN HWPWM FC1 PA1
    #define FAN2_PIN              PA0   //FAN2 FZ1
  #endif

  #define FAN1_PIN              PE8   //FC2
  #define FAN3_PIN              PE7   //FZ2
  #define HEATER_0_PIN          PA5   // HEATER0
  #define HEATER_1_PIN          PA4   // HEATER1
#endif

#define LED_PIN                 PE2   //LED 控制管脚
#ifdef TENLOG_LW
  #define FAN4_PIN                PC3   
  #define CHAMEBER_PIN            FAN4_PIN   //机箱风扇口
#else
  #define FAN4_PIN                PE3   
  #define CHAMEBER_PIN            FAN4_PIN   //机箱风扇口
#endif
//
// Temperature Sensors
//
#define TEMP_0_PIN		     PC1
#define TEMP_1_PIN		     PC2
#define TEMP_BED_PIN       PC0

#define  ADC_CHANNEL_COUNT 3u

//
// Heaters / Fans
//

#if PIN_EXISTS(FAN1)
  #define FAN_SOFT_PWM
#endif

#define REPRINT_PIN         PE11             
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
#if ENABLED(ESP32_WIFI)  
  #define SCK_PIN           PA6
  #define NSS_PIN           PB1
  #define MOSI_PIN          PA7
  #define MISO_PIN          PB0
#endif
