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

#include "../gcode.h"
#include "../../module/settings.h"
#include "../../core/serial.h"
#include "../../inc/MarlinConfig.h"

#if ENABLED(ESP32_WIFI)
#include "../../HAL/ESP32_SPI/esp32_wifi.h"
#endif

#if ENABLED(TL_LASER_ONLY)
#include "../../HAL/PWM/pwm.h"
#endif

#if ENABLED(TENLOG_TOUCH_LCD)
#include "../../lcd/tenlog/tenlog_touch_lcd.h"
#include "../../sd/cardreader.h"
#endif

/**
 * M500: Store settings in EEPROM
 */
void GcodeSuite::M500() {
  (void)settings.save();
}

/**
 * M501: Read settings from EEPROM
 */
void GcodeSuite::M501() {
  (void)settings.load();
}

/**
 * M502: Revert to default settings
 */
void GcodeSuite::M502() {
  (void)settings.reset();
  #if(HAS_WIFI)
  SPI_RestartWIFI();
  #endif
  #if ENABLED(TL_BEEPER)
    start_beeper(4, 1);
  #endif
}

#if DISABLED(DISABLE_M503)

  /**
   * M503: print settings currently in memory
   */
  void GcodeSuite::M503() {
    (void)settings.report(!parser.boolval('S', true));
  }

#endif // !DISABLE_M503

#if ENABLED(EEPROM_SETTINGS)

  #if ENABLED(MARLIN_DEV_MODE)
    #include "../../libs/hex_print.h"
  #endif

  /**
   * M504: Validate EEPROM Contents
   */
  void GcodeSuite::M504() {
    #if ENABLED(MARLIN_DEV_MODE)
      const bool dowrite = parser.seenval('W');
      if (dowrite || parser.seenval('R')) {
        uint8_t val = 0;
        int addr = parser.value_ushort();
        if (dowrite) {
          val = parser.byteval('V');
          persistentStore.write_data(addr, &val);
          SERIAL_ECHOLNPAIR("Wrote address ", addr, " with ", val);
        } else {
          if (parser.seenval('T')) {
            const int endaddr = parser.value_ushort();
            while (addr <= endaddr) {
              persistentStore.read_data(addr, &val);
              SERIAL_ECHOLNPAIR("0x", hex_word(addr), ":", hex_byte(val));
              addr++;
              safe_delay(10);
            }
            SERIAL_EOL();
          }
          else {
            persistentStore.read_data(addr, &val);
            SERIAL_ECHOLNPAIR("Read address ", addr, " and got ", val);
          }
        }
        return;
      }
    #endif

    if (settings.validate())
      SERIAL_ECHO_MSG("EEPROM OK");
  }

#endif

#if ENABLED(TENLOG_TOUCH_LCD)
  void GcodeSuite::M1521(){
    int8_t S = parser.seenval('S');
    command_M1521(S);
  }
  #if ENABLED(TL_LASER_ONLY)
    void GcodeSuite::M1522(){
      static bool weakLaserOn;
      if(!weakLaserOn){
        laser_power = 10;
        set_pwm_hw(laser_power, 1000);
        weakLaserOn = true;
        start_beeper(2, 0);
        TLDEBUG_PRINTLN(" On");
      }else{
        set_pwm_hw(0, 1000);
        laser_power = 0;
        weakLaserOn = false;
        start_beeper(4, 0);
        TLDEBUG_PRINTLN(" Off");
      }
    }

    void GcodeSuite::M1523(){
      if(!IS_SD_PRINTING()){
        start_beeper(2, 1);
        EXECUTE_GCODE("G28 XY");
        safe_delay(1000);
        EXECUTE_GCODE("G0 X2Y2");
      }
    }
  #endif

#endif