/**
 * * Marlin 3D Printer Firmware
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
 * **/

/**
 * 
 * M1501: Set SSID For tenlog wifi
 * 
 * Copyright (c) Tenlog 3D / Zyf
 * **/
#include "../gcode.h"
#if ENABLED(HAS_WIFI)
#include "../../MarlinCore.h"

#if ENABLED(ESP8266_WIFI)
#include "../../HAL/ESP8266_AT/esp8266_wifi.h"
#elif ENABLED(ESP32_WIFI)
#include "../../HAL/ESP32_SPI/esp32_wifi.h"
#endif

#include "../../lcd/tenlog/tenlog_touch_lcd.h"

void GcodeSuite::M1501() {
    const uint8_t uwifiMode = parser.boolval('S');
    wifi_mode = uwifiMode;
}

void GcodeSuite::M1502() {
    sprintf_P(wifi_ssid, "%s", parser.string_arg);
}

void GcodeSuite::M1503() {
    sprintf_P(wifi_pswd, "%s", parser.string_arg);
}

void GcodeSuite::M1505() {
    sprintf_P(wifi_acce_code, "%s", parser.string_arg);
}

void GcodeSuite::M1504() {
    const uint32_t uhttpport = parser.boolval('S');
    http_port = uhttpport;
}

void GcodeSuite::M1510() {
    SPI_ConnectWIFI();
}

#endif
