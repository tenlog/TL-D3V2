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

#ifdef ESP8266_WIFI

extern char wifi_ssid[20];
extern char wifi_pswd[20];
extern char wifi_status[100];
extern uint8_t wifi_ena;
extern uint32_t http_port;

#define DEBUG_ESP8266_AT_WEBSERVER_PORT MYSERIAL1
// Debug Level from 0 to 4
#define _ESP_AT_LOGLEVEL_       0
#define SHIELD_TYPE           "ESP8266-AT Library" 
#define ESP8266_AT_USE_STM32      true
#define EspSerial WIFI_SERIAL
#define BOARD_TYPE      "V2.0.3"
#define BOARD_NAME      "TL BOARD"

void esp_wifi_init();
void esp_wifi_idle();
void wifiResetEEPROM();
#endif
