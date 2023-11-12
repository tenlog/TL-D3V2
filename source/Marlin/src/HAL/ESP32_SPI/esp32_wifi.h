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

//By zyf tenlog 

/*
	MOSI: From mainboard to wifi
	control Code
	 		0x01 = wifi_mode
	 		0x02 = http_port
			0x03 = wifi_ssid
			0x04 = wifi_pswd
			0x05 = asse_code
			0x06 = apply
			0x07 = printer status
			0x08 = sn
			0x09 = settings
			0x0A = file_names
			0x0B = reboot
			0x0C = received file data.

	MISO: From wifi to mainboard
	control Code
	 		0x01 = request wifi connect
	 		0x02 = http_port_ok
			0x03 = wifi_ssid_ok
			0x04 = wifi_pswd_ok
			0x05 = asse_code_ok
			0x06 = ip
			0x07 = execute gcode
			0x08 = wifi version
			0x09 = start send file data
			0x0A = file data.
			0x0B = send file end.
*/

#pragma once

#ifdef ESP32_WIFI

#define WIFI_BUFFER_SIZE 1024
#define WIFI_MSG_LENGTH WIFI_BUFFER_SIZE-4
#define WIFI_FILE_DATA_LENGTH WIFI_MSG_LENGTH-4

extern char wifi_ssid[20];
extern char wifi_pswd[20];
extern char wifi_acce_code[20];
extern uint8_t wifi_ip_settings[20];
extern uint8_t wifi_mode;
extern uint16_t http_port;
extern bool wifi_connected;
extern bool wifi_resent;
extern uint8_t wifiFirstSend;

extern uint8_t wifi_printer_status[WIFI_MSG_LENGTH];
extern uint8_t wifi_printer_settings[WIFI_MSG_LENGTH];
extern uint8_t wifi_file_name[WIFI_MSG_LENGTH];

extern uint8_t wifi_version[3];

extern bool file_uploading;
extern bool file_writing;
extern uint8_t upload_file_data1[WIFI_FILE_DATA_LENGTH];
//extern uint8_t upload_file_data2[WIFI_FILE_DATA_LENGTH];
//extern uint8_t upload_switch_flag;
void wifi_upload_write_data();
extern uint32_t received_file_block_id;
extern uint32_t resend_file_block_id;

#define HEAD_OK(a)	(a[0]==0xFF && a[1]==0xFF)

void WIFI_Init(void);
void SPI_ConnectWIFI();
void SPI_RestartWIFI();
void SPI_resent_wifi_info();

void WIFI_TX_Handler(int8_t control_code);

void wifiResetEEPROM();
#endif
