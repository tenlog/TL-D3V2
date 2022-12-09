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
			0x0B = sned file end.
*/

#pragma once

#ifdef ESP32_WIFI

#define BUFFER_SIZE 1024
#define WIFI_MSG_LENGTH BUFFER_SIZE - 4

extern char wifi_ssid[20];
extern char wifi_pswd[20];
extern char wifi_acce_code[20];
extern uint8_t wifi_ip_settings[20];
extern uint8_t wifi_mode;
extern uint16_t http_port;
extern bool wifi_connected;
extern bool wifi_resent;
extern int8_t wifiFirstSend;

extern uint8_t wifi_printer_status[WIFI_MSG_LENGTH];
extern uint8_t wifi_printer_settings[WIFI_MSG_LENGTH];
extern uint8_t wifi_file_name[WIFI_MSG_LENGTH];

extern uint8_t wifi_version[3];
extern bool file_uploading;
extern bool file_writing;

#define HEAD_OK(a)	(a[0]==0xFF && a[1]==0xFF)

/* SPI_SCK Port/Pin definition */
#define SPI1_SCK_PORT                    (PortA)
#define SPI1_SCK_PIN                     (Pin06)
#define SPI1_SCK_FUNC                    (Func_Spi1_Sck)

/* SPI_NSS Port/Pin definition */
#define SPI1_NSS_PORT                    (PortB)
#define SPI1_NSS_PIN                     (Pin01)
#define SPI1_NSS_HIGH()                  (PORT_SetBits(SPI1_NSS_PORT, SPI1_NSS_PIN))
#define SPI1_NSS_LOW()                   (PORT_ResetBits(SPI1_NSS_PORT, SPI1_NSS_PIN))

/* SPI_MOSI Port/Pin definition */
#define SPI1_MOSI_PORT                   (PortA)
#define SPI1_MOSI_PIN                    (Pin07)
#define SPI1_MOSI_FUNC                   (Func_Spi1_Mosi)

/* SPI_MISO Port/Pin definition */
#define SPI1_MISO_PORT                   (PortB)
#define SPI1_MISO_PIN                    (Pin00)
#define SPI1_MISO_FUNC                   (Func_Spi1_Miso)

/* SPI unit and clock definition */
#define SPI1_UNIT                        (M4_SPI1)
#define SPI1_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI1)

//spi DMA
//#define SPI_DMA_UNIT                     (M4_DMA2)
//#define SPI_DMA_CLOCK_UNIT               (PWC_FCG0_PERIPH_DMA1)
//#define SPI_DMA_TX_CHANNEL               (DmaCh0)
//#define SPI_DMA_TX_TRIG_SOURCE           (EVT_SPI1_SPII)//(EVT_SPI1_SPTI)     

void WIFI_InitGPIO(void);
void WIFI_InitSPI1(void);
uint8_t SPI_RW(M4_SPI_TypeDef *SPIx, uint8_t data);

void WIFI_Init(void);
void SPI_ConnectWIFI();
void SPI_RestartWIFI();

void WIFI_TX_Handler(int8_t control_code);

//uint8_t get_control_code();
//void get_data_code(uint8_t control_code);

//void WIFI_InitDMA(void);

void wifiResetEEPROM();
#endif