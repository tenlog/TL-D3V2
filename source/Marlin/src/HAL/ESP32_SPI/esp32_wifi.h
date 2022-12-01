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

//https://blog.csdn.net/ZCShouCSDN/article/details/118597633?ops_request_misc=&request_id=&biz_id=102&utm_term=hc32f460%20spi%20dma%20%E4%BB%8E%E6%9C%BA&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-0-118597633.142^v63^pc_rank_34_queryrelevant25,201^v3^control,213^v2^t3_esquery_v1&spm=1018.2226.3001.4187
/*
	MISO:	from mainboard to wifi.
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
			0x0C = sent null message to spi slave to receive file data
	MOSI:	from wifi to mainboard.
	control Code
	 		0x01 = request wifi connect
	 		0x02 = http_port_ok
			0x03 = wifi_ssid_ok
			0x04 = wifi_pswd_ok
			0x05 = asse_code_ok
			0x06 = ip
			0x07 = execute gcode
			0x08 = wifi version
			0x09 = start file transfer
			0x0A = write file data
			0x0B = end file transfer
*/

#pragma once

#ifdef ESP32_WIFI

#define SPI_BUFFER_SIZE 256
#define WIFI_DATA_LENGTH SPI_BUFFER_SIZE - 4
#define WIFI_MSG_LENGTH WIFI_DATA_LENGTH

extern uint8_t spi_tx[SPI_BUFFER_SIZE];
extern uint8_t spi_rx[SPI_BUFFER_SIZE];
extern uint8_t spi_rx1[SPI_BUFFER_SIZE];

extern char wifi_ssid[20];
extern char wifi_pswd[20];
extern char wifi_acce_code[20];
extern uint8_t wifi_ip_settings[20];
extern uint8_t wifi_mode;
extern uint16_t http_port;
extern bool wifi_connected;
extern bool wifi_resent;
extern int16_t wifiFirstSend;

extern uint8_t wifi_printer_status[WIFI_MSG_LENGTH];
extern uint8_t wifi_printer_settings[WIFI_MSG_LENGTH];
extern uint8_t wifi_file_name[WIFI_MSG_LENGTH];

extern uint8_t wifi_writing_file_data[WIFI_DATA_LENGTH];
extern char wifi_writing_file_name[26];
extern bool wifi_uploading_file;

extern uint8_t wifi_version[3];

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
#define SPI1_NSS_FUNC                    (Func_Spi1_Nss0)

/* SPI_MISO Port/Pin definition */
#define SPI1_MISO_PORT                   (PortB)
#define SPI1_MISO_PIN                    (Pin00)
#define SPI1_MISO_FUNC                   (Func_Spi1_Miso)

/* SPI_MOSI Port/Pin definition */
#define SPI1_MOSI_PORT                   (PortA)
#define SPI1_MOSI_PIN                    (Pin07)
#define SPI1_MOSI_FUNC                   (Func_Spi1_Mosi)

/* SPI unit and clock definition */
#define WIFI_SPI_UNIT                    (M4_SPI1)
#define WIFI_SPI_UNIT_CLOCK              (PWC_FCG1_PERIPH_SPI1)

//spi DMA
#define WIFI_DMA_UNIT                     (M4_DMA2)
#define WIFI_DMA_CLOCK_UNIT               (PWC_FCG0_PERIPH_DMA2)
#define WIFI_DMA_TX_CHANNEL               (DmaCh1)
#define WIFI_DMA_RX_CHANNEL               (DmaCh0)
#define WIFI_DMA_TX_TRIG_SOURCE           (EVT_SPI1_SRTI)
#define WIFI_DMA_RX_TRIG_SOURCE           (EVT_SPI1_SRRI)	//(EVT_SPI1_SRRI) //SRRI

#define IRQ_DMA_TC					  	  Int007_IRQn
#define INT_DMA_TC					  	  INT_DMA2_TC1

void WIFI_InitGPIO(void);
void WIFI_InitSPI(void);
void WIFI_InitDMA(void);

void WIFI_Init(void);
void SPI_ConnectWIFI();
void SPI_RestartWIFI();
void SPI_Receive_Send_DMA();

void WIFI_TX_Handler(int8_t control_code);
void DmaSPIIrqCallback(void);
void spi_idle();
void wifiResetEEPROM();
void WIFI_Upload_File();
#endif
