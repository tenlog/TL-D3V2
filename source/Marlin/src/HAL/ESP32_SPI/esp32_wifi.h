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

//By tenlog zyf

/*
MOSI (tx) contorl code
from mcu to wifi:

control_code 0x01: wifi mode
control_code 0x02: http port
control_code 0x03: wifi ssid
control_code 0x04: wifi pswd
control_code 0x05: acce code
control_code 0x06: apply wifi

congrol_code 0x07: printer_status_0
congrol_code 0x08: printer_status_1
congrol_code 0x09: printer_status_2
congrol_code 0x0A: printer_status_3
congrol_code 0x0B: printer_status_4

MISO (rx) control code
from wifi to mcu
control_code 0x06: ip
control_code 0x01: request connect wifi

*/

#pragma once

#ifdef ESP32_WIFI

#define WIFI_MSG_LENGTH 27
#define BUFFER_SIZE 32

extern char wifi_ssid[WIFI_MSG_LENGTH];
extern char wifi_pswd[WIFI_MSG_LENGTH];
extern char wifi_acce_code[WIFI_MSG_LENGTH];
extern uint8_t wifi_mode;
extern uint16_t http_port;
extern bool wifi_connected;
extern int8_t wifiFirstSend;

extern char printer_status_0[WIFI_MSG_LENGTH];
extern char printer_status_1[WIFI_MSG_LENGTH];
extern char printer_status_2[WIFI_MSG_LENGTH];
extern char printer_status_3[WIFI_MSG_LENGTH];
extern char printer_status_4[WIFI_MSG_LENGTH];

#define HEAD_OK(a)	(a[0]==0xFF && a[1]==0xFF && a[2]==0xFF)

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

void WIFI_InitSPI(void);
void SPI_ConnectWIFI();

void WIFI_TX_Handler(int8_t control_code);

//uint8_t get_control_code();
//void get_data_code(uint8_t control_code);

//void WIFI_InitDMA(void);

void wifiResetEEPROM();
#endif
