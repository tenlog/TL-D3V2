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

#ifdef HW_SPI

#define SPI_CLK_DIV SpiClkDiv16

/* SPI_SCK Port/Pin definition */
#define SPI1_SCK_PORT                    (PortA)
#define SPI1_SCK_PIN                     (Pin06)
#define SPI1_SCK_FUNC                    (Func_Spi1_Sck)

/* SPI_NSS Port/Pin definition */
#define SPI1_WIFI_NSS_PORT               (PortB)
#define SPI1_WIFI_NSS_PIN                (Pin01)

#ifdef TL_SPI_DRIVE
#define SPI1_Y_NSS_PORT                  (PortB)
#define SPI1_Y_NSS_PIN                   (Pin15)
#define SPI1_Y_NSS_HIGH()                (PORT_SetBits(SPI1_Y_NSS_PORT, SPI1_Y_NSS_PIN))
#define SPI1_Y_NSS_LOW()                 (PORT_ResetBits(SPI1_Y_NSS_PORT, SPI1_Y_NSS_PIN))
#endif

#define SPI1_WIFI_NSS_HIGH()             (PORT_SetBits(SPI1_WIFI_NSS_PORT, SPI1_WIFI_NSS_PIN))
#define SPI1_WIFI_NSS_LOW()              (PORT_ResetBits(SPI1_WIFI_NSS_PORT, SPI1_WIFI_NSS_PIN))

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

void SPI_InitGPIO(void);
void SPI_InitSPI1(void);
uint8_t SPI_RW(M4_SPI_TypeDef *SPIx, uint8_t data);
void TL_SPI_Init(void);
#endif
