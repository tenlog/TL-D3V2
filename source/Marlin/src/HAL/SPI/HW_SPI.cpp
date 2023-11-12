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

//By zyf

#include "watchdog.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"
#include "../../sd/cardreader.h"

#ifdef HW_SPI
#include "HW_SPI.h"

/*
void WIFI_InitDMA(void)
{
    stc_dma_config_t stcDmaCfg;
    stc_irq_regi_conf_t stcIrqRegiCfg;
       
    // configuration structure initialization 
    MEM_ZERO_STRUCT(stcDmaCfg);
    // Configuration peripheral clock 
    PWC_Fcg0PeriphClockCmd(SPI_DMA_CLOCK_UNIT, Enable);
    //PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
    // Configure TX DMA 
    stcDmaCfg.u16BlockSize = 1u;
    stcDmaCfg.u16TransferCnt = 128;
    stcDmaCfg.u32SrcAddr = 0;
    stcDmaCfg.u32DesAddr = (uint32_t)(&SPI1_UNIT->DR);
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;
    stcDmaCfg.stcDmaChCfg.enIntEn = Disable;
    DMA_InitChannel(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, &stcDmaCfg);
    DMA_SetTriggerSrc(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, SPI_DMA_TX_TRIG_SOURCE);
    // Enable DMA. 
    DMA_Cmd(SPI_DMA_UNIT, Enable);
}
*/

//其次是GPIO口初始化（这边将SPI的CS脚当作GPIO进行初始化）：

/**************************************************************************
* 函数名称： SPI_InitGPIO
* 功能描述： SPI初始化GPIO引脚
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 初始化WIFI用到的GPIO口，包括RES、DC、BL以及SPI的NSS引脚
**************************************************************************/
void SPI_InitGPIO(void)
{
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
	//stcPortInit.enPullUp = Enable;
	//stcPortInit.enPinDrv = Pin_Drv_H;		//high drive
    /* SPI NSS */
    PORT_Init(SPI1_WIFI_NSS_PORT, SPI1_WIFI_NSS_PIN, &stcPortInit);
    TERN_(TL_SPI_DRIVE, PORT_Init(SPI1_Y_NSS_PORT, SPI1_Y_NSS_PIN, &stcPortInit));
    
    PORT_Init(SPI1_MOSI_PORT, SPI1_MOSI_PIN, &stcPortInit);
    PORT_Init(SPI1_SCK_PORT, SPI1_SCK_PIN, &stcPortInit);
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_In;
    PORT_Init(SPI1_MISO_PORT, SPI1_MISO_PIN, &stcPortInit);
    SPI1_WIFI_NSS_HIGH();
    TERN_(TL_SPI_DRIVE, SPI1_Y_NSS_HIGH());
}

//SPI初始化：
/**************************************************************************
* 函数名称： SPI_InitSPI
* 功能描述： WIFI初始化SPI
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 初始化SPI用到的口，包括MOSI、MISO、SCK
**************************************************************************/
void SPI_InitSPI(void)
{
    stc_spi_init_t stcSpiInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI1_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI1_SCK_PORT, SPI1_SCK_PIN, SPI1_SCK_FUNC,  Disable);
    PORT_SetFunc(SPI1_MOSI_PORT, SPI1_MOSI_PIN, SPI1_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI1_MISO_PORT, SPI1_MISO_PIN, SPI1_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv                 = SPI_CLK_DIV;
    stcSpiInit.enFrameNumber            = SpiFrameNumber1;
    stcSpiInit.enDataLength             = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition       = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity            = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase               = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject       = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode               = SpiWorkMode4Line;
    stcSpiInit.enTransMode              = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn      = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn     = Disable;
    stcSpiInit.enParityEn               = Disable;
    stcSpiInit.enParity                 = SpiParityEven;

    /* SPI master mode */
    stcSpiInit.enMasterSlaveMode                     = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption   = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime     = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption    = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime      = SpiSsHoldDelaySck3;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime       = SpiSsIntervalSck1PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit              = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity             = SpiSsLowValid;

    SPI_Init(SPI1_UNIT, &stcSpiInit);
    SPI_Cmd(SPI1_UNIT, Enable);
}

//SPI的读写功能：
/**************************************************************************
* 函数名称： SPI_RW
* 功能描述： SPI读写功能
* 输入参数：
* 输出参数：
* 返 回 值：
* 其它说明：
**************************************************************************/
uint8_t SPI_RW(M4_SPI_TypeDef *SPIx, uint8_t data)
{
    while (Reset == SPI_GetFlag(SPIx, SpiFlagSendBufferEmpty))
    {
        NOOP;
    }
    SPI_SendData8(SPIx, data);
    while (Reset == SPI_GetFlag(SPIx, SpiFlagReceiveBufferFull))
    {
        NOOP;
    }
    return SPI_ReceiveData8(SPIx);
}


/**************************************************************************
* 函数名称： WIFI_InitSPI
* 功能描述： WIFI初始化
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 
**************************************************************************/
void TL_SPI_Init(void)
{
    SPI_InitGPIO();    //初始化几个GPIO口，
    SPI_InitSPI();    //初始化SPI的几个口，包括SCK、MOSI以及MISO
    //WIFI_InitDMA();     //初始化SPI DMA
}

#endif
