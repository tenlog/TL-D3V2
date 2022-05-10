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

//By tenlo zyf

#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef ESP32_WIFI

char wifi_status[100] = "";
char wifi_ssid[20] = WIFI_DEFAULT_SSID;
char wifi_pswd[20] = WIFI_DEFAULT_PSWD;
uint8_t wifi_ena = WIFI_DEFAULT_ENA;
uint32_t http_port = WIFI_DEFAULT_PORT;


#include "esp32_wifi.h"
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

//【2】其次是GPIO口初始化（这边将SPI的CS脚当作GPIO进行初始化）：

/**************************************************************************
* 函数名称： WIFI_InitGPIO
* 功能描述： WIFI初始化GPIO引脚
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 初始化WIFI用到的GPIO口，包括RES、DC、BL以及SPI的NSS引脚
**************************************************************************/
void WIFI_InitGPIO(void)
{
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;

    /* SPI NSS */
    PORT_Init(SPI1_NSS_PORT, SPI1_NSS_PIN, &stcPortInit);
    SPI1_NSS_HIGH();
}

//        【3】SPI初始化：

/**************************************************************************
* 函数名称： WIFI_InitSPI1
* 功能描述： WIFI初始化SPI
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 初始化SPI用到的口，包括MOSI、MISO、SCK
**************************************************************************/
void WIFI_InitSPI1(void)
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
    stcSpiInit.enClkDiv                 = SpiClkDiv2;
    stcSpiInit.enFrameNumber            = SpiFrameNumber1;
    stcSpiInit.enDataLength             = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition       = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity            = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase               = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject       = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode               = SpiWorkMode3Line;
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
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime      = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime       = SpiSsIntervalSck6PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit              = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity             = SpiSsLowValid;
    
    SPI_Init(SPI1_UNIT, &stcSpiInit);
    SPI_Cmd(SPI1_UNIT, Enable);
}

//        【4】结合SPI的读写功能，封装接口供WIFI进行写命令（WIFI_WriteCMD）和写数据（WIFI_WriteDAT）：
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
    }
    SPI_SendData8(SPIx, data);
    while (Reset == SPI_GetFlag(SPIx, SpiFlagReceiveBufferFull))
    {
    }
    return SPI_ReceiveData8(SPIx);
}


//【5】封装WIFI的一些基本的接口，比如复位接口以及寄存器配置接口（其中寄存器的相关配置可以参考附录中提供的ILI9431数据手册）：

/**************************************************************************
* 函数名称： WIFI_HardwareReset
* 功能描述： WIFI复位
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 
**************************************************************************/
void Test_SPI(const char SString[])
{
    while(1){
        //TLDEBUG_LNPAIR("Send:", SString);
        char RString[32];
        NULLZERO(RString);
        SPI1_NSS_LOW();        
        delay(100);
        for(int i=0; i<32; i++){
            RString[i] = SPI_RW(SPI1_UNIT, SString[i]); 
        }        
        delay(100);
        SPI1_NSS_HIGH();        
        //TLDEBUG_LNPAIR("Received:",RString);
        //delay(500);
    }
}


/**************************************************************************
* 函数名称： WIFI_InitSPI
* 功能描述： WIFI初始化
* 输入参数： 
* 输出参数： 
* 返 回 值： 
* 其它说明： 
**************************************************************************/
void WIFI_InitSPI(void)
{
    WIFI_InitGPIO();    //初始化几个GPIO口，包括BL、DC、RES以及SPI的CS
    WIFI_InitSPI1();    //初始化SPI的几个口，包括SCK、MOSI以及MISO
    //WIFI_InitDMA();     //初始化SPI DMA

    Test_SPI("ABCDEFGHIJKLMNOPQRSTUVWXYZ012345");  //测试
}

///////////zyf
void wifiResetEEPROM(){
    wifi_ena = WIFI_DEFAULT_ENA;
    sprintf_P(wifi_ssid, "%s", WIFI_DEFAULT_SSID);
    sprintf_P(wifi_pswd, "%s", WIFI_DEFAULT_PSWD);
    http_port = WIFI_DEFAULT_PORT;
}

#endif
