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
#include "esp32_wifi.h"

char wifi_ssid[WIFI_MSG_LENGTH] = WIFI_DEFAULT_SSID;
char wifi_pswd[WIFI_MSG_LENGTH] = WIFI_DEFAULT_PSWD;
char wifi_acce_code[WIFI_MSG_LENGTH] = WIFI_DEFAULT_ACCE_CODE;
uint8_t wifi_mode = WIFI_DEFAULT_MODE;
uint16_t http_port = WIFI_DEFAULT_PORT;

char tjc_cmd[64]="";

char printer_status_0[WIFI_MSG_LENGTH]="";
char printer_status_1[WIFI_MSG_LENGTH]="";
char printer_status_2[WIFI_MSG_LENGTH]="";
char printer_status_3[WIFI_MSG_LENGTH]="";

uint8_t spi_tx[BUFFER_SIZE]="";
uint8_t spi_rx[BUFFER_SIZE]="";

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

//【4】结合SPI的读写功能，封装接口供WIFI进行写命令（WIFI_WriteCMD）和写数据（WIFI_WriteDAT）：
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

uint8_t get_control_code(){
	if(HEAD_OK(spi_rx)){
		int16_t verify=0;
		for(int i=0; i<BUFFER_SIZE-1; i++){
			verify += spi_rx[i];
		}
		if(verify % 0xFF == spi_rx[BUFFER_SIZE-1]){
			return spi_rx[3];
		}
	}
	return 0;	
}

void SPI_RX_Handler(){

    char ret[WIFI_MSG_LENGTH];
	NULLZERO(ret);
    uint8_t control_code = get_control_code();

    if(control_code > 0){
        for(int i=0; i<WIFI_MSG_LENGTH; i++){
            ret[i] = spi_rx[i+4];
            if(spi_rx[i+4] == 10 || spi_rx[i+4] == '\0' || spi_rx[i+4] == 0){
                ret[i] = '\0';
                break;
            }
        }
    }
    if(control_code== 0x06){
        sprintf_P(tjc_cmd, PSTR("wifisetting.tIP.txt=\"%s\""), ret);
        TLSTJC_println(tjc_cmd);
        delay(10);
        sprintf_P(tjc_cmd, PSTR("setting.tIP.txt=\"%s\""), ret);
        TLSTJC_println(tjc_cmd);
        ZERO(spi_rx);
    }else{
        char cmd[BUFFER_SIZE];
        for(int i=0; i<BUFFER_SIZE; i++){
            sprintf_P(cmd, "%X", spi_rx[i]);
            TLDEBUG_LNPGM(cmd);
        }
        TLDEBUG_LNPGM(" ");
    }

}

void SPI_RW_Message(){
    ZERO(spi_rx);
    SPI1_NSS_LOW();        
    delay(3);
    for(int i=0; i<BUFFER_SIZE; i++){
        spi_rx[i] = SPI_RW(SPI1_UNIT, spi_tx[i]); 
    }
    SPI1_NSS_HIGH();
    SPI_RX_Handler();
}

void WIFI_TX_Handler(int8_t control_code){
    ZERO(spi_tx);
    int16_t verify = 0;
    for(int8_t i=0; i<3; i++){
        spi_tx[i]=0xFF;
    }
    uint8_t send[WIFI_MSG_LENGTH];
    ZERO(send);
    
    switch (control_code){
        case 0x03:
        memcpy_P(send, wifi_ssid, WIFI_MSG_LENGTH);
        break;
        case 0x04:
        memcpy_P(send, wifi_pswd, WIFI_MSG_LENGTH);
        break;
        case 0x05:
        memcpy_P(send, wifi_acce_code, WIFI_MSG_LENGTH);
        break;
        case 0x07:
        memcpy_P(send, printer_status_0, WIFI_MSG_LENGTH);
        break;
        case 0x08:
        memcpy_P(send, printer_status_1, WIFI_MSG_LENGTH);
        break;
        case 0x09:
        memcpy_P(send, printer_status_2, WIFI_MSG_LENGTH);
        break;
        case 0x0A:
        memcpy_P(send, printer_status_3, WIFI_MSG_LENGTH);
        break;
    }
    spi_tx[3] = control_code;

    switch (control_code){
        case 0x01:
        spi_tx[4] = wifi_mode;
        break;
        case 0x02:
        spi_tx[4] = http_port / 0xFF;
        spi_tx[5] = http_port % 0xFF;
        break;
        case 0x03:        
        case 0x04:        
        case 0x05:        
        case 0x07:        
        case 0x08:
        case 0x09:
        case 0x0A:
        for(int8_t i=0; i<WIFI_MSG_LENGTH; i++){
            if(send[i]==10 || send[i]=='\0' || send[i]==0){
                spi_tx[i+4]='\0';
                break;
            }else{
                spi_tx[i+4]=send[i];
            }
        }
        break;
    }

    for(uint8_t i=0; i<BUFFER_SIZE-1; i++){
        verify += spi_tx[i];
    }

    uint8_t verify8 = verify % 0xFF;
    spi_tx[BUFFER_SIZE-1]=verify8;

    SPI_RW_Message();

}

/**************************************************************************/
void SPI_ConeectWIFI()
{
    for(int8_t i=1; i<7; i++){
        WIFI_TX_Handler(i);
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
    WIFI_InitGPIO();    //初始化几个GPIO口，
    WIFI_InitSPI1();    //初始化SPI的几个口，包括SCK、MOSI以及MISO
    SPI_ConeectWIFI();
    //WIFI_InitDMA();     //初始化SPI DMA
    //Test_SPI("ABCDEFGHIJKLMNOPQRSTUVWXYZ012345");  //测试
}

///////////zyf
void wifiResetEEPROM(){
    wifi_mode = WIFI_DEFAULT_MODE;
    sprintf_P(wifi_ssid, "%s", WIFI_DEFAULT_SSID);
    sprintf_P(wifi_pswd, "%s", WIFI_DEFAULT_PSWD);
    sprintf_P(wifi_acce_code, "%s", WIFI_DEFAULT_ACCE_CODE);
    http_port = WIFI_DEFAULT_PORT;
}

#endif