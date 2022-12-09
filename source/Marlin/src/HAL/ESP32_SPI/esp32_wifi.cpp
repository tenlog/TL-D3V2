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

#include "watchdog.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"
#include "../../sd/cardreader.h"

#ifdef ESP32_WIFI
#include "esp32_wifi.h"

char wifi_ssid[20] = WIFI_DEFAULT_SSID;
char wifi_pswd[20] = WIFI_DEFAULT_PSWD;
char wifi_acce_code[20] = WIFI_DEFAULT_ACCE_CODE;
uint8_t wifi_ip_settings[20] = WIFI_DEFAULT_IP_SETTINGS;
uint8_t wifi_mode = WIFI_DEFAULT_MODE;
uint16_t http_port = WIFI_DEFAULT_PORT;
bool wifi_connected = false;
int8_t wifiFirstSend = 0;
bool wifi_resent = false;

char wifi_tjc_cmd[64]="";

uint8_t wifi_printer_status[WIFI_MSG_LENGTH]={0};
uint8_t wifi_printer_settings[WIFI_MSG_LENGTH]={0};
uint8_t wifi_file_name[WIFI_MSG_LENGTH]={0};

uint8_t wifi_version[3]={0};

uint8_t spi_tx[BUFFER_SIZE]="";
uint8_t spi_rx[BUFFER_SIZE]="";

bool file_uploading = false;
bool file_writing = false;

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
	//stcPortInit.enPullUp = Enable;
	//stcPortInit.enPinDrv = Pin_Drv_H;		//high drive
    /* SPI NSS */
    PORT_Init(SPI1_NSS_PORT, SPI1_NSS_PIN, &stcPortInit);
    PORT_Init(SPI1_MOSI_PORT, SPI1_MOSI_PIN, &stcPortInit);
    PORT_Init(SPI1_SCK_PORT, SPI1_SCK_PIN, &stcPortInit);
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_In;
    PORT_Init(SPI1_MISO_PORT, SPI1_MISO_PIN, &stcPortInit);
    SPI1_NSS_HIGH();
}

//SPI初始化：
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
    //PORT_SetFunc(SPI1_NSS_PORT, SPI1_NSS_PIN, SPI1_NSS_FUNC,  Disable);
    PORT_SetFunc(SPI1_SCK_PORT, SPI1_SCK_PIN, SPI1_SCK_FUNC,  Disable);
    PORT_SetFunc(SPI1_MOSI_PORT, SPI1_MOSI_PIN, SPI1_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI1_MISO_PORT, SPI1_MISO_PIN, SPI1_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv                 = SpiClkDiv8;
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

uint8_t get_control_code(){
	if(HEAD_OK(spi_rx)){
        uint8_t code = spi_rx[2];
		if(code == 0x0A) return code;
        uint8_t verify=0;
		for(uint16_t i=0; i<BUFFER_SIZE-1; i++){
			verify += spi_rx[i];
		}
		if(verify % 0x100 == spi_rx[BUFFER_SIZE-1]){
			return code;
		}
	}
	return 0;
}

void SPI_RX_Handler(){
    char cmd[64];
	HAL_watchdog_refresh();
    char ret[WIFI_MSG_LENGTH];
	NULLZERO(ret);
    uint8_t control_code = get_control_code();

    if(control_code > 0){
        for(uint16_t i=0; i<WIFI_MSG_LENGTH; i++){
            ret[i] = spi_rx[i+3];
        }
    }

    if(control_code== 0x06){
        if(ret[0] == '1' || ret[0] == '2' || ret[0] == '3' || ret[0] == '4' || ret[0] == '5' || ret[0] == '6' || ret[0] == '7' || ret[0] == '8' || ret[0] == '9' || ret[0] == '0') {
            wifi_connected = true;
            wifi_resent = false;
            tlInitSetting(false);
        }
        if(ret[0]){
            TJC_DELAY;
            TLSTJC_println("sleep=0");
            sprintf_P(wifi_tjc_cmd, PSTR("wifisetting.tIP.txt=\"%s\""), ret);
            TLSTJC_println(wifi_tjc_cmd);
            sprintf_P(wifi_tjc_cmd, PSTR("settings.tIP.txt=\"%s\""), ret);
            TLSTJC_println(wifi_tjc_cmd);
        }
        ZERO(spi_rx);
    }else if(control_code == 0x01){
        TLSTJC_println("sleep=0");        
        sprintf_P(wifi_tjc_cmd, PSTR("wifisetting.tIP.txt=\"Connecting...\""), ret);
        TLSTJC_println(wifi_tjc_cmd);
        sprintf_P(wifi_tjc_cmd, PSTR("settings.tIP.txt=\"Connecting...\""), ret);
        TLSTJC_println(wifi_tjc_cmd);
        SPI_ConnectWIFI();
    }else if(control_code == 0x07){     //GCode handler
        long gcode_wifi[WIFI_MSG_LENGTH];
    	for(uint16_t i=0; i<WIFI_MSG_LENGTH; i++){
            gcode_wifi[i] = ret[i];
        }
        process_command_gcode(gcode_wifi);
    }else if(control_code == 0x08){     //wifi version
    	for(uint8_t i=0; i<3; i++){
            wifi_version[i] = ret[i];
        }
        if(wifi_version[1] % 2 == 0)
            sprintf_P(wifi_tjc_cmd, PSTR("wifisetting.tVersion.txt=\"WIFI V%d.%d.%d\""), wifi_version[0],wifi_version[1],wifi_version[2]);
        else
            sprintf_P(wifi_tjc_cmd, PSTR("wifisetting.tVersion.txt=\"WIFICAM V%d.%d.%d\""), wifi_version[0],wifi_version[1],wifi_version[2]);
        TLSTJC_println(wifi_tjc_cmd);
    }else if(control_code == 0x09){
        file_uploading = true;
        if (!card.isMounted()) card.mount();
        char fname[27] = "";
        NULLZERO(fname);
        uint8_t name_length = ret[0];
        for(uint8_t i = 0; i<name_length; i++){
            fname[i] = ret[i+1];
        }
        card.closefile();
        card.removeFile(fname);

        card.openFileWrite(fname);
        if (!card.isFileOpen()) {
            file_writing = false;
            //sprintf_P(cmd, "Failed to open %s to write. length:%d", fname, name_length);
        }else{
            file_writing = true;
            //sprintf_P(cmd, "Seccess to open %s to write. length:%d", fname, name_length);
        }
        //TLDEBUG_PRINT(cmd);
    }else if(control_code == 0x0A){
        #define WIFI_FILE_DATA_LENGTH WIFI_MSG_LENGTH-2
        if(file_writing){
            uint8_t file_data[WIFI_FILE_DATA_LENGTH]={0};
            ZERO(file_data);
            uint16_t file_buffer_size = ret[0] * 0x100 + ret[1];
            for(uint16_t i=0; i<file_buffer_size; i++){
                file_data[i] = ret[i+2];
            }
            card.write(file_data, file_buffer_size);
            //sprintf_P(cmd, "Writing %d bytes.", file_buffer_size);
            //TLDEBUG_PRINT(cmd);
        }
    }else if(control_code == 0x0B){
        if(file_writing){
            card.closefile();
            sprintf_P(cmd, "Writing Done. %d", file_uploading);
            TLDEBUG_PRINT(cmd);
            card.closefile();
            card.tl_ls(true);
        }
        file_writing = false;
        //wifi_update_interval = 500;
        file_uploading = false;
    }
}

void SPI_RW_Message(){
    ZERO(spi_rx);
    SPI1_NSS_LOW();
    for(uint16_t i=0; i<BUFFER_SIZE; i++){
        spi_rx[i] = SPI_RW(SPI1_UNIT, spi_tx[i]); 
    }
    SPI1_NSS_HIGH();
    SPI_RX_Handler();
}

void WIFI_TX_Handler(int8_t control_code){
    HAL_watchdog_refresh();
    ZERO(spi_tx);
    uint8_t verify = 0;
    for(uint8_t i=0; i<2; i++){
        spi_tx[i]=0xFF;
    }
    uint8_t send[WIFI_MSG_LENGTH];
    ZERO(send);
    
    switch (control_code){
        case 0x01:
            memcpy_P(send, wifi_ip_settings, 20);
        break;
        case 0x03:
            memcpy_P(send, wifi_ssid, 20);
        break;
        case 0x04:
            memcpy_P(send, wifi_pswd, 20);
        break;
        case 0x05:
            memcpy_P(send, wifi_acce_code, 20);
        break;
        case 0x07:
            memcpy_P(send, wifi_printer_status, WIFI_MSG_LENGTH);
        break;        
        case 0x09:
            memcpy_P(send, wifi_printer_settings, WIFI_MSG_LENGTH);            
        break;        
        case 0x0A:
            memcpy_P(send, wifi_file_name, WIFI_MSG_LENGTH);
        break;
        
        case 0x08:      //SN no..
        {
            //0-25  HCSN
            for(uint8_t i=0; i<25; i++){
                send[i] = tl_hc_sn[i];
            }
            //25-42 TJCSN
            for(uint8_t i=0; i<17; i++){
                send[i+25]=tl_tjc_sn[i];
            }
            char str[20];
            sprintf_P(str, PSTR("%s V%s.%s"), TL_MODEL_STR, SHORT_BUILD_VERSION, TL_SUBVERSION);

            //42-59 VERSION
            for(uint8_t i=0; i<17; i++){
                send[i+42]=str[i];
            }
        }
        break;
    }
    spi_tx[2] = control_code;

    switch (control_code){
        case 0x01:
        {
            spi_tx[3] = wifi_mode;
            for(uint16_t i=0; i<WIFI_MSG_LENGTH; i++){
                spi_tx[i+4]=send[i];
            }
        }
        break;
        case 0x02:
        spi_tx[3] = http_port / 0x100;
        spi_tx[4] = http_port % 0x100;
        break;
        case 0x03:        
        case 0x04:        
        case 0x05:        
        case 0x07:  //status_0
        case 0x08:  //Serial Nos..
        case 0x09:  //Settings
        case 0x0A:  //file name
        {
            for(uint16_t i=0; i<WIFI_MSG_LENGTH; i++){
                spi_tx[i+3]=send[i];
            }
        }
        break;
    }

    for(uint16_t i=0; i<BUFFER_SIZE-1; i++){
        verify += spi_tx[i];
    }

    uint8_t verify8 = verify % 0x100;
    spi_tx[BUFFER_SIZE-1]=verify8;
    delay(3);//why need this?
    SPI_RW_Message();
}

/**************************************************************************/
void SPI_ConnectWIFI(){
    wifi_connected = false;
    wifiFirstSend = 0;
    for(uint8_t i=0; i<7; i++){
        delay(5);
        WIFI_TX_Handler(i);
    }
}

void SPI_RestartWIFI(){
    if(wifi_version[0]>=1 && wifi_version[1]>=2){
        TLECHO_PRINTLN("Restarting WIFI...");
        WIFI_TX_Handler(0x0B);
    }
    else{
        TLECHO_PRINTLN("Connecting WIFI...");
        SPI_ConnectWIFI();
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
void WIFI_Init(void)
{
    WIFI_InitGPIO();    //初始化几个GPIO口，
    WIFI_InitSPI1();    //初始化SPI的几个口，包括SCK、MOSI以及MISO
    //WIFI_InitDMA();     //初始化SPI DMA
}

///////////zyf
void wifiResetEEPROM(){
    wifi_mode = WIFI_DEFAULT_MODE;
    sprintf_P(wifi_ssid, "%s", WIFI_DEFAULT_SSID);
    sprintf_P(wifi_pswd, "%s", WIFI_DEFAULT_PSWD);
    sprintf_P(wifi_acce_code, "%s", WIFI_DEFAULT_ACCE_CODE);
    ZERO(wifi_ip_settings);
    uint8_t WIFI_IP[12] = WIFI_DEFAULT_IP_SETTINGS;
    memcpy_P(wifi_ip_settings, WIFI_IP, 12);
    http_port = WIFI_DEFAULT_PORT;
}

#endif