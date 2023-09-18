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

#ifdef ESP32_WIFI
#include "esp32_wifi.h"


char wifi_ssid[20] = WIFI_DEFAULT_SSID;
char wifi_pswd[20] = WIFI_DEFAULT_PSWD;
char wifi_acce_code[20] = WIFI_DEFAULT_ACCE_CODE;
uint8_t wifi_ip_settings[20] = WIFI_DEFAULT_IP_SETTINGS;
uint8_t wifi_mode = WIFI_DEFAULT_MODE;
uint16_t http_port = WIFI_DEFAULT_PORT;
bool wifi_connected = false;
uint8_t wifiFirstSend = 0;
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

uint8_t upload_file_data1[WIFI_FILE_DATA_LENGTH]={0};
//uint8_t upload_file_data2[WIFI_FILE_DATA_LENGTH]={0};
//uint8_t upload_switch_flag = 0;
uint32_t received_file_block_id = 0;
uint32_t resend_file_block_id = 0;

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
    stcSpiInit.enClkDiv                 = ESP_SPI_CLK_DIV;
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
    /*
    char cmd[64];
    for(int i=0; i<4; i++){
        sprintf(cmd, " 0x%2X", spi_rx[i]);
        TLDEBUG_PRINT(cmd);
    }
    sprintf(cmd, " 0x%2X", spi_rx[4]);
    TLDEBUG_PRINTLN(cmd);
    TLDEBUG_PRINTLN("WIFI Control code failed.");
    */
	return 0;
}

/*
void test_write_file(){
    card.openFileWrite("test.gco");
    if (!card.isFileOpen()) {
        TLDEBUG_PRINTLN("Failed to open test.gco to write.");
        return;
    }
    __attribute__((aligned(sizeof(size_t)))) uint8_t buf[512];

    uint16_t c;
    for (c = 0; c < COUNT(buf); c++)
        buf[c] = 'A' + (c % ('Z' - 'A'));

    c = 1024 * 4;
    while (c--) {
        TERN_(USE_WATCHDOG, watchdog_refresh());
        card.write(buf, COUNT(buf));
    }
    TLDEBUG_PRINTLN(" done");
    card.closefile();
}
*/

uint32_t blockCount = 0;
uint32_t lostCount = 0;
//uint32_t CRCerrorCount = 0;
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

    /*
    if(file_writing){
        upload_switch_flag++;
        if(upload_switch_flag > 2) upload_switch_flag -= 2;
    }
    */

    if(control_code== 0x06){
        if(ret[0] == '1' || ret[0] == '2' || ret[0] == '3' || ret[0] == '4' || ret[0] == '5' || ret[0] == '6' || ret[0] == '7' || ret[0] == '8' || ret[0] == '9' || ret[0] == '0') {
            wifi_connected = true;
            wifi_resent = false;
            tlSendSettings(false);
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
    	for(uint8_t i=0; i<4; i++){
            wifi_version[i] = ret[i];
        }
        if(wifi_version[3] % 2 == 0)
            sprintf_P(wifi_tjc_cmd, PSTR("wifisetting.tVersion.txt=\"WIFI V%d.%d.%d\""), wifi_version[0],wifi_version[1],wifi_version[2]);
        else
            sprintf_P(wifi_tjc_cmd, PSTR("wifisetting.tVersion.txt=\"WIFICAM V%d.%d.%d\""), wifi_version[0],wifi_version[1],wifi_version[2]);
        TLSTJC_println(wifi_tjc_cmd);
    }else if(control_code == 0x09){
        if (!card.isMounted()) card.mount();
        file_uploading = true;
        received_file_block_id = 0;
        resend_file_block_id = 0;

        char fname[27] = "";
        NULLZERO(fname);
        uint8_t name_length = ret[0];
        for(uint8_t i = 0; i<name_length; i++){
            fname[i] = ret[i+1];
        }
        
        card.removeFile(fname); //if exist delete it 
        
        card.openFileWrite(fname);
        if (!card.isFileOpen()) {
            file_writing = false;
            sprintf_P(cmd, "Failed to open %s to write. ", fname);
        }else{
            file_writing = true;
            sprintf_P(cmd, "Seccess to open %s to write. ", fname);
            blockCount = 0;
            lostCount = 0;
            //CRCerrorCount = 0;
            #if ENABLED(TL_L)
            ZERO(pre_print_file_name);
            sprintf(pre_print_file_name, "%s", fname);
            #endif
        }
        TLDEBUG_PRINTLN(cmd);
        //upload_switch_flag = 0;
        ZERO(upload_file_data1);
        //ZERO(upload_file_data2);
    }else if(control_code == 0x0A){
        if(file_writing){
            blockCount++;
            static uint32_t old_block_id;
            received_file_block_id = ret[0] * 0x10000 + ret[1] * 0x100 + ret[2];

            //Check lost data..
            if(received_file_block_id > 3 && received_file_block_id - old_block_id != 1 && resend_file_block_id == 0){
                //lostCount++;
                resend_file_block_id = old_block_id + 1; //requir resend.
                upload_file_data1[0] = 0x00;
                #ifdef TL_DEBUG
                sprintf_P(cmd, "0 Received ID:%d, Lost ID:%d, old ID:%d, count:%d", received_file_block_id, resend_file_block_id, old_block_id, lostCount);
                TLECHO_PRINTLN(cmd);
                #endif
            }else if(received_file_block_id == resend_file_block_id && resend_file_block_id > 0){
                lostCount++;
                #ifdef TL_DEBUG
                sprintf_P(cmd, "1 Received ID:%d, Lost ID:%d, old ID:%d, count:%d", received_file_block_id, resend_file_block_id, old_block_id, lostCount);
                TLECHO_PRINTLN(cmd);
                #endif
                resend_file_block_id = 0;
            }else if(resend_file_block_id ==0){
                //sprintf_P(cmd, "2 Received ID:%d, Lost ID:%d, old ID:%d, count:%d", received_file_block_id, resend_file_block_id, old_block_id, lostCount);
                //TLECHO_PRINTLN(cmd);
                resend_file_block_id = 0;
            }
    
            if(resend_file_block_id == 0){
                uint8_t received_block[WIFI_FILE_DATA_LENGTH];
                for(uint16_t i=0; i<WIFI_FILE_DATA_LENGTH; i++){
                    received_block[i] = ret[i+4];
                }
                 memcpy(upload_file_data1, received_block, WIFI_FILE_DATA_LENGTH);
                //if(upload_switch_flag == 1) memcpy(upload_file_data1, received_block, WIFI_FILE_DATA_LENGTH);
                //else if(upload_switch_flag == 2) memcpy(upload_file_data2, received_block, WIFI_FILE_DATA_LENGTH);
                old_block_id = received_file_block_id;
            }
        }
    }else if(control_code == 0x0B){
        if(file_writing){
            #ifdef TL_DEBUG
            sprintf_P(cmd, "Blocks Received: %d, Lost:%d", blockCount, lostCount);
            TLECHO_PRINTLN(cmd);
            #endif
            uint32_t delay_time = blockCount / 10;
            if(delay_time < 50) delay_time = 50;
            if(delay_time > 1000) delay_time = 1000;
            uint32_t wait_start = millis();
            
            while (millis()-wait_start < delay_time)
            {
                watchdog_refresh();
            }
            
            card.closefile();
            delay(50);
            card.release();
            delay(50);
            card.tl_ls(true);
            delay(50);
            received_file_block_id = 0;
            resend_file_block_id = 0;
            lostCount = 0;
            #if ENABLED(TL_BEEPER)
            start_beeper(16, 1);
            #endif
            //CRCerrorCount = 0;
        }
        file_writing = false;
        //wifi_update_interval = 400;
        file_uploading = false;
    }
}

uint16_t check_upload_block_size(){
    uint16_t blockSize = WIFI_FILE_DATA_LENGTH;
    //if(switch_flag == 1){
        for(uint16_t i=WIFI_FILE_DATA_LENGTH-1; i>=0; i--){
            if(upload_file_data1[i] != 0x00){
                blockSize = i + 1;
                return blockSize;
            }
        }
    /*    
    }else if(switch_flag == 2){
        for(uint16_t i=WIFI_FILE_DATA_LENGTH-1; i>=0; i--){
            if(upload_file_data2[i] != 0x00){
                blockSize = i + 1;
                return blockSize;
            }
        }
    }else{
        return WIFI_FILE_DATA_LENGTH;
    }
    */
    return WIFI_FILE_DATA_LENGTH;
}

void wifi_upload_write_data(){    
    if(upload_file_data1[0] != 0x00){
        uint16_t blockSize = check_upload_block_size();
        //if(upload_switch_flag == 1 && upload_file_data1[0] != 0x00){
        card.write(upload_file_data1, blockSize);
        ZERO(upload_file_data1);
        //}else if(upload_switch_flag == 2  && upload_file_data2[0] != 0x00){
        //    card.write(upload_file_data2, blockSize);
        //    ZERO(upload_file_data2);
        //}
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
            sprintf_P(str, PSTR("%s V%s.%s"), TL_MODEL_STR, SHORT_BUILD_VERSION, TL_SUBVERSION); //HANDS2 V2.0.8.038

            //42-59 VERSION
            for(uint8_t i=0; i<17; i++){
                send[i+42]=str[i];
            }

            uint8_t singleHead=0;
            #ifdef SINGLE_HEAD
                singleHead = 1;
            #elif defined(TL_X)
                singleHead = 4;
            #endif
            uint8_t laser_only=0;
            #ifdef TL_L
                laser_only=1;
            #endif
            uint8_t mixing_extruder=0;
            #ifdef MIXING_EXTRUDER
            mixing_extruder=1;
            #endif
            send[59]=singleHead;
            send[60]=laser_only;
            send[61]=mixing_extruder;
            
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
        case 0x0C:  //feed back upload file 3,4,5
        {
            spi_tx[3] = resend_file_block_id / 0x10000;
            spi_tx[4] = resend_file_block_id / 0x100;
            spi_tx[5] = resend_file_block_id % 0x100;
            //resend_file_block_id = 0;
        }
        break;        
        //0x0B //=reboot
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

void SPI_resent_wifi_info(){
    wifiFirstSend = 0;
    for(uint8_t i=0; i<6; i++){
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
