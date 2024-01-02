
#include "startup.h"
#include "sdmmc_cmd.h"
#include "sd_card.h"
#include "sdio_hal.h"
#include "../../Marlin/src/lcd/tenlog/tenlog_touch_lcd.h"

static stc_sd_handle_t stcSdhandle;
#ifdef TL_HIGHSPEED
#define TLSDIOSPEEDMODE SdiocHighSpeedMode//SdiocHighSpeedMode
#define TLSDIOCLOCK	SdiocClk40M
#else
#define TLSDIOSPEEDMODE SdiocNormalSpeedMode
#define TLSDIOCLOCK	SdiocClk40M
#endif
stc_sdcard_init_t stcCardInitCfg = {SdiocBusWidth4Bit, TLSDIOCLOCK, TLSDIOSPEEDMODE, NULL,}; //SdiocHighSpeedMode SdiocNormalSpeedMode Normal speed will cause mainboard reboot when print speed > 200;
stc_sdcard_dma_init_t stcDmaInitCfg = {M4_DMA2, DmaCh0,};//M4_DMA2 DmaCh0 //by zyf dma2 ch0 is woriking...
Sdioc_Class::Sdioc_Class(M4_SDIOC_TypeDef *SDIOCx){
	MEM_ZERO_STRUCT(stcSdhandle);
	/* Initialize SDIOC pin */
	PORT_SetFuncMapp(BOARD_SDIO_D0, Disable);
	PORT_SetFuncMapp(BOARD_SDIO_D1, Disable);
	PORT_SetFuncMapp(BOARD_SDIO_D2, Disable);
	PORT_SetFuncMapp(BOARD_SDIO_D3, Disable);
	PORT_SetFuncMapp(BOARD_SDIO_CLK, Disable);
	PORT_SetFuncMapp(BOARD_SDIO_CMD, Disable);
	PORT_SetFuncMapp(BOARD_SDIO_DET, Disable);
	stcSdhandle.SDIOCx = SDIOCx;
	stcSdhandle.enDevMode = SdCardDmaMode;
	stcSdhandle.pstcDmaInitCfg = &stcDmaInitCfg;
}

Sdioc_Class::~Sdioc_Class(void){

}

en_result_t Sdioc_Class::sdio_port_init(uint8_t PinNum){
	en_result_t rt=Ok;
    stc_port_init_t stcPortInit;
    MEM_ZERO_STRUCT(stcPortInit);
	//stcPortInit.enPinMode = enMode;
    stcPortInit.enPullUp  = Disable;
	stcPortInit.enPinDrv = Pin_Drv_H; //by zyf
	rt = PORT_InitMapp(PinNum, &stcPortInit);

	if(rt!=Ok)
	{
		printf("sdio port init fail !,rt=%u\n",(uint8_t)rt);
	}
	return(rt);

}

en_result_t Sdioc_Class::init(void){
	en_result_t rt=Ok;
	sdio_port_init(BOARD_SDIO_D0);
	sdio_port_init(BOARD_SDIO_D1);
	sdio_port_init(BOARD_SDIO_D2);
	sdio_port_init(BOARD_SDIO_D3);
	sdio_port_init(BOARD_SDIO_CLK);
	sdio_port_init(BOARD_SDIO_CMD);
	sdio_port_init(BOARD_SDIO_DET);	
	/* Initialize SD card */
	rt = SDCARD_Init(&stcSdhandle, &stcCardInitCfg);
	if(rt!=Ok)
	{
		printf("SD init fail !,rt=%u\n",(uint8_t)rt);
	}
	return(rt);
}

void Sdioc_Class::end(M4_SDIOC_TypeDef *SDIOCx){
	
}

en_result_t Sdioc_Class::erase(uint32_t BlkStartAddr,uint32_t BlkEndAddr){
	return SDCARD_Erase(&stcSdhandle, BlkStartAddr, BlkEndAddr, dele_timeout);
}

/**attention: min dataunit: 4byte or 32bit **/
en_result_t Sdioc_Class::read(uint32_t BlockAddr,uint16_t BlockCnt,uint8_t *dest){
	return SDCARD_ReadBlocks(&stcSdhandle, BlockAddr, BlockCnt, (uint8_t *)dest, sd_read_timeout);
}

/**attention: min dataunit: 4byte or 32bit **/
en_result_t Sdioc_Class::write(uint32_t BlockAddr,uint16_t BlockCnt, const uint8_t *dest)
{
    return SDCARD_WriteBlocks(&stcSdhandle, BlockAddr, BlockCnt, (uint8_t *)dest, writ_timeout);
}

/*****************end of file*******************/
