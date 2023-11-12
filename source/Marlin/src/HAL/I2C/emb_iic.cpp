#include "hc32_ddl.h"

#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef  EMB_IIC

#include "emb_iic.h"

en_result_t EMBI2C_Write(uint8_t *pData, uint16_t size)
{
    en_result_t enRet = I2C_Transmit(DEVICE_ADDRESS, pData, size, EXI2C_TIMEOUT);
    if(Ok == enRet)
    {
        //TLDEBUG_PRINTLN("XE I2C Wrinte OK!");
    }else{
        TLDEBUG_PRINTLN("XE I2C Reg Error!");
    }
	return enRet;
}

en_result_t EMBI2C_Reg()
{
	uint8_t w_data[] = {0x00, 0x00, 0x00};
	// 配置PCA9535 端口，即写配置寄存器，数据格式为：地址 + CMD + 8 input + 8 output
	w_data[0] = EMB_CONFIG_PORT0_REG;   //config_port0 寄存器
	w_data[1] = EMB_CONFIG_VAL0;
	w_data[2] = EMB_CONFIG_VAL1;
    en_result_t enRet = I2C_Transmit(DEVICE_ADDRESS, w_data, 3, EXI2C_TIMEOUT);
    if(Ok == enRet){
        //TLDEBUG_PRINTLN("XE I2C Wrinte OK!");
    }else{
        TLDEBUG_PRINTLN("XE I2C Write Error!");
    }
	return enRet;
}

uint8_t EMBI2C_Read(uint8_t witch)
{
    uint8_t _data[] = {0x00, 0x00, 0x00};
	memset(_data, 0, sizeof(_data));
    if(witch == 0){
        _data[0] = EMB_INPUT_PORT0_REG;
    }else if(witch == 1){
        _data[0] = EMB_INPUT_PORT1_REG;
    }
    en_result_t nRet = I2C_Receive(DEVICE_ADDRESS, _data, 3, EXI2C_TIMEOUT);
    if(Ok == enRet)
    {        
        //TLDEBUG_PRINTLN("XE I2C Wrinte OK!");
    }else{
        TLDEBUG_PRINTLN("XE I2C Read Error!");
    }
    if(witch == 0)
        return _
	
}

/**
 ******************************************************************************
 ** \brief   Initialize the I2C peripheral for master
 ** \param   None
 ** \retval en_result_t                Enumeration value:
 **          - Ok:                     Success
 **          - ErrorInvalidParameter:  Invalid parameter
 ******************************************************************************/
en_result_t EMBI2C_Init(void)
{
    
	Ddl_Delay1ms(200); 

	uint8_t r_data[] = {0x00, 0x00};

	// 上电先读取一次清除中断标志
	memset(w_data, 0, sizeof(w_data));
	
	EMBI2C_Read(EMB_INPUT_PORT0_REG, r_data, 2);

	EMBI2C_Reg();
	if (HAL_OK != snbiic_write(w_data, sizeof(w_data)))
	{
		pca9535_write(w_data, sizeof(w_data));
		pr_debug(0x03,"pca9535_write error\n");
	}
    return enRet;
}

#endif
