#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef  XEN_IIC

    #include "xen_iic.h"
    #include "hw_iic.h"

    bool XE_ENA[4] = {0};

    void XE_Enable(){
        uint8_t Ena = XE_ENA[0] * 0B0001 + XE_ENA[1] * 0B0010 + XE_ENA[2] * 0B0100 + XE_ENA[3] * 0B1000; 
        XEI2C_Write(0x03, 0x00); // 将所有引脚设置为输出
        XEI2C_Write(0x01, Ena); // 设置引脚输出状态
    }

    void XEI2C_Write(uint8_t reg, uint8_t data) {
        uint8_t buffer[2];
        buffer[0] = reg;
        buffer[1] = data;
        en_result_t enRet;
        enRet = I2C_Transmit(DEVICE_ADDRESS, buffer, 2, EXI2C_TIMEOUT);
        if(Ok == enRet)
        {
            //TLDEBUG_PRINTLN("XE I2C Wrinte OK!");
        }else{
            TLDEBUG_PRINTLN("XE I2C Write Error!");
        }
    }

    en_result_t XEI2C_Init(void){

        if(Ok == HWI2C_Init()){
            Ddl_Delay1ms(200); 
            XEI2C_Write(0x03, 0x00); // 将所有引脚设置为输出
            XEI2C_Write(0x01, 0x0F); // 设置引脚输出状态
        }
}

#endif
