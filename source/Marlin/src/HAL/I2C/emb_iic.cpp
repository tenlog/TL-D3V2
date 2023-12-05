#include "hc32_ddl.h"

#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef  EMB_IIC

    #include "emb_iic.h"
    #include "hw_iic.h"

    //HC32Read iic example:
    //https://blog.csdn.net/ZhuoWuKeJi/article/details/129237473 
    /* Define slave device address for example */
    #define DEVICE_ADDRESS              0b0100000//    (0x40)
    #define EXI2C_TIMEOUT               (10000ul)

    #define  EMB_INPUT_PORT0_REG        0
    #define  EMB_INPUT_PORT1_REG        1
    #define  EMB_OUTPUT_PORT0_REG       2
    #define  EMB_OUTPUT_PORT1_REG       3
    #define  EMB_INVERSION_PORT0_REG    4
    #define  EMB_INVERSION_PORT1_REG    5
    #define  EMB_CONFIG_PORT0_REG       6
    #define  EMB_CONFIG_PORT1_REG       7
    #define  EMB_CONFIG_VAL0            0b00001100 //I07 FC1 FZ1 HE1 DJ1 DJ2 EN0 EN1
    #define  EMB_CONFIG_VAL1            0b00110001 //EN2 EN3 DJ3 DJ4 HE2 FC2 FZ2 I10
    #define  IIC_BUADRATE                (50000ul)

    bool XE_ENA[4] = {1,1,1,1};
    bool FAN_STA[4] = {1,0,1,0};    //FC1 FZ1 FC1 FZ2
    bool HEA_STA[2] = {0,1};

    en_result_t EMBI2C_Write(uint8_t witch, uint8_t data0, uint8_t data1)
    {
        uint8_t _wdata[3] = {0,0,0};
        if(witch == 0){
            _wdata[0] = EMB_OUTPUT_PORT0_REG;
        }else if(witch == 1){
            _wdata[0] = EMB_OUTPUT_PORT1_REG;
        }
        _wdata[1] = data0;
        _wdata[2] = data1;

        en_result_t enRet = I2C_Transmit(DEVICE_ADDRESS, _wdata, 3, EXI2C_TIMEOUT);
        if(Ok == enRet)
        {
            //TLDEBUG_PRINTLN("XE I2C Wrinte OK!");
        }else{
            TLDEBUG_PRINTLN("EMB I2C Write Error!");
        }
        return enRet;
    }

    void EMB_WRITE(){
        uint8_t _wdata[2] = {0,0};
        _wdata[0] =           0 * 0b10000000;
        _wdata[0] += FAN_STA[0] * 0b01000000;
        _wdata[0] += FAN_STA[1] * 0b00100000;
        _wdata[0] += HEA_STA[0] * 0b00010000;
        _wdata[0] +=          0 * 0b00001000;
        _wdata[0] +=          0 * 0b00000100;
        _wdata[0] +=  XE_ENA[0] * 0b00000010;
        _wdata[0] +=  XE_ENA[1] * 0b00000001;

        _wdata[1] =   XE_ENA[2] * 0b10000000;
        _wdata[1] +=  XE_ENA[3] * 0b01000000;
        _wdata[1] +=          0 * 0b00100000;
        _wdata[1] +=          0 * 0b00010000;
        _wdata[1] += HEA_STA[1] * 0b00001000;
        _wdata[1] += FAN_STA[2] * 0b00000100;
        _wdata[1] += FAN_STA[3] * 0b00000010;
        _wdata[1] +=          0 * 0b00000001;

        EMBI2C_Write(0, _wdata[0], _wdata[1]);
        EMBI2C_Write(1, _wdata[0], _wdata[1]);
    }

    en_result_t EMBI2C_Reg()
    {
        uint8_t w_data0[3] = {0};
        uint8_t w_data1[3] = {0};
        w_data0[0] = EMB_CONFIG_PORT0_REG;
        w_data0[1] = EMB_CONFIG_VAL0;
        w_data0[2] = EMB_CONFIG_VAL1;

        w_data1[0] = EMB_CONFIG_PORT1_REG;
        w_data1[1] = EMB_CONFIG_VAL0;
        w_data1[2] = EMB_CONFIG_VAL1;

        en_result_t enRet = I2C_Transmit(DEVICE_ADDRESS, w_data0, 3, EXI2C_TIMEOUT);
        if(Ok == enRet){
            enRet = I2C_Transmit(DEVICE_ADDRESS, w_data1, 3, EXI2C_TIMEOUT);
            if(Ok == enRet){
            }else{
                TLDEBUG_PRINTLN("EMB I2C Reg 1 Error!");
            }
        }else{
            TLDEBUG_PRINTLN("EMB I2C Reg 0 Error!");
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
        en_result_t enRet = I2C_Receive(DEVICE_ADDRESS, _data, 3, EXI2C_TIMEOUT);
        if(Ok == enRet)
        {        
            //TLDEBUG_PRINTLN("XE I2C Wrinte OK!");
        }else{
            TLDEBUG_PRINTLN("XE I2C Read Error!");
        }

        if(witch == 0){
            return _data[1];
        }else {
            return _data[2];
        }
        
    }

    /**
     ******************************************************************************
    ** \brief   Initialize the I2C peripheral for master
    ** \param   None
    ** \retval en_result_t                Enumeration value:
    **          - Ok:                     Success
    **          - ErrorInvalidParameter:  Invalid parameter
    ******************************************************************************/
    void EMBI2C_Init(void) {

        if(Ok == HWI2C_Init(IIC_BUADRATE)){
            Ddl_Delay1ms(200); 
            uint8_t data_r = EMBI2C_Read(0);
            EMBI2C_Reg();
        }
    }

#endif
