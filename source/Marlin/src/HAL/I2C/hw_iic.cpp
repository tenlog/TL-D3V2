#include "hc32_ddl.h"

#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef HW_IIC
#include "hw_iic.h"

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Master transmit data
 **
 ** \param  u16DevAddr            The slave address
 ** \param  pu8TxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - ErrorTimeout:       Time out
 ******************************************************************************/

en_result_t I2C_Transmit(uint16_t u16DevAddr, uint8_t *pu8TxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;
    I2C_Cmd(I2C_UNIT, Enable);

    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet){
        //TLDEBUG_PRINTLN("XE I2C Start OK!");
#ifdef I2C_10BITS_ADDRESS
        enRet = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2CDirTrans, u32TimeOut);
#else
        enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u16DevAddr, I2CDirTrans, u32TimeOut);
#endif

        if(Ok == enRet){
            //TLDEBUG_PRINTLN("XE I2C Address OK!");
            enRet = I2C_TransData(I2C_UNIT, pu8TxData, u32Size,u32TimeOut);
            if(Ok == enRet){
                //TLDEBUG_PRINTLN("XE I2C TransData OK!");
            }else{
                TLDEBUG_PRINTLN("HW I2C TransData Error!");
            }
        }else{
            TLDEBUG_PRINTLN("HW I2C Address Error!");
        }
    }else{
        TLDEBUG_PRINTLN("WH I2C Start Error!");
    }

    I2C_Stop(I2C_UNIT,u32TimeOut);
    I2C_Cmd(I2C_UNIT, Disable);

    return enRet;
}


/**
 ******************************************************************************
 ** \brief  Master receive data
 **
 ** \param  u16DevAddr            The slave address
 ** \param  pu8RxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
en_result_t I2C_Receive(uint16_t u16DevAddr, uint8_t *pu8RxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;

    I2C_Cmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet)
    {
        if(1ul == u32Size)
        {
            I2C_AckConfig(I2C_UNIT, I2c_NACK);
        }

#ifdef I2C_10BITS_ADDRESS
        enRet = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2CDirReceive, u32TimeOut);
#else
        enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u16DevAddr, I2CDirReceive, u32TimeOut);
#endif

        if(Ok == enRet)
        {

            enRet = I2C_MasterDataReceiveAndStop(I2C_UNIT, pu8RxData, u32Size, u32TimeOut);
        }

        I2C_AckConfig(I2C_UNIT, I2c_ACK);
    }

    if(Ok != enRet)
    {
        I2C_Stop(I2C_UNIT,u32TimeOut);
    }
    I2C_Cmd(I2C_UNIT, Disable);
    return enRet;
}



/**
 ******************************************************************************
 ** \brief   Initialize the I2C peripheral for master
 ** \param   None
 ** \retval en_result_t                Enumeration value:
 **          - Ok:                     Success
 **          - ErrorInvalidParameter:  Invalid parameter
 ******************************************************************************/
en_result_t HWI2C_Init(uint32_t Buadrate)
{
    /* Initialize I2C port*/
    PORT_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC, Disable);
    PORT_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(I2C_FCG_USE, Enable);
    
    en_result_t enRet;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    I2C_DeInit(I2C_UNIT);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV;
    stcI2cInit.u32Baudrate = Buadrate;
    stcI2cInit.u32SclTime = 0ul;
    enRet = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);
    if(Ok == enRet)
    {
        //TLDEBUG_PRINTLN("XE I2C Init OK!");
    }else{
        TLDEBUG_PRINTLN("XE I2C Init Error!");
    }

    I2C_BusWaitCmd(I2C_UNIT, Enable);
    
    return enRet;
}

#endif