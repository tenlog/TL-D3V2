#include "hc32_ddl.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef  XEN_IIC

#include "xen_iic.h"
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
static en_result_t I2C_Master_Transmit(uint16_t u16DevAddr, uint8_t *pu8TxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;
    I2C_Cmd(I2C_UNIT, Enable);

    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet)
    {

#ifdef I2C_10BITS_ADDRESS
        enRet = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2CDirTrans, u32TimeOut);
#else
        enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u16DevAddr, I2CDirTrans, u32TimeOut);
#endif

        if(Ok == enRet)
        {
            enRet = I2C_TransData(I2C_UNIT, pu8TxData, u32Size,u32TimeOut);
        }
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
static en_result_t I2C_Master_Receive(uint16_t u16DevAddr, uint8_t *pu8RxData, uint32_t u32Size, uint32_t u32TimeOut)
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
static en_result_t Master_Initialize(void)
{
    en_result_t enRet;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    I2C_DeInit(I2C_UNIT);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    stcI2cInit.u32SclTime = 0ul;
    enRet = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    I2C_BusWaitCmd(I2C_UNIT, Enable);

    return enRet;
}

/**
 *******************************************************************************
 ** \brief  Main function of template project
 ** \param  None
 ** \retval int32_t return value, if needed
 ******************************************************************************/

int32_t  _main_(void)
{
    uint8_t u8TxBuf[TEST_DATA_LEN];
    uint8_t u8RxBuf[TEST_DATA_LEN];
    uint32_t i;

    for(i=0ul; i<TEST_DATA_LEN; i++)
    {
        u8TxBuf[i] = (uint8_t)(i+1ul);
    }
    memset(u8RxBuf, 0x00, TEST_DATA_LEN);

    /* BSP initialization */
    //BSP_CLK_Init();
    //BSP_LED_Init();
    //BSP_KEY_Init();

    /* Initialize I2C port*/
    PORT_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC, Disable);
    PORT_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(I2C_FCG_USE, Enable);
    /* Initialize I2C peripheral and enable function*/
    Master_Initialize();

    /* Wait key */
    //while (Reset == BSP_KEY_GetStatus(BSP_KEY_1))
    //{
    //    ;
    //}

    Ddl_Delay1ms(5ul);

    I2C_Master_Transmit(DEVICE_ADDRESS, u8TxBuf, TEST_DATA_LEN, TIMEOUT);

    /* 50mS delay for device*/
    Ddl_Delay1ms(50ul);

    I2C_Master_Receive(DEVICE_ADDRESS, u8RxBuf, TEST_DATA_LEN, TIMEOUT);

    /* Compare the data */
    for(i=0ul; i<TEST_DATA_LEN; i++)
    {
        if(u8TxBuf[i] != u8RxBuf[i])
        {
            /* Data write error*/
            while(1)
            {
                //BSP_LED_Toggle(LED_RED);
                Ddl_Delay1ms(500ul);
            }
        }
    }

    /* I2C master polling comunication success */
    while(1)
    {
        //BSP_LED_Toggle(LED_GREEN);
        Ddl_Delay1ms(500ul);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

#endif
