 
#pragma once

#ifdef HW_IIC
    #define I2C_UNIT                        (M4_I2C3)
    
    #define I2C_SCL_PORT                    (PortB)
    #define I2C_SCL_PIN                     (Pin04)
    #define I2C_SDA_PORT                    (PortB)
    #define I2C_SDA_PIN                     (Pin03)
    #define I2C_GPIO_SCL_FUNC               (Func_I2c3_Scl)
    #define I2C_GPIO_SDA_FUNC               (Func_I2c3_Sda)

    #define I2C_FCG_USE                     (PWC_FCG1_PERIPH_I2C3)
    
    #define I2C_BAUDRATE                    (400000ul)
    #define I2C_CLK_DIV                     I2C_CLK_DIV2

    en_result_t HWI2C_Init(uint32_t Buadrate);
    en_result_t I2C_Transmit(uint16_t u16DevAddr, uint8_t *pu8TxData, uint32_t u32Size, uint32_t u32TimeOut);
    en_result_t I2C_Receive(uint16_t u16DevAddr, uint8_t *pu8RxData, uint32_t u32Size, uint32_t u32TimeOut);

#endif
