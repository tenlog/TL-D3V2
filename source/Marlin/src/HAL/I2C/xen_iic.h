
#pragma once

#ifdef XEN_IIC
    #define I2C_UNIT                        (M4_I2C3)
    /* Define slave device address for example */
    #define DEVICE_ADDRESS                  (0x41)
    //#define I2C_10BITS_ADDRESS              (1u)

    //SDA=PB3 SCL=PB4
    /* Define port and pin for SDA and SCL */
    #define I2C_SCL_PORT                    (PortB)
    #define I2C_SCL_PIN                     (Pin04)
    #define I2C_SDA_PORT                    (PortB)
    #define I2C_SDA_PIN                     (Pin03)
    #define I2C_GPIO_SCL_FUNC               (Func_I2c3_Scl)
    #define I2C_GPIO_SDA_FUNC               (Func_I2c3_Sda)

    #define I2C_FCG_USE                     (PWC_FCG1_PERIPH_I2C3)

    #define EXI2C_TIMEOUT                   (10000ul)

    #define I2C_BAUDRATE                    (400000ul)
    #define I2C_CLK_DIV                     I2C_CLK_DIV2

    en_result_t XEI2C_Init(void);
    void XEI2C_Write(uint8_t reg, uint8_t data);
    void XE_Enable();

    extern bool XE_ENA[4];
#endif
