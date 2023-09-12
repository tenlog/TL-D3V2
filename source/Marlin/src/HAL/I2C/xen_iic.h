
#pragma once

#ifdef XEN_IIC

/* Define I2C unit used for the example */
#define I2C_UNIT                        (M4_I2C1)
/* Define slave device address for example */
#define DEVICE_ADDRESS                  (0x06u)
//#define I2C_10BITS_ADDRESS              (1u)

/* Define port and pin for SDA and SCL */
#define I2C_SCL_PORT                    (PortC)
#define I2C_SCL_PIN                     (Pin04)
#define I2C_SDA_PORT                    (PortC)
#define I2C_SDA_PIN                     (Pin05)
#define I2C_GPIO_SCL_FUNC               (Func_I2c1_Scl)
#define I2C_GPIO_SDA_FUNC               (Func_I2c1_Sda)

#define I2C_FCG_USE                     (PWC_FCG1_PERIPH_I2C1)

#define TIMEOUT                         (0x10000ul)

/* Define Write and read data length for the example */
#define TEST_DATA_LEN                   (256u)
/* Define i2c baudrate */
#define I2C_BAUDRATE                    (400000ul)



#endif
