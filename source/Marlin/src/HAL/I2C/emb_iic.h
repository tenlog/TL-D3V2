#pragma once

#ifdef EMB_IIC
    /* Define slave device address for example */
    #define DEVICE_ADDRESS                  (0x40)
    #define EXI2C_TIMEOUT               (10000ul)

    #define  EMB_INPUT_PORT0_REG        0
    #define  EMB_INPUT_PORT1_REG        1
    #define  EMB_OUTPUT_PORT0_REG       2
    #define  EMB_OUTPUT_PORT1_REG       3
    #define  EMB_INVERSION_PORT0_REG    4
    #define  EMB_INVERSION_PORT1_REG    5
    #define  EMB_CONFIG_PORT0_REG       6
    #define  EMB_CONFIG_PORT1_REG       7
    #define  EMB_CONFIG_VAL0            0b00001100
    #define  EMB_CONFIG_VAL1            0b00110001

    en_result_t EMBI2C_Init(void);
    //void EMBI2C_Write(uint8_t *pu8RxData, uint32_t u32Size);
    //void EMBI2C_Read(uint8_t reg, uint8_t data);
    void EMB_WriteStatus();
    void EMB_ReadStatus();

#endif
