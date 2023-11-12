#pragma once

#ifdef XEN_IIC
    #define DEVICE_ADDRESS              (0x41)
    #define EXI2C_TIMEOUT               (10000ul)

    en_result_t XEI2C_Init(void);
    void XEI2C_Write(uint8_t reg, uint8_t data);
    void XE_Enable();
 
    extern bool XE_ENA[4];
#endif
