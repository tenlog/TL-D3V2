#pragma once

#ifdef XEN_IIC
    #define DEVICE_ADDRESS              (0x41)
    #define EXI2C_TIMEOUT               (10000ul)
    
    extern bool XE_ENA[4];

    en_result_t XEI2C_Init(void);
    void XE_Enable();
 
#endif
