#pragma once

#ifdef EMB_IIC
    //void EMB_WriteStatus();
    //void EMB_ReadStatus();
    #define XE_Enable() EMB_WRITE();

    extern bool XE_ENA[4];
    extern bool FAN_STA[4];
    extern bool HEA_STA[2];

    void XEI2C_Init(void);
    void EMB_WRITE();

#endif
