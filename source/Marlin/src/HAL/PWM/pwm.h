
#pragma once

#ifdef HWPWM

/* pwm Port/Pin definition */


// TIMERA unit and clock definition 
#define TIMERA_UNIT2                    (M4_TMRA2)      //3
#define TIMERA_UNIT2_CLOCK              (PWC_FCG2_PERIPH_TIMA2) //3
#define TIMERA_UNIT2_OVERFLOW_INT       (INT_TMRA2_OVF)//3

// TIMERA channel 1 Port/Pin definition
#define TIMERA_UNIT2_CH2                (TimeraCh2)
#define TIMERA_UNIT2_CH2_PORT           (PortA)     //PA1
#define TIMERA_UNIT2_CH2_PIN            (Pin01)
#define TIMERA_UNIT2_CH2_FUNC           (Func_Tima0) //0

/* TIMERA 4 channel 8 Port/Pin definition */
#define TIMERA_UNIT2_CH8                (TimeraCh8)
#define TIMERA_UNIT2_CH8_PORT           (PortC)     //PC3
#define TIMERA_UNIT2_CH8_PIN            (Pin03)
#define TIMERA_UNIT2_CH8_FUNC           (Func_Tima0)

//#define TIMERA_COUNT_OVERFLOW_F       (1250)  //5000 Hz
#define TIMERA_COUNT_OVERFLOW           (30000)  //50 Hz
#define IRQ_INDEX_INT_TIMA_CH1          (Int007_IRQn)

void pwm_init();
void set_pwm_hw(uint16_t pwm_value, uint16_t max_value, uint8_t CH=2);
#endif
