
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

#define TIMERA_COUNT_OVERFLOW_2          (1250)  //5000 Hz
#define IRQ_INDEX_INT_TIMA_CH_2          (Int007_IRQn)

#define TIMERA_UNIT1                    (M4_TMRA4)      //4
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA4) 
#define TIMERA_UNIT1_OVERFLOW_INT       (INT_TMRA4_OVF)

// TIMERA channel 1 Port/Pin definition
#define TIMERA_UNIT1_CH4                (TimeraCh5)
#define TIMERA_UNIT1_CH4_PORT           (PortC)     //PC14
#define TIMERA_UNIT1_CH4_PIN            (Pin14)
#define TIMERA_UNIT1_CH4_FUNC           (Func_Tima0) //0

#define TIMERA_COUNT_OVERFLOW_1          (30000)  //50 Hz
#define IRQ_INDEX_INT_TIMA_CH_1          (Int006_IRQn)

void pwm_init();
void set_pwm_hw(uint16_t pwm_value, uint16_t max_value, uint8_t unitNo=2);
#endif
