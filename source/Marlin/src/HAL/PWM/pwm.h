
#pragma once

#ifdef HWPWM

#define UN_BELT 3
#define UN_TLTS 5
#define UN_TLT  4
#define UN_F0   2
#define UN_F1   1
//for fan0
// TIMERA unit and clock definition 
#define TAUNIT_F0                    (M4_TMRA2)      //3
#define TAUNIT_F0_CLOCK              (PWC_FCG2_PERIPH_TIMA2) //3
#define TAUNIT_F0_OVERFLOW_INT       (INT_TMRA2_OVF)//3

// TIMERA channel Port/Pin definition
#define TAUNIT_F0_CH                (TimeraCh2)
#define TAUNIT_F0_CH_PORT           (PortA)     //PA1
#define TAUNIT_F0_CH_PIN            (Pin01)
#define TAUNIT_F0_CH_FUNC           (Func_Tima0) //0

#if ENABLED(TL_L)
#define TIMERA_COUNT_OVERFLOW_F0          (1250)  //5000 Hz
#else
#define TIMERA_COUNT_OVERFLOW_F0          (25000)  //500 Hz
#endif
#define IRQ_INDEX_INT_TIMA_CH_F0          (Int007_IRQn)

//for fan1
#define TAUNIT_F1                    (M4_TMRA1)      
#define TAUNIT_F1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TAUNIT_F1_OVERFLOW_INT       (INT_TMRA1_OVF)

#define TAUNIT_F1_CH                (TimeraCh5)
#define TAUNIT_F1_CH_PORT           (PortE)     //PE8
#define TAUNIT_F1_CH_PIN            (Pin08)
#define TAUNIT_F1_CH_FUNC           (Func_Tima0)

#define TIMERA_COUNT_OVERFLOW_F1            (25000)  //500 Hz
#define IRQ_INDEX_INT_TIMA_CH_F1            (Int008_IRQn)

//for tltouch
#ifdef TLTOUCH
#define TAUNIT_TLT                          (M4_TMRA4) 
#define TAUNIT_TLT_CLOCK                    (PWC_FCG2_PERIPH_TIMA4) 
#define TAUNIT_TLT_OVERFLOW_INT             (INT_TMRA4_OVF)

// TIMERA channel 1 Port/Pin definition
#define TAUNIT_TLT_CH                       (TimeraCh5)
#define TAUNIT_TLT_CH_PORT                  (PortC)     //PC14 TimerA4 PWM5
#define TAUNIT_TLT_CH_PIN                   (Pin14)
#define TAUNIT_TLT_CH_FUNC                  (Func_Tima0) //0

#define TIMERA_COUNT_OVERFLOW_TLT           (40000)  //500 Hz
#define IRQ_INDEX_INT_TIMA_CH_TLT           (Int006_IRQn)
#endif

//for tl-F
#ifdef TL_STEPTEST
#define TAUNIT_TLTS                    (M4_TMRA1) 
#define TAUNIT_TLTS_CLOCK              (PWC_FCG2_PERIPH_TIMA1) 
#define TAUNIT_TLTS_OVERFLOW_INT       (INT_TMRA1_OVF)

// TIMERA channel 1 Port/Pin definition
#define TAUNIT_TLTS_CH                (TimeraCh2)
#define TAUNIT_TLTS_CH_PORT           (PortA)     //PA9 TimerA1 PWM2
#define TAUNIT_TLTS_CH_PIN            (Pin09)
#define TAUNIT_TLTS_CH_FUNC           (Func_Tima0) //0

#define TIMERA_COUNT_OVERFLOW_TLTS          (5208)  ////freq: 1200Hz -> 200 000 000 /4/1200HZ/4/2 = 5208(period val)
#define IRQ_INDEX_INT_TIMA_CH_TLTS          (Int005_IRQn)
#define CAL_TLTS_OVERFLOW              200000000/4/4/STEPTEST_HZ
#endif

#ifdef CONVEYOR_BELT
    #define TAUNIT_BELT                    (M4_TMRA1) 
    #define TAUNIT_BELT_CLOCK              (PWC_FCG2_PERIPH_TIMA1) 
    #define TAUNIT_BELT_OVERFLOW_INT       (INT_TMRA1_OVF)

    // TIMERA channel 1 Port/Pin definition
    #define TAUNIT_BELT_CH                (TimeraCh6)
    #define TAUNIT_BELT_CH_PORT           (PortB)     //PB0 TimerA1 PWM6
    #define TAUNIT_BELT_CH_PIN            (Pin00)
    #define TAUNIT_BELT_CH_FUNC           (Func_Tima0) //0

    #define TIMERA_COUNT_OVERFLOW_BELT          (5208)  ////freq: 1200Hz -> 200 000 000 /4/1200HZ/4/2 = 5208(period val)
    #define IRQ_INDEX_INT_TIMA_CH_BELT          (Int004_IRQn)
    #define CAL_BELT_OVERFLOW              200000000/4/4/1200
#endif

void pwm_init();
void set_pwm_hw(uint16_t pwm_value, uint16_t max_value, uint8_t unitNo=UN_F0);
#endif
