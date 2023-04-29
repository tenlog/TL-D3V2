#include "hc32_ddl.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"



#ifdef HWPWM
#include "pwm.h"

//#define ENA_CH2

#ifdef TLTOUCH
//#define ENA_CH8
#endif

static void TimeraUnit2_IrqCallback(void)
{
    TIMERA_ClearFlag(TIMERA_UNIT2, TimeraFlagOverflow);
}

static void Tim_Config(void)
{
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    MEM_ZERO_STRUCT(stcPortInit);

    //configuration structure initialization 
    stcPortInit.enPinMode = Pin_Mode_Out;
		stcPortInit.enPullUp = Disable;
		stcPortInit.enExInt = Enable;
	
    // Initialize pwm pin 
    #ifdef ENA_CH2
		PORT_Init(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN, &stcPortInit);
		PORT_ResetBits(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN);
    #endif
    #ifdef ENA_CH8
		PORT_Init(TIMERA_UNIT2_CH8_PORT, TIMERA_UNIT2_CH8_PIN, &stcPortInit);
		PORT_ResetBits(TIMERA_UNIT2_CH8_PORT, TIMERA_UNIT2_CH8_PIN);
    #endif

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT2_CLOCK, Enable);

    /* Configuration TIMERA compare pin */
    #ifdef ENA_CH2
    PORT_SetFunc(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN, TIMERA_UNIT2_CH2_FUNC, Disable);
    #endif
    #ifdef ENA_CH8
    PORT_SetFunc(TIMERA_UNIT2_CH8_PORT, TIMERA_UNIT2_CH8_PIN, TIMERA_UNIT2_CH8_FUNC, Disable);
    #endif

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv32;  //100 000 000 /4 //by zyf temp
    stcTimeraInit.enCntMode = TimeraCountModeTriangularWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
	  stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW; // //freq: 5000Hz -> 100 000 000 /4/5000HZ/4 = 1250(period val)
    TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
		
    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = stcTimeraInit.u16PeriodVal/2;   // 50%的占空比
	  stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
	  stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputHigh;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputHigh; //TimeraCompareMatchOutputReverse
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Disable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 2 */
    #ifdef ENA_CH2
    TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH2, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH2, Enable);
    #endif    
    #ifdef ENA_CH8
    /* Configure channel 8 */
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputHigh;
    TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH8, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH8, Enable);
    #endif

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH1;
    stcIrqRegiConf.pfnCallback = &TimeraUnit2_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
		
    TIMERA_Cmd(TIMERA_UNIT2,Enable);	
    #ifdef ENA_CH2
    TIMERA_SetCompareValue( TIMERA_UNIT2,TIMERA_UNIT2_CH2, TIMERA_COUNT_OVERFLOW*0.1);
    TIMERA_SpecifyOutputSta(TIMERA_UNIT2,TIMERA_UNIT2_CH2,TimeraSpecifyOutputInvalid);	
    #endif
    #ifdef ENA_CH8
    TIMERA_SetCompareValue( TIMERA_UNIT2,TIMERA_UNIT2_CH8, TIMERA_COUNT_OVERFLOW*0.1);
    TIMERA_SpecifyOutputSta(TIMERA_UNIT2,TIMERA_UNIT2_CH8,TimeraSpecifyOutputInvalid);	
    #endif
}

void set_duty_cycle(uint16_t duty, uint8_t CH)
{
		if(duty == 0)
		{
        if(CH == 2){
          #ifdef ENA_CH2
          TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH2, TimeraSpecifyOutputLow);
          #endif
        }else if(CH == 8){
          #ifdef ENA_CH8
          TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH8, TimeraSpecifyOutputLow);
          #endif
        }
			  TIMERA_Cmd(TIMERA_UNIT2,Enable);
		}
    else if(duty >= TIMERA_COUNT_OVERFLOW)
		{
        if(CH == 2){
          #ifdef ENA_CH2
          TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH2, TimeraSpecifyOutputHigh);
          #endif
        }else if(CH == 8){
          #ifdef ENA_CH8
          TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH8, TimeraSpecifyOutputHigh);
          #endif
        }
				TIMERA_Cmd(TIMERA_UNIT2,Enable);
		}
    else
		{
				stc_timera_compare_init_t stcTimerCompareInit;
				MEM_ZERO_STRUCT(stcTimerCompareInit);
			
				 /* Configuration timera unit 1 compare structure */
				stcTimerCompareInit.u16CompareVal = duty;   //1250		
				stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
				stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
			
				stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputReverse;
				stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
				stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
				
				stcTimerCompareInit.enCacheEn = Disable;
				stcTimerCompareInit.enTriangularTroughTransEn = Disable;
				stcTimerCompareInit.enTriangularCrestTransEn = Disable;
				stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
				/* Configure Channel 1 */
        if(CH == 2){
          #ifdef ENA_CH2
          TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH2, &stcTimerCompareInit);
          TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH2, Enable);
          #endif
        }else if(CH == 8){
          #ifdef ENA_CH8
          TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH8, &stcTimerCompareInit);
          TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH8, Enable);
          #endif
        }
			  TIMERA_Cmd(TIMERA_UNIT2,Enable);
		}		
}

void set_steering_gear_dutyfactor(uint16_t dutyfactor, uint8_t CH)
{
	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0 > dutyfactor ? 0 : dutyfactor;
	dutyfactor = TIMERA_COUNT_OVERFLOW < dutyfactor ? TIMERA_COUNT_OVERFLOW : dutyfactor;

  set_duty_cycle(dutyfactor, CH);
}

void set_pwm_hw(uint16_t pwm_value, uint16_t max_value, uint8_t CH)  //0-255
{
  //"good teacher" algorithm
  float gtValue = (float)pwm_value / (float) max_value;// * 100.0;

  pwm_value = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW);    // 占空比
  set_steering_gear_dutyfactor(pwm_value, CH);    // 设置占空比
}

void pwm_init()
{
  //Port_Init();
	Tim_Config();
  set_pwm_hw(0, 255);
}
#endif    //HWPWM
