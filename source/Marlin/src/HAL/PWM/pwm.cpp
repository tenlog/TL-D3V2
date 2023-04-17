#include "hc32_ddl.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"



#ifdef HWPWM
#include "pwm.h"

static void Port_Init(void)
{
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
		stcPortInit.enPullUp = Disable;
		stcPortInit.enExInt = Enable;
	
    /* Initialize servo motor pin */
		PORT_Init(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN, &stcPortInit);
		PORT_ResetBits(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN);
}

static void TimeraUnit1_IrqCallback(void)
{
    TIMERA_ClearFlag(TIMERA_UNIT2, TimeraFlagOverflow);
}

static void Tim_Config(void)
{
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    //stc_timera_hw_startup_config_t stcTimeraHwConfig;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    //MEM_ZERO_STRUCT(stcTimeraHwConfig);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT2_CLOCK, Enable);
    /* Configuration TIMERA compare pin */
    PORT_SetFunc(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN, TIMERA_UNIT2_CH2_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv4;  //100 000 000 /4
    stcTimeraInit.enCntMode = TimeraCountModeTriangularWave;

    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
	  stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW; // //freq: 5000Hz -> 100 000 000 /4/5000HZ/4 = 1250(period val)
    TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
		
    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = stcTimeraInit.u16PeriodVal/2;   // 50%的占空比
	  stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
	  stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputHigh;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputReverse;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
					
    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Disable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
    TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH2, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH2, Enable);

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH1;
    stcIrqRegiConf.pfnCallback = &TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
		

    TIMERA_Cmd(TIMERA_UNIT2,Enable);	
    TIMERA_SetCompareValue( TIMERA_UNIT2,TIMERA_UNIT2_CH2, TIMERA_COUNT_OVERFLOW*0.1);
    TIMERA_SpecifyOutputSta(TIMERA_UNIT2,TIMERA_UNIT2_CH2,TimeraSpecifyOutputInvalid);	
}

void set_duty_cycle(uint16_t duty)
{
		if(duty == 0)
		{
        TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH2, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TIMERA_UNIT2,Enable);
		}
    else if(duty >= TIMERA_COUNT_OVERFLOW)
		{
        TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH2, TimeraSpecifyOutputHigh);
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
				TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH2, &stcTimerCompareInit);
				TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH2, Enable);
			  TIMERA_Cmd(TIMERA_UNIT2,Enable);
		}		
}

void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0 > dutyfactor ? 0 : dutyfactor;
	dutyfactor = TIMERA_COUNT_OVERFLOW < dutyfactor ? TIMERA_COUNT_OVERFLOW : dutyfactor;

  set_duty_cycle(dutyfactor);
}

void set_pwm_f0(uint16_t pwm_value, uint16_t max_value)  //0-255
{
  //"good teacher" algorithm
  float gtValue = (float)pwm_value / (float) max_value;// * 100.0;
  //gtValue = SQRT(gtValue) * 10.0;
  //gtValue = gtValue / 100.0;

  pwm_value = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW);    // 占空比
  set_steering_gear_dutyfactor(pwm_value);    // 设置占空比
}

void pwm_init()
{
  Port_Init();
	Tim_Config();
  set_pwm_f0(0, 255);
}
#endif    //HWPWM
