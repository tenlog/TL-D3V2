#include "hc32_ddl.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef HWPWM
#include "pwm.h"

static void TimeraUnit_IrqCallback_1(void)
{
    TIMERA_ClearFlag(TIMERA_UNIT1, TimeraFlagOverflow);
}

static void TimeraUnit_IrqCallback_2(void)
{
    TIMERA_ClearFlag(TIMERA_UNIT2, TimeraFlagOverflow);
}

static void Tim_Config(const uint8_t UN){

    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;
    //stc_timera_hw_startup_config_t stcTimeraHwConfig;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    //MEM_ZERO_STRUCT(stcTimeraHwConfig);

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
		stcPortInit.enPullUp = Disable;
		stcPortInit.enExInt = Enable;
    
    /* Initialize servo motor pin */
    if(UN == 2){
      PORT_Init(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN, &stcPortInit);
      PORT_ResetBits(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN);
    }else if(UN == 1){
      PORT_Init(TIMERA_UNIT1_CH4_PORT, TIMERA_UNIT1_CH4_PIN, &stcPortInit);
      PORT_ResetBits(TIMERA_UNIT1_CH4_PORT, TIMERA_UNIT1_CH4_PIN);
    }

    if(UN == 2){
      /* Configuration peripheral clock */
      PWC_Fcg2PeriphClockCmd(TIMERA_UNIT2_CLOCK, Enable);
      /* Configuration TIMERA compare pin */
      PORT_SetFunc(TIMERA_UNIT2_CH2_PORT, TIMERA_UNIT2_CH2_PIN, TIMERA_UNIT2_CH2_FUNC, Disable);
    }else if(UN == 1){
      /* Configuration peripheral clock */
      PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
      /* Configuration TIMERA compare pin */
      PORT_SetFunc(TIMERA_UNIT1_CH4_PORT, TIMERA_UNIT1_CH4_PIN, TIMERA_UNIT1_CH4_FUNC, Disable);
    }

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv4;  //100 000 000 /4
    stcTimeraInit.enCntMode = TimeraCountModeTriangularWave;

    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    if(UN == 2){
      stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW_2; // //freq: 5000Hz -> 200 000 000 /4/5000HZ/4 = 2500(period val)
      TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
    }else if(UN == 1){
      stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW_1; // 
      TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);
    }
		
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
    
    if(UN == 2){
      /* Configure Channel 1 */
      TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH2, &stcTimerCompareInit);
      TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH2, Enable);
      /* Enable period count interrupt */
      TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);
     /* Interrupt of timera unit 1 */
      stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
      stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH_2;
      stcIrqRegiConf.pfnCallback = &TimeraUnit_IrqCallback_2;
    }else if(UN == 1){
      /* Configure Channel 1 */
      TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH4, &stcTimerCompareInit);
      TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH4, Enable);
      /* Enable period count interrupt */
      TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);
     /* Interrupt of timera unit 1 */
      stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
      stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH_1;
      stcIrqRegiConf.pfnCallback = &TimeraUnit_IrqCallback_1;
    }

    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
		
    if(UN == 2){
      TIMERA_Cmd(TIMERA_UNIT2,Enable);	
      TIMERA_SetCompareValue( TIMERA_UNIT2,TIMERA_UNIT2_CH2, TIMERA_COUNT_OVERFLOW_2*0.1);
      TIMERA_SpecifyOutputSta(TIMERA_UNIT2,TIMERA_UNIT2_CH2,TimeraSpecifyOutputInvalid);	
    }else if(UN == 1){
      TIMERA_Cmd(TIMERA_UNIT2,Enable);	
      TIMERA_SetCompareValue( TIMERA_UNIT1,TIMERA_UNIT1_CH4, TIMERA_COUNT_OVERFLOW_1*0.1);
      TIMERA_SpecifyOutputSta(TIMERA_UNIT1,TIMERA_UNIT1_CH4, TimeraSpecifyOutputInvalid);	
    } 
}

void set_duty_cycle(uint16_t duty, uint8_t UN){

    char temp[31];
    sprintf(temp, "duty: %d", duty);
    TLDEBUG_PRINTLN(temp);

		if(duty == 0 && UN == 2){
        TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH2, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TIMERA_UNIT2,Enable);
		} else if(duty >= TIMERA_COUNT_OVERFLOW_2 && UN == 2){
        TIMERA_SpecifyOutputSta(TIMERA_UNIT2, TIMERA_UNIT2_CH2, TimeraSpecifyOutputHigh);
				TIMERA_Cmd(TIMERA_UNIT2,Enable);
		}else if(duty == 0 && UN == 1){
        TIMERA_SpecifyOutputSta(TIMERA_UNIT1, TIMERA_UNIT1_CH4, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TIMERA_UNIT1,Enable);
		}else if(duty >= TIMERA_COUNT_OVERFLOW_1 && UN == 1){
        TIMERA_SpecifyOutputSta(TIMERA_UNIT1, TIMERA_UNIT1_CH4, TimeraSpecifyOutputHigh);
				TIMERA_Cmd(TIMERA_UNIT1,Enable);
		}else{

				stc_timera_compare_init_t stcTimerCompareInit;
				MEM_ZERO_STRUCT(stcTimerCompareInit);
			
				 /* Configuration timera unit 1 compare structure */
				stcTimerCompareInit.u16CompareVal = duty; 		
				stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
				stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
			
				stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputReverse;
				stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
				stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
				
				stcTimerCompareInit.enCacheEn = Disable;
				stcTimerCompareInit.enTriangularTroughTransEn = Disable;
				stcTimerCompareInit.enTriangularCrestTransEn = Disable;
				stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;

        if(UN == 2){
          /* Configure Channel 2 */
          TIMERA_CompareInit(TIMERA_UNIT2, TIMERA_UNIT2_CH2, &stcTimerCompareInit);
          TIMERA_CompareCmd(TIMERA_UNIT2, TIMERA_UNIT2_CH2, Enable);
          TIMERA_Cmd(TIMERA_UNIT2,Enable);
        }else if(UN == 1){
          /* Configure Channel 4 */
          TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH4, &stcTimerCompareInit);
          TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH4, Enable);
          TIMERA_Cmd(TIMERA_UNIT1,Enable);
        }
		}
}

void set_steering_gear_dutyfactor(uint16_t dutyfactor, uint8_t UN)
{
  char temp[31];
  sprintf(temp, "dutyfactor: %d", dutyfactor);
  TLDEBUG_PRINTLN(temp);

	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0 > dutyfactor ? 0 : dutyfactor;
  if(UN == 2){
  	dutyfactor = TIMERA_COUNT_OVERFLOW_2 < dutyfactor ? TIMERA_COUNT_OVERFLOW_2 : dutyfactor;
  }else if(UN == 1){
  	dutyfactor = TIMERA_COUNT_OVERFLOW_1 < dutyfactor ? TIMERA_COUNT_OVERFLOW_1 : dutyfactor;
  }

  set_duty_cycle(dutyfactor, UN);
}

void set_pwm_hw(uint16_t pwm_value, uint16_t max_value, uint8_t unitNo)  //0-255
{
  uint16_t pwm_value_V = 0;
  float gtValue = (float)pwm_value / (float) max_value;// * 100.0;

  if(unitNo == 2){
    pwm_value_V = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW_2);    // 占空比
  }else if(unitNo == 1) {
    pwm_value_V = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW_1);    // 占空比
  }
  set_steering_gear_dutyfactor(pwm_value_V, unitNo);    // 设置占空比
}

void pwm_init()
{
	Tim_Config(2);
  set_pwm_hw(0, 255, 2);
  #ifdef TLTOUCH
 	Tim_Config(1);
  set_pwm_hw(0, 255, 1);
  #endif
}
#endif    //HWPWM
