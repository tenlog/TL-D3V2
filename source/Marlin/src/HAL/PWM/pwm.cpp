#include "hc32_ddl.h"
#include "../../MarlinCore.h"
#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"
#include "../../lcd/tenlog/tenlog_touch_lcd.h"

#ifdef HWPWM
#include "pwm.h"

#ifdef TLTOUCH
static void TimeraUnit_IrqCallback_tlt(void)
{
    TIMERA_ClearFlag(TAUNIT_TLT, TimeraFlagOverflow);
}
#endif

#ifdef TL_STEPTEST
static void TimeraUnit_IrqCallback_tlts(void)
{
    TIMERA_ClearFlag(TAUNIT_TLTS, TimeraFlagOverflow);
}
#endif

static void TimeraUnit_IrqCallback_F0(void)
{
    TIMERA_ClearFlag(TAUNIT_F0, TimeraFlagOverflow);
}

static void TimeraUnit_IrqCallback_F1(void)
{
    TIMERA_ClearFlag(TAUNIT_F1, TimeraFlagOverflow);
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
  if(UN == UN_F0){
    PORT_Init(TAUNIT_F0_CH_PORT, TAUNIT_F0_CH_PIN, &stcPortInit);
    PORT_ResetBits(TAUNIT_F0_CH_PORT, TAUNIT_F0_CH_PIN);
    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TAUNIT_F0_CLOCK, Enable);
    /* Configuration TIMERA compare pin */
    PORT_SetFunc(TAUNIT_F0_CH_PORT, TAUNIT_F0_CH_PIN, TAUNIT_F0_CH_FUNC, Disable);
  }else if(UN == UN_F1){
    PORT_Init(TAUNIT_F1_CH_PORT, TAUNIT_F1_CH_PIN, &stcPortInit);
    PORT_ResetBits(TAUNIT_F1_CH_PORT, TAUNIT_F1_CH_PIN);
    PWC_Fcg2PeriphClockCmd(TAUNIT_F1_CLOCK, Enable);
    PORT_SetFunc(TAUNIT_F1_CH_PORT, TAUNIT_F1_CH_PIN, TAUNIT_F1_CH_FUNC, Disable);
  }else if(UN == UN_TLT){
    #ifdef TLTOUCH
    PORT_Init(TAUNIT_TLT_CH_PORT, TAUNIT_TLT_CH_PIN, &stcPortInit);
    PORT_ResetBits(TAUNIT_TLT_CH_PORT, TAUNIT_TLT_CH_PIN);
    PWC_Fcg2PeriphClockCmd(TAUNIT_TLT_CLOCK, Enable);
    PORT_SetFunc(TAUNIT_TLT_CH_PORT, TAUNIT_TLT_CH_PIN, TAUNIT_TLT_CH_FUNC, Disable);
    #endif
  }else if(UN == UN_TLTS){
    #ifdef TL_STEPTEST
    PORT_Init(TAUNIT_TLTS_CH_PORT, TAUNIT_TLTS_CH_PIN, &stcPortInit);
    PORT_ResetBits(TAUNIT_TLTS_CH_PORT, TAUNIT_TLTS_CH_PIN);
    PWC_Fcg2PeriphClockCmd(TAUNIT_TLTS_CLOCK, Enable);
    PORT_SetFunc(TAUNIT_TLTS_CH_PORT, TAUNIT_TLTS_CH_PIN, TAUNIT_TLTS_CH_FUNC, Disable);
    #endif
  }

  /* Configuration timera unit 1 base structure */
  stcTimeraInit.enClkDiv = TimeraPclkDiv4;  //200 000 000 /4
  stcTimeraInit.enCntMode = TimeraCountModeTriangularWave;

  stcTimeraInit.enCntDir = TimeraCountDirUp;
  stcTimeraInit.enSyncStartupEn = Disable;
  if(UN == UN_F0){
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW_F0; // //freq: 5000Hz -> 200 000 000 /4/5000HZ/4 = 2500(period val)
    TIMERA_BaseInit(TAUNIT_F0, &stcTimeraInit);
  }else if(UN == UN_F1){
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW_F1; // //freq: 5000Hz -> 200 000 000 /4/5000HZ/4 = 2500(period val)
    TIMERA_BaseInit(TAUNIT_F1, &stcTimeraInit);
  }else if(UN == UN_TLT){
    #ifdef TLTOUCH
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW_TLT; // 
    TIMERA_BaseInit(TAUNIT_TLT, &stcTimeraInit);
    #endif
  }else if(UN == UN_TLTS){
    #ifdef TL_STEPTEST
    uint32_t TIMERA_COUNT_OVERFLOW_TLTS_ = CAL_TLTS_OVERFLOW;
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW_TLTS_; // 
    TIMERA_BaseInit(TAUNIT_TLTS, &stcTimeraInit);
    #endif
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
  
  if(UN == UN_F0){
    /* Configure Channel 1 */
    TIMERA_CompareInit(TAUNIT_F0, TAUNIT_F0_CH, &stcTimerCompareInit);
    TIMERA_CompareCmd(TAUNIT_F0, TAUNIT_F0_CH, Enable);
    /* Enable period count interrupt */
    TIMERA_IrqCmd(TAUNIT_F0, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TAUNIT_F0_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH_F0;
    stcIrqRegiConf.pfnCallback = &TimeraUnit_IrqCallback_F0;
  }else if(UN == UN_F1){
    TIMERA_CompareInit(TAUNIT_F1, TAUNIT_F1_CH, &stcTimerCompareInit);
    TIMERA_CompareCmd(TAUNIT_F1, TAUNIT_F1_CH, Enable);
    TIMERA_IrqCmd(TAUNIT_F1, TimeraIrqOverflow, Enable);
    stcIrqRegiConf.enIntSrc = TAUNIT_F1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH_F1;
    stcIrqRegiConf.pfnCallback = &TimeraUnit_IrqCallback_F1;
  }else if(UN == UN_TLT){
    #ifdef TLTOUCH
    /* Configure Channel 1 */
    TIMERA_CompareInit(TAUNIT_TLT, TAUNIT_TLT_CH, &stcTimerCompareInit);
    TIMERA_CompareCmd(TAUNIT_TLT, TAUNIT_TLT_CH, Enable);
    /* Enable period count interrupt */
    TIMERA_IrqCmd(TAUNIT_TLT, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TAUNIT_TLT_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH_TLT;
    stcIrqRegiConf.pfnCallback = &TimeraUnit_IrqCallback_tlt;
    #endif
  }else if(UN == UN_TLTS){
    #ifdef TL_STEPTEST
    /* Configure Channel 1 */
    TIMERA_CompareInit(TAUNIT_TLTS, TAUNIT_TLTS_CH, &stcTimerCompareInit);
    TIMERA_CompareCmd(TAUNIT_TLTS, TAUNIT_TLTS_CH, Enable);
    /* Enable period count interrupt */
    TIMERA_IrqCmd(TAUNIT_TLTS, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TAUNIT_TLTS_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = IRQ_INDEX_INT_TIMA_CH_TLTS;
    stcIrqRegiConf.pfnCallback = &TimeraUnit_IrqCallback_tlts;
    #endif
  }

  enIrqRegistration(&stcIrqRegiConf);
  NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
  NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
  
  if(UN == UN_F0){
    TIMERA_Cmd(TAUNIT_F0,Enable);	
    TIMERA_SetCompareValue( TAUNIT_F0,TAUNIT_F0_CH, TIMERA_COUNT_OVERFLOW_F0*0.1);
    TIMERA_SpecifyOutputSta(TAUNIT_F0,TAUNIT_F0_CH,TimeraSpecifyOutputInvalid);	
  }else if(UN == UN_F1){
    TIMERA_Cmd(TAUNIT_F1,Enable);	
    TIMERA_SetCompareValue( TAUNIT_F1,TAUNIT_F1_CH, TIMERA_COUNT_OVERFLOW_F1*0.1);
    TIMERA_SpecifyOutputSta(TAUNIT_F1,TAUNIT_F1_CH,TimeraSpecifyOutputInvalid);	
  }else if(UN == UN_TLT){
    #ifdef TLTOUCH
    TIMERA_Cmd(TAUNIT_TLT,Enable);	
    TIMERA_SetCompareValue( TAUNIT_TLT,TAUNIT_TLT_CH, TIMERA_COUNT_OVERFLOW_TLT*0.1);
    TIMERA_SpecifyOutputSta(TAUNIT_TLT,TAUNIT_TLT_CH, TimeraSpecifyOutputInvalid);	
    #endif
  }else if(UN == UN_TLTS){
    #ifdef TL_STEPTEST
    uint32_t TIMERA_COUNT_OVERFLOW_TLTS_ = CAL_TLTS_OVERFLOW;
    TIMERA_Cmd(TAUNIT_TLTS,Enable);	
    TIMERA_SetCompareValue( TAUNIT_TLTS,TAUNIT_TLTS_CH, TIMERA_COUNT_OVERFLOW_TLTS_*0.1);
    TIMERA_SpecifyOutputSta(TAUNIT_TLTS,TAUNIT_TLTS_CH, TimeraSpecifyOutputInvalid);	
    #endif
    } 
}

void set_duty_cycle(uint16_t duty, uint8_t UN){

    //char temp[31];
    //sprintf(temp, "duty: %d", duty);
    //TLDEBUG_PRINTLN(temp);
    #ifdef TL_STEPTEST
      uint32_t TIMERA_COUNT_OVERFLOW_TLTS_ = CAL_TLTS_OVERFLOW;
    #endif

		if(duty == 0 && UN == UN_F0){
        TIMERA_SpecifyOutputSta(TAUNIT_F0, TAUNIT_F0_CH, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TAUNIT_F0,Enable);
		} else if(duty >= TIMERA_COUNT_OVERFLOW_F0 && UN == UN_F0){
        TIMERA_SpecifyOutputSta(TAUNIT_F0, TAUNIT_F0_CH, TimeraSpecifyOutputHigh);
				TIMERA_Cmd(TAUNIT_F0,Enable);
		}else if(duty == 0 && UN == UN_F1){
        TIMERA_SpecifyOutputSta(TAUNIT_F1, TAUNIT_F1_CH, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TAUNIT_F1,Enable);
		} else if(duty >= TIMERA_COUNT_OVERFLOW_F1 && UN == UN_F1){
        TIMERA_SpecifyOutputSta(TAUNIT_F1, TAUNIT_F1_CH, TimeraSpecifyOutputHigh);
				TIMERA_Cmd(TAUNIT_F1,Enable);
    #ifdef TLTOUCH
		}else if(duty == 0 && UN == UN_TLT){
        TIMERA_SpecifyOutputSta(TAUNIT_TLT, TAUNIT_TLT_CH, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TAUNIT_TLT,Enable);
		}else if(duty >= TIMERA_COUNT_OVERFLOW_TLT && UN == UN_TLT){
        TIMERA_SpecifyOutputSta(TAUNIT_TLT, TAUNIT_TLT_CH, TimeraSpecifyOutputHigh);
				TIMERA_Cmd(TAUNIT_TLT,Enable);
    #endif
    #ifdef TL_STEPTEST
		}else if(duty == 0 && UN == UN_TLTS){
        TIMERA_SpecifyOutputSta(TAUNIT_TLTS, TAUNIT_TLTS_CH, TimeraSpecifyOutputLow);
				TIMERA_Cmd(TAUNIT_TLTS,Enable);
		}else if(duty >= TIMERA_COUNT_OVERFLOW_TLTS_ && UN == UN_TLTS){
        TIMERA_SpecifyOutputSta(TAUNIT_TLTS, TAUNIT_TLTS_CH, TimeraSpecifyOutputHigh);
				TIMERA_Cmd(TAUNIT_TLTS,Enable);
    #endif
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

        if(UN == UN_F0){
          TIMERA_CompareInit(TAUNIT_F0, TAUNIT_F0_CH, &stcTimerCompareInit);
          TIMERA_CompareCmd(TAUNIT_F0, TAUNIT_F0_CH, Enable);
          TIMERA_Cmd(TAUNIT_F0,Enable);
        }else if(UN == UN_F1){
          TIMERA_CompareInit(TAUNIT_F1, TAUNIT_F1_CH, &stcTimerCompareInit);
          TIMERA_CompareCmd(TAUNIT_F1, TAUNIT_F1_CH, Enable);
          TIMERA_Cmd(TAUNIT_F1,Enable);
        }else if(UN == UN_TLT){
          #ifdef TLTOUCH
          /* Configure Channel 4 */
          TIMERA_CompareInit(TAUNIT_TLT, TAUNIT_TLT_CH, &stcTimerCompareInit);
          TIMERA_CompareCmd(TAUNIT_TLT, TAUNIT_TLT_CH, Enable);
          TIMERA_Cmd(TAUNIT_TLT,Enable);
          #endif
        }else if(UN == UN_TLTS){
          #ifdef TL_STEPTEST
          /* Configure Channel 4 */
          TIMERA_CompareInit(TAUNIT_TLTS, TAUNIT_TLTS_CH, &stcTimerCompareInit);
          TIMERA_CompareCmd(TAUNIT_TLTS, TAUNIT_TLTS_CH, Enable);
          TIMERA_Cmd(TAUNIT_TLTS,Enable);
          #endif
        }
		}
}

void set_steering_gear_dutyfactor(uint16_t dutyfactor, uint8_t UN)
{
  //char temp[31];
  //sprintf(temp, "dutyfactor: %d", dutyfactor);
  //TLDEBUG_PRINTLN(temp);

	/* 对超过范围的占空比进行边界处理 */
	dutyfactor = 0 > dutyfactor ? 0 : dutyfactor;
  if(UN == UN_F0){
  	dutyfactor = TIMERA_COUNT_OVERFLOW_F0 < dutyfactor ? TIMERA_COUNT_OVERFLOW_F0 : dutyfactor;
  }else if(UN == UN_F1){
  	dutyfactor = TIMERA_COUNT_OVERFLOW_F1 < dutyfactor ? TIMERA_COUNT_OVERFLOW_F1 : dutyfactor;
  }else if(UN == UN_TLT){
    #ifdef TLTOUCH
  	dutyfactor = TIMERA_COUNT_OVERFLOW_TLT < dutyfactor ? TIMERA_COUNT_OVERFLOW_TLT : dutyfactor;
    #endif
  }else if(UN == UN_TLTS){
    #ifdef TL_STEPTEST
    uint32_t TIMERA_COUNT_OVERFLOW_TLTS_ = CAL_TLTS_OVERFLOW;
  	dutyfactor = TIMERA_COUNT_OVERFLOW_TLTS_ < dutyfactor ? TIMERA_COUNT_OVERFLOW_TLTS_ : dutyfactor;
    #endif
  }

  set_duty_cycle(dutyfactor, UN);
}

void set_pwm_hw(uint16_t pwm_value, uint16_t max_value, uint8_t unitNo)  //0-255
{
  uint16_t pwm_value_V = 0;
  float gtValue = (float)pwm_value / (float) max_value;// * 100.0;

  if(unitNo == UN_F0){
    pwm_value_V = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW_F0);    // 占空比
  }else if(unitNo == UN_F1){
    pwm_value_V = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW_F1);    // 占空比
  }else if(unitNo == UN_TLT) {
    #ifdef TLTOUCH
    pwm_value_V = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW_TLT);    // 占空比
    #endif
  }else if(unitNo == UN_TLTS) {
    #ifdef TL_STEPTEST
    uint32_t TIMERA_COUNT_OVERFLOW_TLTS_ = CAL_TLTS_OVERFLOW;
    pwm_value_V = uint16_t(gtValue * (float)TIMERA_COUNT_OVERFLOW_TLTS_);    // 占空比
    #endif
  }
  set_steering_gear_dutyfactor(pwm_value_V, unitNo);    // 设置占空比
}

void pwm_init()
{
	Tim_Config(UN_F0);
  set_pwm_hw(0, 255, UN_F0);
	Tim_Config(UN_F1);
  set_pwm_hw(0, 255, UN_F1);
  #ifdef TLTOUCH
 	Tim_Config(UN_TLT);
  set_pwm_hw(0, 255, UN_TLT);
  #endif
  #ifdef TL_STEPTEST
 	Tim_Config(UN_TLTS);
  set_pwm_hw(0, 255, UN_TLTS);
  #endif
}
#endif    //HWPWM
