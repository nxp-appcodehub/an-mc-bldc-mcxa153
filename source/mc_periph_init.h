/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * Motor Control - Configuration File
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_

#include "fsl_device_registers.h"
#include "mcdrv_eflexpwm.h"
#include "mcdrv_adc_ncxn.h"
#include "m1_bldc_appconfig.h"
#include "m1_sm_ref_sol.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
   
/* Structure used during clocks and modulo calculations */
typedef struct _clock_setup
{
    uint32_t ui32FastPeripheralClock;
    uint32_t ui32CpuFrequency;
    uint32_t ui32BusClock;
    uint32_t ui32SysPllClock;
    uint16_t ui16M1SpeedLoopFreq;
    uint16_t ui16M1SpeedLoopModulo;
    uint16_t ui16M1PwmFreq;
    uint16_t ui16M1PwmModulo;
    uint16_t ui16M1PwmDeadTime;
    uint16_t ui32CmtTimerFreq;
} clock_setup_t;

typedef struct _mcdrv_ctimer_cmt
{
    CTIMER_Type *pui32CtimerBase;     /* pointer CTIMER base address */
    uint16_t *pui16CtimerCntAct;   /* pointer to actual value of CTIMER counter */
    uint16_t *pui16CtimerMatchAct; /* pointer to actual value of CTIMER match value register */
    uint16_t ui16ChannelNum;       /* number of MATCH channel used for compare event */
} mcdrv_ctimer_cmt_t;
      
/******************************************************************************
 * Clock & PWM definition
 ******************************************************************************/
/******************************************************************************
 * Timing
 ******************************************************************************/
/* MCU core clock */
#define MCU_CLOCK_FREQ          (192000000U) /* 192 MHz */
/* PWM frequency in Hz*/
#define M1_PWM_FREQ             (20000U)
/* PWM modulo = FTM_input_clock / M1_PWM_FREQ */
#define M1_PWM_MODULO           (MCU_CLOCK_FREQ / M1_PWM_FREQ)
/* Output PWM deadtime value in nanoseconds */
#define M1_PWM_DEADTIME (500)
/* PWM vs. Fast control loop ratio */
#define M1_FOC_FREQ_VS_PWM_FREQ (1U)
/* Fast loop frequency in Hz */
#define M1_FAST_LOOP_FREQ       (M1_PWM_FREQ / M1_FOC_FREQ_VS_PWM_FREQ)
/* Slow control loop frequency */
#define M1_SLOW_LOOP_FREQ       (1000U)
/* Over Current Fault detection */
#define M1_FAULT_NUM (0)

 /******************************************************************************
  * Output control
  ******************************************************************************/ 

/* Top and Bottom transistors PWM polarity */
#define M1_PWM_POL_TOP         (1U)
#define M1_PWM_POL_BOTTOM      (1U)

/******************************************************************************
 * ADC measurement definition
 ******************************************************************************/
/* Configuration of ADC channels according to the input pin signal */
#define BEMF_A_CHANNEL_NUMBER           6U
#define BEMF_B_CHANNEL_NUMBER           7U
#define BEMF_C_CHANNEL_NUMBER           8U
#define VOLT_DCB_CHANNEL_NUMBER         9U     
#define CUR_DCB_CHANNEL_NUMBER          10U   
   
#define M1_ADC0_PH_A (6)
#define M1_ADC0_PH_B (7)
#define M1_ADC0_PH_C (8)
#define M1_ADC0_UDCB (9)
#define M1_ADC0_IDCB (10)

/******************************************************************************
 * MC driver macro definition and check - do not change this part
 ******************************************************************************/
/******************************************************************************
 * Define motor ADC control functions
 ******************************************************************************/
#define M1_MCDRV_ADC_GET(par) \
    MCDRV_BemfVoltageGet(par); \
    MCDRV_VoltDcBusGet(par);   \
    MCDRV_CurrDcBusGet(par);
#define M1_MCDRV_ADC_ASSIGN_BEMF(par) MCDRV_AssignBemfChannel(par)
#define M1_MCDRV_CURR_CALIB_INIT(par) MCDRV_CurrOffsetCalibInit(par)
#define M1_MCDRV_CURR_CALIB_SET(par) MCDRV_CurrOffsetCalibSet(par)
      
/******************************************************************************
 * Define motor 3-ph PWM control functions
 ******************************************************************************/
#define M1_MCDRV_PWM3PH_SET_PWM_OUTPUT(par1) MCDRV_eFlexPwm3PhOutEn(par1)
#define M1_MCDRV_PWM3PH_SET_DUTY(par1, par2) MCDRV_eFlexPwm3PhSet(par1, par2)
      
/******************************************************************************
 * define motor 1 asynchronous time event functions                           *
 ******************************************************************************/
#define M1_MCDRV_TMR_CMT_SET(par1, par2) MCDRV_CTimerCmtSet(par1, par2)
#define M1_MCDRV_TMR_CMT_GET(par) MCDRV_CTimerCmtGet(par)
    
/******************************************************************************
 * global variable definitions
 ******************************************************************************/
extern clock_setup_t g_sClockSetup;
extern mcdrv_eflexpwm_t g_sM1Pwm3ph;
extern mcdrv_adc_t g_sM1AdcSensor;

/*******************************************************************************
 * API
 ******************************************************************************/

void MCDRV_Init_M1(void);
void InitFlexPWM0(void);
void InitSlowLoop(void);
void InitForceCommutation(void);
void InitADC0(void);
void InitClock(void);
void InitInputmux(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _MC_PERIPH_INIT_H_  */
