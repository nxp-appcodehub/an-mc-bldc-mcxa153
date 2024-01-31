/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mc_periph_init.h"
#include "peripherals.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_inputmux.h"
#include "fsl_port.h"
#include "fsl_lpadc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* commutation table */
/* commutation table */
const uint16_t bldcCommutationTableComp[28] = {
    /*DTSRCSEL, MASK*/
    0x0040, 0x0440,/* [0] - sector 0 */
    0x0400, 0x0220,/* [1] - sector 1 */
    0x0400, 0x0110,/* [2] - sector 2 */
    0x0004, 0x0440,/* [3] - sector 3 */
    0x0004, 0x0220,/* [4] - sector 4 */
    0x0040, 0x0110,/* [5] - sector 5 */
    0x0440, 0x0000/* [6] - alignment vector (combination of sectors 0 & 1) */
};
/*******************************************************************************
 * Functions
 ******************************************************************************/
#if M1_FAULT_ENABLE   
static void InitCMP(void);
#endif /* M1_FAULT_ENABLE */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* configuration structure for 3-phase PWM mc driver */
mcdrv_eflexpwm_t g_sM1Pwm3ph;

/* structure for current and voltage measurement*/
mcdrv_adc_t g_sM1AdcSensor;;

/* Clock setup structure */
clock_setup_t g_sClockSetup;

/* structure for time event driver */
mcdrv_ctimer_cmt_t g_sM1CmtTmr;

/*! @brief Main control structure */
extern mcdef_bldc_t g_sM1Drive;

static const lpadc_reference_voltage_source_t DEMO_LPADC_VREF_SOURCE =
    kLPADC_ReferenceVoltageAlt3;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   void MCDRV_Init_M1(void)
 *           - Motor control driver main initialization
 *           - Calls initialization functions of peripherals required for motor
 *             control functionality
 *
 * @param   void
 *
 * @return  none
 */
void MCDRV_Init_M1(void)
{
   /* Init application clock dependent variables */
    InitClock();
    
    /* Init slow loop counter */
    InitSlowLoop();
    
    /* Init Forced commutation counter*/
    InitForceCommutation();
    
    /* Init signal multiplexing */
    InitInputmux();             
    
    /* 6-channel PWM peripheral init for M1 */
    InitFlexPWM0();
    
    /* Init ADC */
    InitADC0();
    
    /* Init actuators pointers */
    g_sM1AdcSensor.pf16UDcBus           = &(g_sM1Drive.sCtrlBLDC.f16UDcBusNoFilt);
    g_sM1AdcSensor.pf16IDcBus           = &(g_sM1Drive.sCtrlBLDC.f16IDcBusNoFilt);
    g_sM1AdcSensor.pf16BemfVoltage      = &(g_sM1Drive.sCtrlBLDC.f16UPhase);    
    g_sM1CmtTmr.pui16CtimerCntAct       = &(g_sM1Drive.ui16TimeCurrent);
    g_sM1CmtTmr.pui16CtimerMatchAct     = &(g_sM1Drive.ui16TimeCurrentEvent);
    
#if M1_FAULT_ENABLE    
    /* Comparator ACOMP */
    InitCMP();
#endif /* M1_FAULT_ENABLE */
}

/*!
* @brief   void InitClock(void)
*          - Core, bus, flash clock setup
*
* @param   void
*
* @return  none
*/
void InitClock(void)
{
    uint32_t ui32CyclesNumber = 0U;

    /* Calculate clock dependant variables for PMSM control algorithm */
    g_sClockSetup.ui32FastPeripheralClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    g_sClockSetup.ui32CpuFrequency = CLOCK_GetFreq(kCLOCK_CoreSysClk);

    /* Parameters for motor */
    g_sClockSetup.ui16M1PwmFreq   = M1_PWM_FREQ; /* 10 kHz */
    g_sClockSetup.ui16M1PwmModulo = (g_sClockSetup.ui32FastPeripheralClock) / g_sClockSetup.ui16M1PwmFreq;
    ui32CyclesNumber = ((M1_PWM_DEADTIME * (g_sClockSetup.ui32FastPeripheralClock / 1000000U)) / 1000U);
    g_sClockSetup.ui16M1PwmDeadTime   = ui32CyclesNumber;
    g_sClockSetup.ui16M1SpeedLoopFreq = M1_SLOW_LOOP_FREQ; /* 1kHz */
}


/*!
 * @brief   void InitPWM(void)
 *           - Initialization of the FTM0 peripheral for motor M1
 *           - 3-phase center-aligned PWM
 *
 * @param   void
 *
 * @return  none
 */
void InitFlexPWM0(void)
{
    uint32_t u32TmpRegisterContent;
    uint8_t u8SubmoduleIndex;
    
    RESET_PeripheralReset(kFLEXPWM0_RST_SHIFT_RSTn);     
    CLOCK_EnableClock(kCLOCK_GateCTIMER0);

    // Enable eFlexPWM1 AHB clock
    CLOCK_EnableClock(kCLOCK_GateFLEXPWM0);
    
    // Enable Submodules 0 - 2 clocks
    SYSCON->PWM0SUBCTL |= SYSCON_PWM0SUBCTL_CLK0_EN_MASK |
                          SYSCON_PWM0SUBCTL_CLK1_EN_MASK |
                          SYSCON_PWM0SUBCTL_CLK2_EN_MASK;

    /* Init SMs 0 - 2 *********************************************************/
    
    for( u8SubmoduleIndex = 0U; u8SubmoduleIndex < 3 ; ++u8SubmoduleIndex )
    {
        FLEXPWM0->SM[u8SubmoduleIndex].INIT = PWM_INIT_INIT((uint16_t)(-(M1_PWM_MODULO / 2)));
        FLEXPWM0->SM[u8SubmoduleIndex].VAL0 = 0U;
        FLEXPWM0->SM[u8SubmoduleIndex].VAL1 = PWM_VAL1_VAL1((uint16_t)((M1_PWM_MODULO / 2) - 1));
        FLEXPWM0->SM[u8SubmoduleIndex].VAL2 = (uint16_t)(-(M1_PWM_MODULO / 4));
        FLEXPWM0->SM[u8SubmoduleIndex].VAL3 = (uint16_t)((M1_PWM_MODULO / 4) - 1);

        /* Reload is generated at every opportunity */
        FLEXPWM0->SM[u8SubmoduleIndex].CTRL |= PWM_CTRL_LDFQ(0);
        
        FLEXPWM0->SM[u8SubmoduleIndex].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M1PwmDeadTime);
        FLEXPWM0->SM[u8SubmoduleIndex].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M1PwmDeadTime);
        /* Full cycle reload */
        FLEXPWM0->SM[u8SubmoduleIndex].CTRL |= PWM_CTRL_FULL_MASK;
        
        /* Setup SM1 & SM2 to be driven by SM0 signals */
        if( u8SubmoduleIndex > 0U )
        {
            u32TmpRegisterContent = FLEXPWM0->SM[u8SubmoduleIndex].CTRL2;
          
            /* Set SM1 & SM2 to be clocked by SM0 */
            u32TmpRegisterContent &= (~PWM_CTRL2_CLK_SEL_MASK);
            u32TmpRegisterContent |= (PWM_CTRL2_CLK_SEL(0x02U));
            
            /* Master reload signal from SM0 is used to reload registers of SM1 & SM2*/
            u32TmpRegisterContent |= PWM_CTRL2_RELOAD_SEL_MASK;
            
            u32TmpRegisterContent |= PWM_CTRL2_FORCE_SEL(1U);
            
            /* Master sync from SM0 causes initialization of SM1 & SM2 */
            u32TmpRegisterContent &= (~PWM_CTRL2_INIT_SEL_MASK);
            u32TmpRegisterContent |= PWM_CTRL2_INIT_SEL(0x02U);
            
            FLEXPWM0->SM[u8SubmoduleIndex].CTRL2 = u32TmpRegisterContent;
        }

        /* Setup fault 0 - 2 (CMPs 0 - 2 outputs) trigger */
        FLEXPWM0->SM[u8SubmoduleIndex].DISMAP[0] = 0xF000U;
        
    }    

    /* PWM1 module 0 trigger on VAL4 enabled for ADC synchronization */
    FLEXPWM0->SM[0].VAL4 = (uint16_t)(-(M1_PWM_MODULO / 2));
    FLEXPWM0->SM[0].VAL5 = 0U;
    FLEXPWM0->SM[0].TCTRL |= 0x30;
    
    /* PWM outputs are re-enabled at PWM full cycle */
    FLEXPWM0->FSTS |= PWM_FSTS_FFULL(0x07U);
    
    /* PWM fault filter - 3 Fast periph. clocks sample rate,
       2 clock cycles filter period */
    FLEXPWM0->FFILT = (FLEXPWM0->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(2U);
    
    /* Setup PWM fault control register */
    u32TmpRegisterContent = FLEXPWM0->FCTRL;
    
    u32TmpRegisterContent &= ~(PWM_FCTRL_FLVL_MASK | PWM_FCTRL_FAUTO_MASK | PWM_FCTRL_FSAFE_MASK | PWM_FCTRL_FIE_MASK);
    u32TmpRegisterContent |= PWM_FCTRL_FIE(0U);         /* FAULT 0 & FAULT 1 - Interrupt disable */
    u32TmpRegisterContent |= PWM_FCTRL_FLVL(0U);        /* A logic 0 on the fault input indicates a fault condition */
    u32TmpRegisterContent |= PWM_FCTRL_FAUTO(0U);       /* Manual fault clearing */
    u32TmpRegisterContent |= PWM_FCTRL_FSAFE(0x7U);     /* Outputs 0 - 2 set to safe mode */
    
    FLEXPWM0->FCTRL = u32TmpRegisterContent;
    
    /* Clear all fault flags */
    FLEXPWM0->FSTS |= PWM_FSTS_FFLAG(0x07U);

    /* Start PWM1 SMs 0-2 (set load OK flags and run) */
    FLEXPWM0->MCTRL |= PWM_MCTRL_CLDOK(0x07U);
    FLEXPWM0->MCTRL |= PWM_MCTRL_LDOK(0x07U);
    FLEXPWM0->MCTRL |= PWM_MCTRL_RUN(0x07U);

    /* Enable  PWM0 "A" and "B" output of SMs 0-2,*/
    FLEXPWM0->OUTEN |= PWM_OUTEN_PWMA_EN(0x07U);
    FLEXPWM0->OUTEN |= PWM_OUTEN_PWMB_EN(0x07U);

    /* ---------------------------------------- */
    /* Initialization FTM 3-phase PWM mc driver */
    g_sM1Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)(FLEXPWM0); /* FTM base address */
    //g_sM1Pwm3ph.ui16ChanPhA  = M1_PWM_PAIR_PHA;            /* PWM phase A top&bottom channel pair number */
    //g_sM1Pwm3ph.ui16ChanPhB  = M1_PWM_PAIR_PHB;            /* PWM phase B top&bottom channel pair number */
    //g_sM1Pwm3ph.ui16ChanPhC  = M1_PWM_PAIR_PHC;            /* PWM phase C top&bottom channel pair number */

    /* FTM Fault number for over current fault detection */
    g_sM1Pwm3ph.ui16FaultFixNum = M1_FAULT_NUM; 
    
    /* initialization of PWM modulo */
    //g_sM1Pwm3ph.ui16PwmModulo = g_sClockSetup.ui16M1PwmModulo;
    g_sM1Pwm3ph.ui16PwmModulo = M1_PWM_MODULO;
    
    /* initialization of BLDC commucation table */
    g_sM1Pwm3ph.pcBldcTable = &bldcCommutationTableComp[0];
}

/*!
 * @brief   void InitSlowLoop(void)
 *           - Initialization of the CTIMER0 peripheral
 *           - performs slow control loop counter
 *
 * @param   void
 *
 * @return  none
 */
void InitSlowLoop(void)
{
    RESET_PeripheralReset(kCTIMER0_RST_SHIFT_RSTn );     

    /* Use 96 MHz clock for some of the Ctimer0. */
    CLOCK_SetClockDiv(kCLOCK_DivCTIMER0, 2u);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);
    
    CLOCK_EnableClock(kCLOCK_GateCTIMER0);

    /* Configure match control register. */
    CTIMER0->MCR |= CTIMER_MCR_MR0R(1U)  |   /* Enable reset of TC after it matches with MR0. */
                    CTIMER_MCR_MR0I(1U);     /* Enable interrupt generation after TC matches with MR0. */
    
    /* Configure match register. */
    CTIMER0->MR[0] = (CLOCK_GetCTimerClkFreq(0U))  /* Get CTimer0 frequency for correct set Match register value. */
                     / M1_SLOW_LOOP_FREQ;           /* Set slow control loop frequency in Hz. */
    
    /* Configure interrupt register. */
    CTIMER0->IR = CTIMER_IR_MR0INT_MASK;     /* Set interrupt flag for match channel 0. */
    //NVIC_SetPriority(CTIMER0_IRQn, 1U);
    NVIC_EnableIRQ(CTIMER0_IRQn);            /* Enable LEVEL1 interrupt and update the call back function. */

    /* Configure timer control register. */
    CTIMER0->TCR |= CTIMER_TCR_CEN_MASK;     /* Start the timer counter. */
}

/*!
 * @brief   void InitSlowLoop(void)
 *           - Initialization of the CTIMER0 peripheral
 *           - performs slow control loop counter
 *
 * @param   void
 *
 * @return  none
 */
void InitForceCommutation(void)
{
    RESET_PeripheralReset(kCTIMER1_RST_SHIFT_RSTn );     

    /* Use 96 MHz clock for some of the Ctimer1. */
    CLOCK_SetClockDiv(kCLOCK_DivCTIMER1, 2u);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER1);
    
    CLOCK_EnableClock(kCLOCK_GateCTIMER1);
    
    CTIMER1->PR =159U;

    /* Configure match control register. */
    CTIMER1->MCR |= CTIMER_MCR_MR0R(1U)  |   /* Enable reset of TC after it matches with MR0. */
                    CTIMER_MCR_MR1I(1U);     /* Enable interrupt generation after TC matches with MR1. */
    
    /* Configure match register. */
    CTIMER1->MR[0] = 0xFFFF; /* Get CTimer1 frequency for correct set Match register value. */
     
    CTIMER1->MR[1] = 0xFFFF;
    
    /* Configure interrupt register. */
    CTIMER1->IR = CTIMER_IR_MR1INT_MASK ;     /* Set interrupt flag for match channel 0. */
 
    NVIC_EnableIRQ(CTIMER1_IRQn);            /* Enable LEVEL1 interrupt and update the call back function. */

    /* Configure timer control register. */
    CTIMER1->TCR |= CTIMER_TCR_CEN_MASK;     /* Start the timer counter. */
}

/*!
 * @brief   void InitADC(void)
 *           - Initialization of the ADC16 peripheral
 *           - Initialization of the A/D converter for current and voltage sensing
 *
 * @param   void
 *
 * @return  none
 */
void InitADC0(void)
{
    lpadc_conv_trigger_config_t lpadcTriggerConfig;
    lpadc_conv_command_config_t lpadcCommandConfig;
    lpadc_config_t lpadcConfig;
    
    CLOCK_EnableClock(kCLOCK_GateADC0);    
    /* Set clocks */
    CLOCK_SetClockDiv(kCLOCK_DivADC0, 4);
    CLOCK_AttachClk(kFRO_HF_to_ADC0);
    
    // Init the lpadcConfig struct
    LPADC_GetDefaultConfig(&lpadcConfig);
    lpadcConfig.enableAnalogPreliminary = true;
    lpadcConfig.referenceVoltageSource = DEMO_LPADC_VREF_SOURCE;
    lpadcConfig.conversionAverageMode = kLPADC_ConversionAverage64;
    RESET_PeripheralReset(kADC0_RST_SHIFT_RSTn);  
    
    ADC0->CTRL |=ADC_CTRL_RST_MASK;
    ADC0->CTRL &=(~ADC_CTRL_RST_MASK);
    ADC0->CTRL |=ADC_CTRL_RSTFIFO0_MASK;
    
    LPADC_Init(ADC0, &lpadcConfig);

    //LPADC_DoOffsetCalibration(ADC0);
    ADC0->CTRL |= ADC_CTRL_CALOFS_MASK;
    while (ADC_STAT_CAL_RDY_MASK != (ADC0->STAT & ADC_STAT_CAL_RDY_MASK)){}
    
    LPADC_GetDefaultConvCommandConfig(&lpadcCommandConfig);

    lpadcCommandConfig.channelNumber = 20U;//20_21_2
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
    lpadcCommandConfig.sampleTimeMode = kLPADC_SampleTimeADCK19;
    lpadcCommandConfig.chainedNextCommandNumber = 0U;
    LPADC_SetConvCommandConfig( ADC0, 1, &lpadcCommandConfig );
    
    lpadcCommandConfig.channelNumber = 1U;
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
    lpadcCommandConfig.sampleTimeMode = kLPADC_SampleTimeADCK19;
    lpadcCommandConfig.chainedNextCommandNumber = 3U;
    LPADC_SetConvCommandConfig( ADC0, 2, &lpadcCommandConfig );
    
    lpadcCommandConfig.channelNumber = 4U;
    lpadcCommandConfig.conversionResolutionMode = kLPADC_ConversionResolutionStandard;
    lpadcCommandConfig.sampleTimeMode = kLPADC_SampleTimeADCK19;
    lpadcCommandConfig.chainedNextCommandNumber = 0U;
    LPADC_SetConvCommandConfig( ADC0, 3, &lpadcCommandConfig );
    
    /* Init triggers (use trigger 0). */
    LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfig);
    lpadcTriggerConfig.targetCommandId = 2U;
    lpadcTriggerConfig.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC0, 0U, &lpadcTriggerConfig);
    
    lpadcTriggerConfig.targetCommandId = 1U;
    lpadcTriggerConfig.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC0, 1U, &lpadcTriggerConfig);
    
    /* Enable TCOMP interrupt. */
    LPADC_EnableInterrupts(ADC0, ADC_IE_TCOMP_IE(0x2U));
    NVIC_SetPriority(ADC0_IRQn, 0U);
    NVIC_EnableIRQ(ADC0_IRQn);
    
    g_sM1AdcSensor.pToAdcBase = (ADC_Type *)ADC0;
          
    /* prepare first measurement */
    /* pass initialization structure to ADC MC driver */
    //g_sM1Adc16Init.ui16AdcArray  = (&ui16AdcArray[0]);
    //g_sM1Adc16Init.pui32Adc0Base = (ADC_Type *)ADC0;
    //MCDRV_Adc16Init(&g_sM1AdcSensor, &g_sM1Adc16Init);
    
    g_sM1AdcSensor.ui16OffsetFiltWindow = 3;
    g_sM1AdcSensor.ui16OffsetDcCurr = 0x3fff;
        
}

/*!
@brief   void InitCMP(void)
          - Initialization of the comparator 1 module for dc-bus over current
            detection to generate FTM0 fault

@param   void

@return  none
*/

#if M1_FAULT_ENABLE
static void InitCMP(void)
{
  
}
#endif /* M1_FAULT_ENABLE */

/*!
 * @brief   void InitInputmux(void)
 *           - Initialization of the Input Multiplexing
 *
 * @param   void
 *
 * @return  none
 */
void InitInputmux(void)
{
    RESET_PeripheralReset(kINPUTMUX0_RST_SHIFT_RSTn);     
  
    CLOCK_EnableClock(kCLOCK_InputMux);

    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Pwm0Sm0OutTrig0ToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, 1U, kINPUTMUX_Pwm0Sm0OutTrig1ToAdc0Trigger);
}

