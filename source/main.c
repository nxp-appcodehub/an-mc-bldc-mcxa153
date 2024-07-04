/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "pin_mux.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_lpuart.h"
#include "mc_periph_init.h"
#include "m1_sm_ref_sol.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Example's feature set in form of bits inside ui16featureSet.
   This feature set is expected to be growing over time.
   ... | FEATURE_S_RAMP | FEATURE_FIELD_WEAKENING | FEATURE_ENC
*/
#define FEATURE_ENC (0)               /* Encoder feature flag */
#define FEATURE_FIELD_WEAKENING (0)   /* Field weakening feature flag */
#define FEATURE_S_RAMP (0)            /* S-ramp feature flag */

#define FEATURE_SET (FEATURE_ENC << (0) | \
                     FEATURE_FIELD_WEAKENING << (1) | \
                     FEATURE_S_RAMP << (2))

/* Macro for correct Cortex CM0 / CM4 end of interrupt */
#define M1_END_OF_ISR \
    {                 \
        __DSB();      \
        __ISB();      \
    }

/* CPU load measurement SysTick START / STOP macros */
#define SYSTICK_START_COUNT() (SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    uint32_t val  = SysTick->VAL;  \
    uint32_t load = SysTick->LOAD; \
    par1          = load - val
      
static void BOARD_Init(void);
static void BOARD_InitSysTick(void);
static void DemoSpeedStimulator(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* CPU load measurement using Systick*/
uint32_t g_ui32NumberOfCycles    = 0;
uint32_t g_ui32MaxNumberOfCycles = 0;

/* Demo mode enabled/disabled */
bool_t bDemoMode = FALSE;

/* Used for demo mode */
static uint32_t ui32SpeedStimulatorCnt = 0;

/* Counter for button pressing */
static uint32_t ui32ButtonFilter = 0;

uint32_t ctimer0_isr_cnt        =0u;
uint32_t ctimer1_isr_cnt        =0u;
uint32_t adc0_isr_cnt           =0u;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   Application main function processing peripheral function calling and
 *          infinite loop
 *
 * @param   void
 *
 * @return  none
 */
void main(void)
{ 
    uint32_t ui32PrimaskReg;

    /* disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    /* disable demo mode after reset */
    bDemoMode              = FALSE;
    ui32SpeedStimulatorCnt = 0;

    /* Init board hardware. */
    BOARD_Init();
    
    LED_RED_OFF();
    LED_BLUE_OFF();
    LED_GREEN_OFF();

    /* initialize peripheral motor control driver for motor M1*/
    MCDRV_Init_M1();

    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();
    
    /* Turn off application */
    M1_SetAppSwitch(FALSE);

    /* Enable interrupts */
    EnableGlobalIRQ(ui32PrimaskReg);
    
    /* infinite loop */
    while (1)
    {
    }
}

static void BOARD_Init(void)
{
    /* Initialize clock configuration */
    BOARD_InitBootClocks();
    /* Init pins set in pin_mux file */
    BOARD_InitBootPins();
}

/*!
 * @brief   ADC conversion complete ISR called with 100us period processes
 *           - motor M1 fast application machine function
 *
 * @param   void
 *
 * @return  none
 */
void ADC0_IRQHandler(void)
{
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();
    
    GPIO1->PSOR |= 1<<10U;
    adc0_isr_cnt++;

    /* Start CPU tick number couting */
    SYSTICK_START_COUNT();

    /* Read available converted values from the FIFO. */
    while( LPADC_GetConvResult(g_sM1AdcSensor.pToAdcBase, &g_sM1AdcSensor.s_ADC_ResultStructure) )
    {       
    	switch( g_sM1AdcSensor.s_ADC_ResultStructure.commandIdSource )
    	{
    		case 1U:
                        /* Command 1 */
    			*g_sM1AdcSensor.pf16BemfVoltage = (frac16_t)(g_sM1AdcSensor.s_ADC_ResultStructure.convValue) ;
    			break;

    		case 2U:
                        /* Command 2 */
    			*g_sM1AdcSensor.pf16UDcBus    = (frac16_t)(g_sM1AdcSensor.s_ADC_ResultStructure.convValue);
    			break;

    		case 3U:
                        /* Command 3 */
    			*g_sM1AdcSensor.pf16IDcBus    = (frac16_t)(g_sM1AdcSensor.s_ADC_ResultStructure.convValue);
    			break;                          
    		default:
    			break;
    	}
    }
    
    /* DC-Bus current filter */
    g_sM1Drive.sCtrlBLDC.f16IDcBus =
        GDFLIB_FilterIIR1_F16(g_sM1Drive.sCtrlBLDC.f16IDcBusNoFilt-g_sM1AdcSensor.ui16OffsetDcCurr, &g_sM1Drive.sCtrlBLDC.sIDcBusFilter);

    /* DC-Bus voltage filter */
    g_sM1Drive.sCtrlBLDC.f16UDcBus =
        GDFLIB_FilterIIR1_F16(g_sM1Drive.sCtrlBLDC.f16UDcBusNoFilt, &g_sM1Drive.sCtrlBLDC.sUDcBusFilter);

    switch (g_sM1AdcSensor.ui16Sector)
    {
        /* BEMF phase C sensing */
        case 0:
        case 3:
          g_sM1Drive.sCtrlBLDC.f16UPhaseBemf =
            MLIB_Sub_F16(MLIB_Sub_F16(g_sM1Drive.sCtrlBLDC.f16UPhase, g_sM1AdcSensor.ui16OffsetBemfPhC),g_sM1Drive.sCtrlBLDC.f16UDcBus>>1);           
          break;
          
        /* BEMF phase B sensing */
        case 1:
        case 4:
          g_sM1Drive.sCtrlBLDC.f16UPhaseBemf =
            MLIB_Sub_F16(MLIB_Sub_F16(g_sM1Drive.sCtrlBLDC.f16UPhase, g_sM1AdcSensor.ui16OffsetBemfPhB),g_sM1Drive.sCtrlBLDC.f16UDcBus>>1);        
          break;
          
        /* BEMF phase A sensing */
        case 2:
        case 5:
          g_sM1Drive.sCtrlBLDC.f16UPhaseBemf =
            MLIB_Sub_F16(MLIB_Sub_F16(g_sM1Drive.sCtrlBLDC.f16UPhase, g_sM1AdcSensor.ui16OffsetBemfPhA),g_sM1Drive.sCtrlBLDC.f16UDcBus>>1);  
          break;
          
        default:
          break;
    }

    /* M1 state machine */    
    SM_StateMachineFast(&g_sM1Ctrl);
    
    /* Stop CPU tick number couting and store actual and maximum ticks */
    SYSTICK_STOP_COUNT(g_ui32NumberOfCycles);
    g_ui32MaxNumberOfCycles =
        g_ui32NumberOfCycles > g_ui32MaxNumberOfCycles ? g_ui32NumberOfCycles : g_ui32MaxNumberOfCycles;

    /* Enable interrupts  */
    EnableGlobalIRQ(ui32PrimaskReg);

    /* Clear the TCOMP INT flag */
    ADC0->STAT |= (uint32_t)(1U << 9);
    
    GPIO1->PCOR |= 1<<10U;
    
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}

/*!
 * @brief   FTM1 interrupt service routine
 *          - Forced commutation control
 *
 * @param   void
 *
 * @return  none
 */
void CTIMER1_IRQHandler(void)
{
    ctimer1_isr_cnt++;

    /* asynchronous time event processing */
    M1_TimeEvent();

    /* Clear the match interrupt flag. */
    CTIMER1->IR |= CTIMER_IR_MR1INT_MASK;

    /* add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}

/*!
 * @brief   Slow loop ISR
 *
 * @param   void
 *
 * @return  none
 */
void CTIMER0_IRQHandler(void)
{
    static int16_t ui16i = 0;

    ctimer0_isr_cnt++;

    /* M1 Slow StateMachine call */
    SM_StateMachineSlow(&g_sM1Ctrl);
    
        /* If in STOP state turn on RED */
    if (M1_GetAppState() == 2)
    {
        LED_RED_ON();
        LED_GREEN_OFF();
    }

    /* If in FAULT state RED blinking*/
    else if (M1_GetAppState() == 0)
    {
        if (ui16i-- < 0)
        {
            LED_RED_TOGGLE();
            bDemoMode= FALSE;
            ui16i = 125;
        }
        LED_GREEN_OFF();
    }

    /* If in RUN or INIT state turn on green */
    else
    {
        LED_RED_OFF();
        LED_GREEN_ON();
    }

    /* Demo speed stimulator */
    DemoSpeedStimulator();

    /* Clear the match interrupt flag. */
    CTIMER0->IR |= CTIMER_IR_MR0INT(1U);

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}


/*!
 * @brief   DemoSpeedStimulator
 *           - When demo mode is enabled it changes the required speed according
 *             to predefined profile
 *
 * @param   void
 *
 * @return  none
 */
void DemoSpeedStimulator(void)
{
    /* increase push button pressing counter  */
    if (ui32ButtonFilter < 1000)
        ui32ButtonFilter++;

    if (bDemoMode)
    {
        ui32SpeedStimulatorCnt++;
        switch (ui32SpeedStimulatorCnt)
        {
            case 1:
               M1_SetSpeed(FRAC16(1000.0 / M1_N_MAX));
                break;
            case 2500:
                M1_SetSpeed(FRAC16(1500.0 / M1_N_MAX));
                break;
            case 5000:
                M1_SetSpeed(FRAC16(2000.0 / M1_N_MAX));
                break;    
            case 7500:
                M1_SetSpeed(FRAC16(1000.0 / M1_N_MAX));
                break;
            case 10000:
                M1_SetSpeed(FRAC16(-2000.0 / M1_N_MAX));
                break;
            case 20000:
                ui32SpeedStimulatorCnt = 0;
                break;
            default:
                break;
              
        }
    }
}


/*!
 * @brief   Port interrupt handler
 *
 * @param   void
 *
 * @return  none
 */
void GPIO3_IRQHandler(void)
{
    if (GPIO3->ISFR[0] & GPIO_ISFR_ISF29_MASK)
    {
        /* clear the flag */
        GPIO3->ISFR[0] |= GPIO_ISFR_ISF29_MASK;

        /* proceed only if pressing longer than timeout */
        if (ui32ButtonFilter > 200U)
        {
            ui32ButtonFilter = 0U;
            if (bDemoMode)
            {
                M1_SetSpeed(0);
                M1_SetAppSwitch(FALSE);
                bDemoMode = FALSE;
            }
            else
            {
                M1_SetAppSwitch(TRUE);
                bDemoMode              = TRUE;
                ui32SpeedStimulatorCnt = 0U;
            }
        }
    }

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}


/*!
 *@brief      SysTick initialization for CPU cycle measurement
 *
 *@param      none
 *
 *@return     none
 */
void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
