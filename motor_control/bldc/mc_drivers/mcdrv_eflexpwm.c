/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_eflexpwm.h"
#include "mc_periph_init.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static bool_t s_statusPass;

/*******************************************************************************
 * Code
 ******************************************************************************/
    int16_t i16Duty;
/*!
 * @brief Function updates FTM value register
 *
 * @param this   Pointer to the current object
 * TODO - Deadtime compensation?
 * @return none
 */
void MCDRV_eFlexPwm3PhSet(mcdrv_eflexpwm_t *this, int16_t i16InpDuty)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;



    i16Duty = MLIB_Mul_F16((i16InpDuty), (this->ui16PwmModulo) / 2);
 
    if (i16Duty > ((this->ui16PwmModulo) / 2)) i16Duty = (this->ui16PwmModulo) / 2;

    pCurrentPwm->SM[0].VAL2 = -i16Duty; // rising edge value register update
    pCurrentPwm->SM[0].VAL3 = i16Duty;  // falling edge value register update, no need to calculate it

    pCurrentPwm->SM[1].VAL2 = -i16Duty; // rising edge value register update
    pCurrentPwm->SM[1].VAL3 = i16Duty; // falling edge value register update, no need to calculate it

    pCurrentPwm->SM[2].VAL2 = -i16Duty; // rising edge value register update
    pCurrentPwm->SM[2].VAL3 = i16Duty; // falling edge value register update, no need to calculate it

    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(15);

}

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutEn(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    /* Clear fault flag (we're in safe mode so the PWM won't run if there's an error condition */
    pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(1U);

    /* Start PWMs (set load OK flags and run) */
    pCurrentPwm->MCTRL |= PWM_MCTRL_CLDOK(15);
    pCurrentPwm->MCTRL |= PWM_MCTRL_LDOK(15);
    pCurrentPwm->MCTRL |= PWM_MCTRL_RUN(15);

    /* Enable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMA_EN(0xF);
    pCurrentPwm->OUTEN |= PWM_OUTEN_PWMB_EN(0xF);

}

/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutDis(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    /* Disable A&B (Top & Bottm) PWM outputs of submodules one, two and three */
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMA_EN(0xF));
    pCurrentPwm->OUTEN &= (~PWM_OUTEN_PWMB_EN(0xF));

}

/*!
 * @brief Function initialite PWM outputs structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutInit(mcdrv_eflexpwm_t *this)
{

}

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_eFlexPwm3PhFltGet(mcdrv_eflexpwm_t *this)
{
    /* read over-current flags */
//    s_statusPass = (((this->pui32PwmBaseAddress->FSTS & PWM_FSTS_FFPIN_MASK) >> 8) &
//                    (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));
  
    s_statusPass = (((0U) >> 8) &
                    (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));
  
    /* clear faults flag */
    this->pui32PwmBaseAddress->FSTS = ((this->pui32PwmBaseAddress->FSTS & ~(uint16_t)(PWM_FSTS_FFLAG_MASK)) |
                                       (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum));

    return ((s_statusPass > 0));
}

void MCDRV_eFlexPwm3PhFltTryClr(mcdrv_eflexpwm_t *this)
{
    PWM_Type * const pCurrentPwm = this->pui32PwmBaseAddress;

    /* We can clear the FFLAGs only if the respective FFPIN (raw fault input) isn't set. */
    const uint8_t u8FfpinNoErrorMask =
                    (uint8_t)(~(((pCurrentPwm->FSTS) & PWM_FSTS_FFLAG_MASK) >> PWM_FSTS_FFLAG_SHIFT));

    pCurrentPwm->FSTS |= PWM_FSTS_FFLAG(u8FfpinNoErrorMask);
}

/*!
 * @brief Function set pwm sector from input
 *
 * @param this Pointer to the current object
 * @param sector Actual commutation sector
 *
 * @return boot_t true on success
 */
void MCDRV_eFlexPwmSetPwmOutput(mcdrv_eflexpwm_t *this, int16_t i16Sector, int16_t dir)
{
    this->pui32PwmBaseAddress->DTSRCSEL = *((this->pcBldcTable) + 2 * i16Sector);
    this->pui32PwmBaseAddress->MASK     = *((this->pcBldcTable) + 2 * i16Sector+1);

    //this->pui32PwmBaseAddress->MCTRL |= PWM_MCTRL_LDOK(15);
    this->pui32PwmBaseAddress->SM[0].CTRL2 |= PWM_CTRL2_FORCE_MASK;
}
