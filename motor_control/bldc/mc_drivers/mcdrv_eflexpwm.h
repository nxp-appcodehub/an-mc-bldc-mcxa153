/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCDRV_EFLEXPWM_H_
#define _MCDRV_EFLEXPWM_H_

#include "mlib.h"
#include "mlib_types.h"
#include "fsl_device_registers.h"
#include "gmclib.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MCDRV_PWMA (1)

/* init sensors/actuators pointers */
#define M1_SET_PTR_DUTY(par1) (g_sM1Pwm3ph.psUABC = &(par1))

typedef struct _mcdrv_pwm3ph_pwma
{
    GMCLIB_3COOR_T_F16 *psUABC; /* pointer to the 3-phase PWM duty cycles */
    PWM_Type *pui32PwmBaseAddress;     /* pointer to phase A top value */
    uint16_t ui16ChanPhA;       /* number of channel for phase A */
    uint16_t ui16ChanPhB;       /* number of channel for phase A top */
    uint16_t ui16ChanPhC;       /* number of channel for phase B bottom */
    uint16_t ui16PwmModulo;     /* FTM MODULO Value */
    
    uint16_t ui16FaultFixNum;   /* FTM fault number for fixed over-current fault detection */
    uint16_t ui16FaultAdjNum; /* PWMA fault number for adjustable over-current fault detection */
    
    const uint16_t *pcBldcTable;    /* pointer to BLDC commutation Table */
} mcdrv_eflexpwm_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Function updates FTM value register
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhSet(mcdrv_eflexpwm_t *this, int16_t i16InpDuty);

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutEn(mcdrv_eflexpwm_t *this);

/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutDis(mcdrv_eflexpwm_t *this);

/*!
 * @brief Function initialite PWM outputs structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_eFlexPwm3PhOutInit(mcdrv_eflexpwm_t *this);

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_eFlexPwm3PhFltGet(mcdrv_eflexpwm_t *this);

/*!
 * @brief Try to clear the FFLAG (if FFPIN is not set).
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
void MCDRV_eFlexPwm3PhFltTryClr(mcdrv_eflexpwm_t *this);

/*!
 * @brief Function set pwm sector from input
 *
 * @param this Pointer to the current object
 * @param sector Actual commutation sector
 *
 * @return boot_t true on success
 */
void MCDRV_eFlexPwmSetPwmOutput(mcdrv_eflexpwm_t *this, int16_t i16Sector, int16_t dir);

#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_EFLEXPWM_H_ */
