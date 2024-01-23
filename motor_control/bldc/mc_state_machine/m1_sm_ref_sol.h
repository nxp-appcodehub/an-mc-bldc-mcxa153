/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _M1_STATEMACHINE_H_
#define _M1_STATEMACHINE_H_

#include "m1_bldc_appconfig.h"
#include "state_machine.h"
#include "mc_periph_init.h"
#include "sm_common.h"

/* library headers */
#include "gmclib.h"
#include "gflib.h"
#include "gdflib.h"
#include "amclib.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MC_FAULT_I_DCBUS_OVER 0  /* OverCurrent fault flag */
#define MC_FAULT_U_DCBUS_UNDER 1 /* Undervoltage fault flag */
#define MC_FAULT_U_DCBUS_OVER 2  /* Overvoltage fault flag */

/* Sets the fault bit defined by faultid in the faults variable */
#define MC_FAULT_SET(faults, faultid) (faults |= ((mcdef_fault_t)1 << faultid))

/* Clears the fault bit defined by faultid in the faults variable */
#define MC_FAULT_CLEAR(faults, faultid) (faults &= ~((mcdef_fault_t)1 << faultid))

/* Check the fault bit defined by faultid in the faults variable, returns 1 or 0 */
#define MC_FAULT_CHECK(faults, faultid) ((faults & ((mcdef_fault_t)1 << faultid)) >> faultid)

/* Clears all fault bits in the faults variable */
#define MC_FAULT_CLEAR_ALL(faults) (faults = 0)

/* Check if a fault bit is set in the faults variable, 0 = no fault */
#define MC_FAULT_ANY(faults) (faults > 0)

/* Update a fault bit defined by faultid in the faults variable according to the LSB of value */
#define MC_FAULT_UPDATE(faults, faultid, value)                     \
    {                                                               \
        MC_FAULT_CLEAR(faults, faultid);                            \
        faults |= (((MC_FAULT_T)value & (MC_FAULT_T)1) << faultid); \
    }

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern bool_t g_bM1SwitchAppOnOff;
extern mcdef_bldc_t g_sM1Drive;
extern sm_app_ctrl_t g_sM1Ctrl;
extern run_substate_t s_eM1StateRun;

extern volatile float s_fltM1DCBvoltageScale;
extern volatile float s_fltM1currentScale;
extern volatile float s_fltM1speedScale;

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Set application switch value to On or Off mode
 *
 * @param bValue  bool value, true - On of false - Off
 *
 * @return None
 */
void M1_SetAppSwitch(bool_t bValue);

/*!
 * @brief Get application switch value
 *
 * @param void  No input parameter
 *
 * @return bool_t Return bool value, true or false
 */
bool_t M1_GetAppSwitch(void);

/*!
 * @brief Get application state
 *
 * @param void  No input parameter
 *
 * @return uint16_t Return current application state
 */
uint16_t M1_GetAppState(void);

/*!
 * @brief Set spin speed of the motor in fractional value
 *
 * @param f16SpeedCmd  Speed command - set speed
 *
 * @return None
 */
void M1_SetSpeed(frac16_t f16SpeedCmd);

/*!
 * @brief Get spin speed of the motor in fractional value
 *
 * @param void  No input parameter
 *
 * @return frac16_t Fractional value of the current speed
 */
frac16_t M1_GetSpeed(void);

/*!
 * @brief Forced commutation if regular commutation not detected using BEMF method
 *
 * @param void  No input parameter
 *
 * @return No return value
 */
void M1_TimeEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* STATEMACHINE */
