/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SM_REF_SOL_COMM_H_
#define _SM_REF_SOL_COMM_H_

#include "bldc_control.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Application info */
typedef struct _app_ver
{
    char cBoardID[20];
    char cExampleID[30];
    char cAppVer[5];
    uint16_t ui16FeatureSet;
} app_ver_t;

/*! @brief device fault typedef */
typedef uint16_t mcdef_fault_t;

/*! @brief States of machine enumeration */
typedef enum _run_substate_t
{
    kRunState_Calib     = 0,
    kRunState_Ready     = 1,
    kRunState_Align     = 2,
    kRunState_Startup   = 3,
    kRunState_Spin      = 4,
    kRunState_Freewheel = 5,
} run_substate_t; /* Run sub-states */

/*! @brief Control modes of the motor */
typedef enum _mcs_ctrl_mode_t
{
    kControlMode_Scalar     = 0,
    kControlMode_VoltageFOC = 1,
    kControlMode_CurrentFOC = 2,
    kControlMode_SpeedFOC   = 3,
    kControlMode_OpenLoop 	= 4,
} mcs_control_mode_t;

/*! @brief Device fault thresholds */
typedef struct _mcdef_fault_thresholds_t
{
    frac16_t f16IDcBusOver;  /* DC bus over current level */
    frac16_t f16UDcBusOver;  /* DC bus over voltage level */
    frac16_t f16UDcBusUnder; /* DC bus under voltage level */   uint16_t ui16BlockedPerNum; /* Number of period to set blocked rotor fault */
} mcdef_fault_thresholds_t;

/*! @brief BLDC sensorless with BEMF integration method */
typedef struct _mcdef_bldc_t
{
    mcs_bldc_ctrl_t sCtrlBLDC;                 /* Main BLDC control structure */
    mcdef_fault_t sFaultId;                    /* Application motor faults */
    mcdef_fault_t sFaultIdPending;             /* Fault pending structure */
    mcdef_fault_thresholds_t sFaultThresholds; /* Fault thresholds */
    uint16_t ui16PeriodCmt[6];                 /* commutation periods */
    uint16_t ui16TimeCurrent;                  /* current time */
    uint16_t ui16TimeCurrentEvent;             /* time of current event */
    uint16_t ui16TimeNextEvent;                /* time of next event */
    uint16_t ui16TimeOfCmt;                    /* current commutation time */
    uint16_t ui16TimeOfCmtOld;                 /* previous commutation time */
    uint16_t ui16PeriodCmtNext;                /* next commutation period */
    uint16_t ui16PeriodToff;                   /* Toff period */
    uint16_t ui16CounterStartCmt;              /* startup commutations counter */
    uint16_t ui16CounterCmtError;              /* commutation error counter */
    frac32_t f32UBemfIntegSum;                 /* BEMF integrator */
    frac32_t f32UBemfIntegThreshold;           /* BEMF integration threshold */
    bool_t bCommutatedSnsless;                 /* commutated by sensorless algorithm flag */
    uint16_t ui16PeriodFreewheelLong;          /* long free-wheel period */
    uint16_t ui16PeriodFreewheelShort;         /* short free-wheel period */
    uint16_t ui16TimeAlignment;                /* alignment period */
    uint16_t ui16TimeCalibration;              /* Calibration time count number */
    uint16_t ui16TimeFaultRelease;             /* Fault time count number */
    uint16_t ui16CounterState;                 /* Main state counter */
    bool_t bFaultClearMan;                     /* Manual fault clear detection */
    frac16_t f16StartCmtAcceleration;          /* Startup commutation acceleration init value*/
    uint16_t ui16StartCmtNumber;               /* Startup commutation counter init value */
    uint16_t ui16PeriodCmtNextInit;            /* Next commutation period init value */
    uint16_t ui16PeriodToffInit;               /* Toff period init value */
    uint16_t ui16Aux;                          /* auxiliary quantity measured value */
    uint16_t ui16FreqCtrlLoop;                 /* Pass defined loop frequency to FreeMASTER */
    uint32_t ui32FreqCmtTimer;                 /* Pass defined commutation timer frequency to FreeMASTER */
    uint32_t ui16FreqPwm;                      /* Pass defined PWM frequency to FreeMASTER */
} mcdef_bldc_t;

#define FAULT_I_DCBUS_OVER (0U)  /* OverCurrent fault flag */
#define FAULT_U_DCBUS_UNDER (1U) /* Undervoltage fault flag */
#define FAULT_U_DCBUS_OVER (2U)  /* Overvoltage fault flag */
#define FAULT_LOAD_OVER (3U)     /* Overload fault flag */
#define FAULT_SPEED_OVER (4U)    /* Over speed fault flag */
#define FAULT_ROTOR_BLOCKED (5U) /* Blocked rotor fault flag */

/* Sets the fault bit defined by faultid in the faults variable */
#define FAULT_SET(faults, faultid) ((faults) |= (((mcdef_fault_t)1U) << (faultid)))

/* Clears the fault bit defined by faultid in the faults variable */
#define FAULT_CLEAR(faults, faultid) ((faults) &= ~(((mcdef_fault_t)1U) << (faultid)))

/* Check the fault bit defined by faultid in the faults variable, returns 1 or 0 */
#define FAULT_CHECK(faults, faultid) (((faults) & (((mcdef_fault_t)1U) << (faultid))) >> (faultid))

/* Clears all fault bits in the faults variable */
#define FAULT_CLEAR_ALL(faults) ((faults) = 0U)

/* Check if a fault bit is set in the faults variable, 0 = no fault */
#define FAULT_ANY(faults) ((faults) > 0U)

/* Update a fault bit defined by faultid in the faults variable according to the LSB of value */
#define FAULT_UPDATE(faults, faultid, value)                     \
    {                                                               \
        FAULT_CLEAR(faults, faultid);                            \
        faults |= (((FAULT_T)value & (FAULT_T)1) << faultid); \
    }


/*******************************************************************************
 * Variables
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _SM_REF_SOL_COMM_H_ */
