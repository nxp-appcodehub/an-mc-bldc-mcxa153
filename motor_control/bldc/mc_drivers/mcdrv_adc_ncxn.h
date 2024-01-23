/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCDRV_ADC_NCXN_H_
#define _MCDRV_ADC_NCXN_H_

#include "fsl_lpadc.h"

#include "gdflib.h"
#include "mlib_types.h"
#include "gmclib.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MCDRV_ADC (3)

/* init sensors/actuators pointers */
#define M1_SET_PTR_U_DC_BUS(par1) (g_sM1AdcSensor.pf16UDcBus = &(par1))
#define M1_SET_PTR_I_DC_BUS(par1) (g_sM1AdcSensor.pf16IDcBus = &(par1))
#define M1_SET_PTR_I_ABC(par1) (g_sM1AdcSensor.psIABC = &(par1))
#define M1_SET_PTR_SECTOR(par1) (g_sM1AdcSensor.pui16SVMSector = &(par1))
#define M1_SET_PTR_AUX_CHAN(par1) (g_sM1AdcSensor.pui16AuxChan = &(par1))

typedef struct _mcdrv_adc
{
    GDFLIB_FILTER_MA_T_A32 ui16FiltDcCurr; /* Dc-bus current offset filter */
    GDFLIB_FILTER_MA_T_A32 ui16FiltBemfPhA; /* Bemf-phA offset filter */
    GDFLIB_FILTER_MA_T_A32 ui16FiltBemfPhB; /* Bemf-phB offset filter */
    GDFLIB_FILTER_MA_T_A32 ui16FiltBemfPhC; /* Bemf-phC offset filter */
  
    ADC_Type * pToAdcBase;

    frac16_t *pf16IDcBus;                  /* pointer to DC-bus  current variable */
    frac16_t *pf16UDcBus;                  /* pointer to DC-bus  voltage variable */
    frac16_t *pf16BemfVoltage;             /* pointer to actual BEMF voltage     */

    uint16_t ui16OffsetFiltWindow;         /* ADC offset filter window */
    uint16_t ui16OffsetDcCurr;             /* Dc-bus current offset */
    uint16_t ui16CalibDcCurr;              /* Dc-bus current offset calibration */
    
    uint16_t ui16OffsetBemfPhA;            /* Bemf-phA offset */
    uint16_t ui16CalibBemfPhA;             /* Bemf-phA offset calibration */
    
    uint16_t ui16OffsetBemfPhB;            /* Bemf-phB offset */
    uint16_t ui16CalibBemfPhB;             /* Bemf-phB calibration */
    
    uint16_t ui16OffsetBemfPhC;            /* Bemf-phC offset */
    uint16_t ui16CalibBemfPhC;             /* Bemf-phC calibration */
    
    uint16_t ui16Sector;                   /* commutation sector */ 

    lpadc_conv_result_t s_ADC_ResultStructure;
} mcdrv_adc_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Function initializes phase current channel offset measurement
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
void MCDRV_CurrOffsetCalibInit(mcdrv_adc_t *this);


#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_ADC_NCXN_H_ */
