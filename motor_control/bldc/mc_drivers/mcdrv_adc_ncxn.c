/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_adc_ncxn.h"
#include "fsl_gpio.h"

#include "mc_periph_init.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Function initializes phase current channel offset measurement
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
void MCDRV_CurrOffsetCalibInit(mcdrv_adc_t *this)
{
    /* clear offset values */
    this->ui16OffsetDcCurr = 0x3fff;
    this->ui16CalibDcCurr  = 0;
    
    this->ui16OffsetBemfPhA = 0;
    this->ui16CalibBemfPhA  = 0;

    this->ui16OffsetBemfPhB = 0;
    this->ui16CalibBemfPhB  = 0;
    
    this->ui16OffsetBemfPhC = 0;
    this->ui16CalibBemfPhC  = 0;
    
    /* initialize offset filters */
    this->ui16FiltDcCurr.u16Sh = this->ui16OffsetFiltWindow;
    this->ui16FiltBemfPhA.u16Sh = this->ui16OffsetFiltWindow;
    this->ui16FiltBemfPhB.u16Sh = this->ui16OffsetFiltWindow;
    this->ui16FiltBemfPhC.u16Sh = this->ui16OffsetFiltWindow;
        
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->ui16FiltDcCurr);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->ui16FiltBemfPhA);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->ui16FiltBemfPhB);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->ui16FiltBemfPhC);    
}

