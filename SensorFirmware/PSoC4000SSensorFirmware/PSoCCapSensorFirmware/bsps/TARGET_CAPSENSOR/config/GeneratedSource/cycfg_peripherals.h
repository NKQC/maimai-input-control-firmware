/*******************************************************************************
 * File Name: cycfg_peripherals.h
 *
 * Description:
 * Peripheral Hardware Block configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.60.0
 * device-db 4.31.0.9165
 * mtb-pdl-cat2 2.18.0.16566
 *
 *******************************************************************************
 * Copyright 2025 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "cy_scb_i2c.h"
#include "cy_sysclk.h"
#include "cy_csd.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#endif /* defined (CY_USING_HAL) */

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define scb_1_ENABLED 1U
#define scb_1_HW SCB1
#define scb_1_IRQ scb_1_interrupt_IRQn
#define csd_0_csd_0_ENABLED 1U
#define CY_CAPSENSE_CORE 0u
#define CY_CAPSENSE_CPU_CLK 48000000u
#define CY_CAPSENSE_PERI_CLK 48000000u
#define CY_CAPSENSE_VDDA_MV 3300u
#define CY_CAPSENSE_PERI_DIV_TYPE CY_SYSCLK_DIV_16_5_BIT
#define CY_CAPSENSE_PERI_DIV_INDEX 0u
#define csd_0_csd_0_HW CSD0
#define csd_0_csd_0_IRQ csd_interrupt_IRQn

extern const cy_stc_scb_i2c_config_t scb_1_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t scb_1_obj;
#endif /* defined (CY_USING_HAL) */

extern cy_stc_csd_context_t cy_csd_0_context;

void init_cycfg_peripherals(void);
void reserve_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PERIPHERALS_H */
