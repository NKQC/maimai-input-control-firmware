/*******************************************************************************
 * File Name: cycfg_peripherals.c
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

#include "cycfg_peripherals.h"

const cy_stc_scb_i2c_config_t scb_1_config =
{
    .i2cMode = CY_SCB_I2C_MASTER_SLAVE,
    .useRxFifo = true,
    .useTxFifo = true,
    .slaveAddress = 16,
    .slaveAddressMask = 254,
    .acceptAddrInFifo = false,
    .ackGeneralAddr = false,
    .enableWakeFromSleep = false,
    .enableDigitalFilter = false,
    .lowPhaseDutyCycle = 16,
    .highPhaseDutyCycle = 9,
    .delayInFifoAddress = 0,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t scb_1_obj =
{
    .type = CYHAL_RSC_SCB,
    .block_num = 1U,
    .channel_num = 0U,
};
#endif /* defined (CY_USING_HAL) */

cy_stc_csd_context_t cy_csd_0_context =
{
    .lockKey = CY_CSD_NONE_KEY,
};

void init_cycfg_peripherals(void)
{
    Cy_SysClk_PeriphAssignDivider(PCLK_SCB1_CLOCK, CY_SYSCLK_DIV_16_BIT, 5U);
    Cy_SysClk_PeriphAssignDivider(PCLK_CSD_CLOCK, CY_SYSCLK_DIV_16_5_BIT, 0U);
}
void reserve_cycfg_peripherals(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&scb_1_obj);
#endif /* defined (CY_USING_HAL) */
}
