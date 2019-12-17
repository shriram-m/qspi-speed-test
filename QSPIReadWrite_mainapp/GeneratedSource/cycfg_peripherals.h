/*******************************************************************************
* File Name: cycfg_peripherals.h
*
* Description:
* Peripheral Hardware Block configuration
* This file was automatically generated and should not be modified.
* 
********************************************************************************
* Copyright 2017-2019 Cypress Semiconductor Corporation
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
********************************************************************************/

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "cy_scb_uart.h"
#include "cy_sysclk.h"
#include "cy_smif.h"
#include "cy_tcpwm_counter.h"
#include "cycfg_routing.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define UART_HW SCB5
#define UART_IRQ scb_5_interrupt_IRQn
#define SMIF_HW SMIF0
#define SMIF_IRQ smif_interrupt_IRQn
#define SMIF_MEMORY_MODE_ALIGMENT_ERROR (0UL)
#define SMIF_RX_DATA_FIFO_UNDERFLOW (0UL)
#define SMIF_TX_COMMAND_FIFO_OVERFLOW (0UL)
#define SMIF_TX_DATA_FIFO_OVERFLOW (0UL)
#define SMIF_RX_FIFO_TRIGEER_LEVEL (0UL)
#define SMIF_TX_FIFO_TRIGEER_LEVEL (0UL)
#define SMIF_DATALINES0_1 (1UL)
#define SMIF_DATALINES2_3 (1UL)
#define SMIF_DATALINES4_5 (0UL)
#define SMIF_DATALINES6_7 (0UL)
#define SMIF_SS0 (1UL)
#define SMIF_SS1 (0UL)
#define SMIF_SS2 (0UL)
#define SMIF_SS3 (0UL)
#define SMIF_DESELECT_DELAY 7
#define Timer_HW TCPWM0
#define Timer_NUM 0UL
#define Timer_MASK (1UL << 0)

extern const cy_stc_scb_uart_config_t UART_config;
extern const cy_stc_smif_config_t SMIF_config;
extern const cy_stc_tcpwm_counter_config_t Timer_config;

void init_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_PERIPHERALS_H */
