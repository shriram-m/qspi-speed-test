/*******************************************************************************
* File Name: cycfg_pins.c
*
* Description:
* Pin configuration
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

#include "cycfg_pins.h"

const cy_stc_gpio_pin_config_t PIN_SW2_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_PULLUP,
	.hsiom = PIN_SW2_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t SMIF_SS0_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = SMIF_SS0_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t SMIF_DATA3_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG,
	.hsiom = SMIF_DATA3_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t SMIF_DATA2_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG,
	.hsiom = SMIF_DATA2_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t SMIF_DATA1_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG,
	.hsiom = SMIF_DATA1_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t SMIF_DATA0_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG,
	.hsiom = SMIF_DATA0_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t SMIF_SPI_CLOCK_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = SMIF_SPI_CLOCK_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t LED_RED_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = LED_RED_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t UART_TX_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = UART_TX_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_4_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_STRONG_IN_OFF,
	.hsiom = ioss_0_port_6_pin_4_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_6_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_PULLUP,
	.hsiom = ioss_0_port_6_pin_6_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};
const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_7_config = 
{
	.outVal = 1,
	.driveMode = CY_GPIO_DM_PULLDOWN,
	.hsiom = ioss_0_port_6_pin_7_HSIOM,
	.intEdge = CY_GPIO_INTR_DISABLE,
	.intMask = 0UL,
	.vtrip = CY_GPIO_VTRIP_CMOS,
	.slewRate = CY_GPIO_SLEW_FAST,
	.driveSel = CY_GPIO_DRIVE_FULL,
	.vregEn = 0UL,
	.ibufMode = 0UL,
	.vtripSel = 0UL,
	.vrefSel = 0UL,
	.vohSel = 0UL,
};


void init_cycfg_pins(void)
{
	Cy_GPIO_Pin_Init(PIN_SW2_PORT, PIN_SW2_PIN, &PIN_SW2_config);

	Cy_GPIO_Pin_Init(SMIF_SS0_PORT, SMIF_SS0_PIN, &SMIF_SS0_config);

	Cy_GPIO_Pin_Init(SMIF_DATA3_PORT, SMIF_DATA3_PIN, &SMIF_DATA3_config);

	Cy_GPIO_Pin_Init(SMIF_DATA2_PORT, SMIF_DATA2_PIN, &SMIF_DATA2_config);

	Cy_GPIO_Pin_Init(SMIF_DATA1_PORT, SMIF_DATA1_PIN, &SMIF_DATA1_config);

	Cy_GPIO_Pin_Init(SMIF_DATA0_PORT, SMIF_DATA0_PIN, &SMIF_DATA0_config);

	Cy_GPIO_Pin_Init(SMIF_SPI_CLOCK_PORT, SMIF_SPI_CLOCK_PIN, &SMIF_SPI_CLOCK_config);

	Cy_GPIO_Pin_Init(LED_RED_PORT, LED_RED_PIN, &LED_RED_config);

	Cy_GPIO_Pin_Init(UART_TX_PORT, UART_TX_PIN, &UART_TX_config);

	Cy_GPIO_Pin_Init(ioss_0_port_6_pin_4_PORT, ioss_0_port_6_pin_4_PIN, &ioss_0_port_6_pin_4_config);

	Cy_GPIO_Pin_Init(ioss_0_port_6_pin_6_PORT, ioss_0_port_6_pin_6_PIN, &ioss_0_port_6_pin_6_config);

	Cy_GPIO_Pin_Init(ioss_0_port_6_pin_7_PORT, ioss_0_port_6_pin_7_PIN, &ioss_0_port_6_pin_7_config);
}
