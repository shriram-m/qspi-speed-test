/*******************************************************************************
* File Name: cycfg_pins.h
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

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "cy_gpio.h"
#include "cycfg_routing.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define PIN_SW2_PORT GPIO_PRT0
#define PIN_SW2_PIN 4U
#define PIN_SW2_NUM 4U
#define PIN_SW2_DRIVEMODE CY_GPIO_DM_PULLUP
#define PIN_SW2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_4_HSIOM
	#define ioss_0_port_0_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define PIN_SW2_HSIOM ioss_0_port_0_pin_4_HSIOM
#define PIN_SW2_IRQ ioss_interrupts_gpio_0_IRQn
#define SMIF_SS0_PORT GPIO_PRT11
#define SMIF_SS0_PIN 2U
#define SMIF_SS0_NUM 2U
#define SMIF_SS0_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define SMIF_SS0_INIT_DRIVESTATE 1
#ifndef ioss_0_port_11_pin_2_HSIOM
	#define ioss_0_port_11_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define SMIF_SS0_HSIOM ioss_0_port_11_pin_2_HSIOM
#define SMIF_SS0_IRQ ioss_interrupts_gpio_11_IRQn
#define SMIF_DATA3_PORT GPIO_PRT11
#define SMIF_DATA3_PIN 3U
#define SMIF_DATA3_NUM 3U
#define SMIF_DATA3_DRIVEMODE CY_GPIO_DM_STRONG
#define SMIF_DATA3_INIT_DRIVESTATE 1
#ifndef ioss_0_port_11_pin_3_HSIOM
	#define ioss_0_port_11_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define SMIF_DATA3_HSIOM ioss_0_port_11_pin_3_HSIOM
#define SMIF_DATA3_IRQ ioss_interrupts_gpio_11_IRQn
#define SMIF_DATA2_PORT GPIO_PRT11
#define SMIF_DATA2_PIN 4U
#define SMIF_DATA2_NUM 4U
#define SMIF_DATA2_DRIVEMODE CY_GPIO_DM_STRONG
#define SMIF_DATA2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_11_pin_4_HSIOM
	#define ioss_0_port_11_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define SMIF_DATA2_HSIOM ioss_0_port_11_pin_4_HSIOM
#define SMIF_DATA2_IRQ ioss_interrupts_gpio_11_IRQn
#define SMIF_DATA1_PORT GPIO_PRT11
#define SMIF_DATA1_PIN 5U
#define SMIF_DATA1_NUM 5U
#define SMIF_DATA1_DRIVEMODE CY_GPIO_DM_STRONG
#define SMIF_DATA1_INIT_DRIVESTATE 1
#ifndef ioss_0_port_11_pin_5_HSIOM
	#define ioss_0_port_11_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define SMIF_DATA1_HSIOM ioss_0_port_11_pin_5_HSIOM
#define SMIF_DATA1_IRQ ioss_interrupts_gpio_11_IRQn
#define SMIF_DATA0_PORT GPIO_PRT11
#define SMIF_DATA0_PIN 6U
#define SMIF_DATA0_NUM 6U
#define SMIF_DATA0_DRIVEMODE CY_GPIO_DM_STRONG
#define SMIF_DATA0_INIT_DRIVESTATE 1
#ifndef ioss_0_port_11_pin_6_HSIOM
	#define ioss_0_port_11_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define SMIF_DATA0_HSIOM ioss_0_port_11_pin_6_HSIOM
#define SMIF_DATA0_IRQ ioss_interrupts_gpio_11_IRQn
#define SMIF_SPI_CLOCK_PORT GPIO_PRT11
#define SMIF_SPI_CLOCK_PIN 7U
#define SMIF_SPI_CLOCK_NUM 7U
#define SMIF_SPI_CLOCK_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define SMIF_SPI_CLOCK_INIT_DRIVESTATE 1
#ifndef ioss_0_port_11_pin_7_HSIOM
	#define ioss_0_port_11_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define SMIF_SPI_CLOCK_HSIOM ioss_0_port_11_pin_7_HSIOM
#define SMIF_SPI_CLOCK_IRQ ioss_interrupts_gpio_11_IRQn
#define LED_RED_PORT GPIO_PRT13
#define LED_RED_PIN 7U
#define LED_RED_NUM 7U
#define LED_RED_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define LED_RED_INIT_DRIVESTATE 1
#ifndef ioss_0_port_13_pin_7_HSIOM
	#define ioss_0_port_13_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define LED_RED_HSIOM ioss_0_port_13_pin_7_HSIOM
#define LED_RED_IRQ ioss_interrupts_gpio_13_IRQn
#define UART_TX_PORT GPIO_PRT5
#define UART_TX_PIN 1U
#define UART_TX_NUM 1U
#define UART_TX_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define UART_TX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_5_pin_1_HSIOM
	#define ioss_0_port_5_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define UART_TX_HSIOM ioss_0_port_5_pin_1_HSIOM
#define UART_TX_IRQ ioss_interrupts_gpio_5_IRQn
#define ioss_0_port_6_pin_4_PORT GPIO_PRT6
#define ioss_0_port_6_pin_4_PIN 4U
#define ioss_0_port_6_pin_4_NUM 4U
#define ioss_0_port_6_pin_4_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define ioss_0_port_6_pin_4_INIT_DRIVESTATE 1
#ifndef ioss_0_port_6_pin_4_HSIOM
	#define ioss_0_port_6_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_6_pin_4_IRQ ioss_interrupts_gpio_6_IRQn
#define ioss_0_port_6_pin_6_PORT GPIO_PRT6
#define ioss_0_port_6_pin_6_PIN 6U
#define ioss_0_port_6_pin_6_NUM 6U
#define ioss_0_port_6_pin_6_DRIVEMODE CY_GPIO_DM_PULLUP
#define ioss_0_port_6_pin_6_INIT_DRIVESTATE 1
#ifndef ioss_0_port_6_pin_6_HSIOM
	#define ioss_0_port_6_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_6_pin_6_IRQ ioss_interrupts_gpio_6_IRQn
#define ioss_0_port_6_pin_7_PORT GPIO_PRT6
#define ioss_0_port_6_pin_7_PIN 7U
#define ioss_0_port_6_pin_7_NUM 7U
#define ioss_0_port_6_pin_7_DRIVEMODE CY_GPIO_DM_PULLDOWN
#define ioss_0_port_6_pin_7_INIT_DRIVESTATE 1
#ifndef ioss_0_port_6_pin_7_HSIOM
	#define ioss_0_port_6_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_6_pin_7_IRQ ioss_interrupts_gpio_6_IRQn

extern const cy_stc_gpio_pin_config_t PIN_SW2_config;
extern const cy_stc_gpio_pin_config_t SMIF_SS0_config;
extern const cy_stc_gpio_pin_config_t SMIF_DATA3_config;
extern const cy_stc_gpio_pin_config_t SMIF_DATA2_config;
extern const cy_stc_gpio_pin_config_t SMIF_DATA1_config;
extern const cy_stc_gpio_pin_config_t SMIF_DATA0_config;
extern const cy_stc_gpio_pin_config_t SMIF_SPI_CLOCK_config;
extern const cy_stc_gpio_pin_config_t LED_RED_config;
extern const cy_stc_gpio_pin_config_t UART_TX_config;
extern const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_4_config;
extern const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_6_config;
extern const cy_stc_gpio_pin_config_t ioss_0_port_6_pin_7_config;

void init_cycfg_pins(void);

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_PINS_H */
