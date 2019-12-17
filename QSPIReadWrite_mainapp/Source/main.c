/***************************************************************************//**
* \file main.c
* \version 1.0
*
* \brief
* Objective:
*    Demonstrates read/write operation to external memory by SMIF(QSPI) driver
*       in quad SPI mode. The example also check the integrity of the read data
*       against written data.
*
* Compatible Kits:
*    CY8CKIT-062-BLE
*    CY8CKIT-062-WIFI-BT
*
* Migration to CY8CPROTO-062-4343W kit (ModusToolbox IDE):
*   1. Create this project targeting the CY8CPROTO-062-4343W kit.
*   2. Open design.modus file and replicate P0[3] configuration on P13[7]. Give
*      this pin the alias "LED_RED". Disable P0[3].
*   3. Build and Program
* 
* Migration to CY8CPROTO-062-4343W kit (command line make):
*   1. Launch the Device Configurator tool from 
*      ModusToolbox_1.0\tools\device-configurator-1.0\
*   2. In Device Configurator, open design.modus file  
*      (ModusToolbox_1.0\libraries\psoc6sw-1.0\examples\BlinkyLED\design.modus) 
*      and replicate P0[3] configuration on P13[7]. 
*      Give this pin the alias "LED_RED". Disable P0[3].
*   3. Perform "make clean"
*   4. Build and Program the device with "make DEVICE=CY8C624ABZI-D44 program"
*      Note that depending on the method used to program the device, you may 
*      need to manually reset it by pressing the SW1 RESET button on the kit.
*   4. Observe the red blinking LED.
*   5. To switch back to CY8CKIT-062-BLE or CY8CKIT-062-WIFI-BT, 
*      perform steps 1 through 3 to reconfigure the "LED_RED" to P0[3]. 
*      Then use "make program".
*
*   Instructions:
*       1. Connect CY8CKIT-062-BLE to a USB port on your PC.
*       2. Open a serial port communication program, such as Tera Term and 
*          select the corresponding COM port. Configure the terminal to match 
*          UART: 115200 baud rate, 8N1, and Flow control – None. These settings 
*          must match the configuration of the SCB UART driver in the project.
*       3. Build and program the application into the kit.
*       4. Observe the red LED is blinking to determine that the SMIF operation 
*          finished successfully and passed read data integrity check.
*       5. Make sure that debug messages display in the terminal window as 
*          expected.
*   NOTE If code example run for first time QE is not set. User need to allow
*   sending Quad Enable command manually by pressing SW2 button to ensure that
*   power is stable. Debug message will appear in terminal window. 
*
********************************************************************************
* \copyright
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
*******************************************************************************/


#include <stdio.h>
#include <string.h>

#include "cy_pdl.h"
#include "cycfg.h"
#include "cycfg_qspi_memslot.h"

/***************************************************************************
* Global constants
***************************************************************************/

#define SMIF_PRIORITY       (1u)      /* SMIF interrupt priority */
#define TIMEOUT_1_MS        (1000ul)  /* 1 ms timeout for all blocking functions */
#define PACKET_SIZE1        (512)    /* The memory Read/Write packet */
#define ADDRESS_SIZE        (3u)      /* Memory address size */
#define SUCCESS             (0u)
#define CHECK_INDIVIDUAL_WRITE_SPEED     (1u)
/***************************************************************************
* Global variables
***************************************************************************/
cy_stc_scb_uart_context_t UART_context;
cy_stc_smif_context_t SMIF_context;

uint32_t w1Time, w2Time, w3Time, w4Time, w5Time;

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
*
* This function processes unrecoverable errors
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts */
    __disable_irq();
    while(1u) {}
}


/*******************************************************************************
* Function Name: SMIF_Interrupt_User
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void SMIF_Interrupt_User(void)
{
    Cy_SMIF_Interrupt(SMIF_HW, &SMIF_context);
}


/*******************************************************************************
* Function Name: InitBuffers
****************************************************************************//**
*
* This function initializes the transfer buffers.
*
* \param txBuffer - The buffer for Write data.
*
* \param rxBuffer - The buffer for Read data.
*
*******************************************************************************/
void InitBuffers(uint8_t txBuffer[], uint8_t rxBuffer[], uint32_t bufferSize)
{
    for(uint32_t index=0; index<bufferSize; index++)
    {
        txBuffer[index] = (uint8_t) (index & 0xFF);
        rxBuffer[index] = 0;
    }
}


/*******************************************************************************
* Function Name: PrintArray
****************************************************************************//**
*
* This function prints the content of the RX buffer to the UART console.
*
* \param msg - message print before array output
*
* \param  rxBuffer - The buffer to the console output.
*
* \param  size - The size of the buffer to the console output.
*
*******************************************************************************/
void PrintArray(char * msg, uint8_t * buff, uint32_t size)
{
    printf("%s", msg);
    for(uint32_t index=0; index<size; index++)
    {
        printf("0x%02X ", (unsigned int) buff[index]);
    }
    printf("\r\n=======================\r\n");
}


/*******************************************************************************
* Function Name: CheckStatus
****************************************************************************//**
*
* Check if status is SUCCES and call handle error function
*
*******************************************************************************/
void CheckStatus(char * msg, uint32_t status)
{
    if(SUCCESS != status)
    {
        printf("%s", msg);
        handle_error();
    }
}


/*******************************************************************************
* Function Name: EnableQuadMode
****************************************************************************//**
*
* This function check if Quad mode is enabled and enable it is it is not enabled.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param memConfig
* Memory device configuration
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
void EnableQuadMode(SMIF_Type *baseaddr, cy_stc_smif_mem_config_t *memConfig,
		            cy_stc_smif_context_t const *smifContext)
{
	cy_en_smif_status_t status;
	uint8_t readStatus = 0;
	uint32_t statusCmd = memConfig->deviceCfg->readStsRegQeCmd->command;
	uint8_t maskQE = (uint8_t) memConfig->deviceCfg->stsRegQuadEnableMask;

	status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, memConfig, &readStatus, statusCmd,
							   	        smifContext);
	CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_QuadEnable failed\r\n", status);

	if(maskQE != (readStatus & maskQE))
	{
		/* Check user confirmation to do QE*/
		printf("\r\nQuad is NOT enabled. Pleas press button to continue\r\n");
		bool btnPressed = false;
		while (!btnPressed)
	    {   /* 50 ms delay for button press check*/
			Cy_SysLib_Delay(50u);
			if (Cy_GPIO_Read(PIN_SW2_PORT, PIN_SW2_PIN) == 0u)
			{
				/* 50 ms delay for button debounce on button press */
				Cy_SysLib_Delay(50u);
			    if (Cy_GPIO_Read(PIN_SW2_PORT, PIN_SW2_PIN) == 0u)
			    {
			    	while (Cy_GPIO_Read(PIN_SW2_PORT, PIN_SW2_PIN) == 0u)
			    	{   /* 50 ms delay for button debounce on button release */
			    		Cy_SysLib_Delay(50u);
			    	}
			    	btnPressed = true;
			    }

			}
	    }

		status = Cy_SMIF_Memslot_QuadEnable(baseaddr, memConfig, smifContext);
		CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_QuadEnable failed\r\n", status);

		while(Cy_SMIF_Memslot_IsBusy(baseaddr, memConfig, smifContext))
		{
			/* Wait until the QE operation is completed */
		}
	}
	else
	{
		printf("\r\nQuad is enabled. Enabling operation skipped\r\n");
	}
}


/*******************************************************************************
* Function Name: ReadMemory
****************************************************************************//**
*
* This function reads data from the external memory in the quad mode.
* The function sends the Quad I/O Read: 0xEB command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param rxBuffer
* The buffer for read data.
*
* \param rxSize
* The size of data to read.
*
* \param address
* The address to read data from.
*
*******************************************************************************/
void ReadMemory(SMIF_Type *baseaddr,
                            cy_stc_smif_context_t *smifContext,
                            uint8_t rxBuffer[],
                            uint32_t rxSize,
                            uint8_t *address)
{
    cy_en_smif_status_t status;
    uint8_t rxBuffer_reg;
    cy_stc_smif_mem_device_cfg_t *device = smifMemConfigs[0]->deviceCfg;
    cy_stc_smif_mem_cmd_t *cmdreadStsRegQe = device->readStsRegQeCmd;

    /* Read data from the external memory configuration register */
    status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, smifMemConfigs[0], &rxBuffer_reg, (uint8_t)cmdreadStsRegQe->command , smifContext);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdReadSts failed\r\n", status);

//    printf("Received Data: 0x%X\r\n", (unsigned int) rxBuffer_reg);
//    printf("\r\nQuad I/O Read (QIOR 0x%0X) \r\n", 0x38);

    /* The 4 Page program command */
    status = Cy_SMIF_Memslot_CmdRead(baseaddr, smifMemConfigs[0], address, rxBuffer, rxSize, NULL, &SMIF_context);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdRead failed\r\n",status);

    while(Cy_SMIF_BusyCheck(baseaddr))
    {
        /* Wait until the SMIF IP operation is completed. */
    }

    /* Send received data to the console */
//    PrintArray("Received Data: ",rxBuffer, rxSize);
}


/*******************************************************************************
* Function Name: WriteMemory
********************************************************************************
*
* This function writes data to the external memory in the quad mode.
* The function sends the Quad Page Program: 0x38 command to the external memory.
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param smifContext
* The internal SMIF context data.
*
* \param txBuffer
* Data to write in the external memory.
*
* \param txSize
* The size of data.
*
* \param address
* The address to write data to.
*
*******************************************************************************/
void WriteMemory(SMIF_Type *baseaddr,
                    cy_stc_smif_context_t *smifContext,
                    uint8_t txBuffer[],
                    uint32_t txSize,
                    uint8_t *address)
{
    cy_en_smif_status_t status;
    uint8_t rxBuffer_reg;
    cy_stc_smif_mem_device_cfg_t *device = smifMemConfigs[0]->deviceCfg;
    cy_stc_smif_mem_cmd_t *cmdreadStsRegQe = device->readStsRegQeCmd;
    cy_stc_smif_mem_cmd_t *cmdreadStsRegWip = device->readStsRegWipCmd;

    #if (CHECK_INDIVIDUAL_WRITE_SPEED == 1u)
    Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
    Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
    Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
    /* Read data from the external memory configuration register */
    status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, smifMemConfigs[0], &rxBuffer_reg, (uint8_t)cmdreadStsRegQe->command , smifContext);
    w1Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdReadSts failed\r\n", status);

//    printf("Received Data: 0x%X\r\n", (unsigned int) rxBuffer_reg);

    Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
    Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
    Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
    /* Send Write Enable to external memory */
    status = Cy_SMIF_Memslot_CmdWriteEnable(baseaddr, smifMemConfigs[0], &SMIF_context);
    w2Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdWriteEnable failed\r\n", status);

//    printf("\r\nQuad Page Program (QPP 0x%0X) \r\n", 0x38);

    Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
    Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
    Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
    /* Quad Page Program command */
    status = Cy_SMIF_Memslot_CmdProgram(SMIF_HW, smifMemConfigs[0], address, txBuffer, txSize, NULL, &SMIF_context);
    w3Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdProgram failed\r\n", status);

//    PrintArray("Written Data: ", txBuffer, txSize);

    Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
    Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
    Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
    while(Cy_SMIF_Memslot_IsBusy(SMIF_HW, (cy_stc_smif_mem_config_t*)smifMemConfigs[0], &SMIF_context))
    {
        /* Wait until the Erase operation is completed */
    }
    w4Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);

    Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
    Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
    Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
    /* Read data from the external memory status register */
    status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, smifMemConfigs[0], &rxBuffer_reg,
                             (uint8_t)cmdreadStsRegWip->command , smifContext);
    w5Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
    CheckStatus("\r\n\r\nSMIF ReadStatusReg failed\r\n", status);
    #else
    /* Read data from the external memory configuration register */
    status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, smifMemConfigs[0], &rxBuffer_reg, (uint8_t)cmdreadStsRegQe->command , smifContext);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdReadSts failed\r\n", status);

//    printf("Received Data: 0x%X\r\n", (unsigned int) rxBuffer_reg);

    /* Send Write Enable to external memory */
    status = Cy_SMIF_Memslot_CmdWriteEnable(baseaddr, smifMemConfigs[0], &SMIF_context);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdWriteEnable failed\r\n", status);

//    printf("\r\nQuad Page Program (QPP 0x%0X) \r\n", 0x38);

    /* Quad Page Program command */
    status = Cy_SMIF_Memslot_CmdProgram(SMIF_HW, smifMemConfigs[0], address, txBuffer, txSize, NULL, &SMIF_context);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdProgram failed\r\n", status);

//    PrintArray("Written Data: ", txBuffer, txSize);

    while(Cy_SMIF_Memslot_IsBusy(SMIF_HW, (cy_stc_smif_mem_config_t*)smifMemConfigs[0], &SMIF_context))
    {
        /* Wait until the Erase operation is completed */
    }

    /* Read data from the external memory status register */
    status = Cy_SMIF_Memslot_CmdReadSts(baseaddr, smifMemConfigs[0], &rxBuffer_reg,
                             (uint8_t)cmdreadStsRegWip->command , smifContext);
    CheckStatus("\r\n\r\nSMIF ReadStatusReg failed\r\n", status);
    #endif
//    printf("Received Data: 0x%X\r\n", (unsigned int) rxBuffer_reg);
}


/*******************************************************************************
* Function Name: EraseMemory
********************************************************************************
*
* Erase block of external memory
*
* \param baseaddr
* Holds the base address of the SMIF block registers.
*
* \param memConfig configuration of external memory
*
* \param address
* The address to write data to.
*
* \param smifContext
* The internal SMIF context data.
*
*******************************************************************************/
void EraseMemory(SMIF_Type *baseaddr, cy_stc_smif_mem_config_t *memConfig,
                 uint8_t *address,
                 cy_stc_smif_context_t const *smifContext)
{
    cy_en_smif_status_t status;
    status = Cy_SMIF_Memslot_CmdWriteEnable(baseaddr, memConfig, smifContext);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdWriteEnable failed\r\n", status);

    status = Cy_SMIF_Memslot_CmdSectorErase(baseaddr, memConfig, address, smifContext);
    CheckStatus("\r\n\r\nSMIF Cy_SMIF_Memslot_CmdSectorErase failed\r\n", status);

    /* Wait until the memory is erased */
    while(Cy_SMIF_Memslot_IsBusy(baseaddr, memConfig, smifContext))
    {
        /* Wait until the Erase operation is completed */
    }
}

/*******************************************************************************
* Function Name: TxRxEqualCheck
****************************************************************************//**
*
* This function checks if the transmitted and received arrays are equal
*
* \param txBuffer - The buffer for Write data.
*
* \param rxBuffer - The buffer for Read data.
*
*******************************************************************************/
bool TxRxEqualCheck(uint8_t txBuffer[],uint8_t rxBuffer[], int PACKET_SIZE)
{
    return(0U == memcmp(txBuffer, rxBuffer, PACKET_SIZE));
}


/***************************************************************************
* Function Name: main.c
***************************************************************************/
int main(void)
{
    /* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();

    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    Cy_SCB_UART_Enable(UART_HW);

    /* enable interrupts, and the CM4 */
    __enable_irq();

    uint32_t count = 0;
    uint32_t eTime, r1Time, r2Time, wTime;

    printf("\r\n\r\nUART initialization complete\r\n");
    printf("\r\n\r\nSMIF code example started\r\n");

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(Timer_HW, Timer_NUM, &Timer_config))
    {
    	/* Handle possible errors */
    	printf("Timer Init Error!!\r\n");
    	handle_error();
    }
    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(Timer_HW, Timer_NUM);
    Cy_TCPWM_TriggerStart(Timer_HW, Timer_MASK);
    Cy_SysLib_Delay(1000);
//    count =

    /* Configure SMIF interrupt */
    cy_stc_sysint_t smifIntConfig =
    {
        .intrSrc = smif_interrupt_IRQn,     /* SMIF interrupt */
        .intrPriority = SMIF_PRIORITY       /* SMIF interrupt priority */
    };

    /* SMIF interrupt initialization status */
    cy_en_sysint_status_t intr_init_status;
    intr_init_status = Cy_SysInt_Init(&smifIntConfig, SMIF_Interrupt_User);
    CheckStatus("\r\n\r\nSMIF interrupt initialization failed\r\n", (uint32_t)intr_init_status);

    printf("SMIF interrupt initialization complete\r\n");

    /* Initialize SMIF */
    cy_en_smif_status_t smif_status;
    smif_status = Cy_SMIF_Init(SMIF_HW, &SMIF_config, TIMEOUT_1_MS, &SMIF_context);
    CheckStatus("\r\n\r\nSMIF initialization failed\r\n", smif_status);

    /* Configures data select */
    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_Enable(SMIF_HW, &SMIF_context);
    printf("SMIF initialization complete\r\n");

    /* Initialize the transfer buffers */
    uint8_t txBuffer[PACKET_SIZE1] = {0};
    uint8_t rxBuffer[PACKET_SIZE1] = {0};
    InitBuffers(txBuffer, rxBuffer, PACKET_SIZE1);

    /* Enable the SMIF interrupt */
    NVIC_EnableIRQ(smif_interrupt_IRQn);
    printf("=========================================================\r\n");
    printf("\r\nSMIF operation in quad mode\r\n");

    uint8_t extMemAddress[ADDRESS_SIZE] = {0x00, 0x00, 0x00};
    /* Set QE */
    EnableQuadMode(SMIF_HW, (cy_stc_smif_mem_config_t*)smifMemConfigs[0], &SMIF_context);

	#if 0
		printf("Erase Time: %d\r\n", eTime);
		#if (CHECK_INDIVIDUAL_WRITE_SPEED == 0u)
		printf("i\t\teTime\tr1Time\twTime\tr2Time\r\n");
		#else
		printf("i\tj\teTime\tr1Time\tw1Time\tw2Time\tw3Time\tw4Time\tw5Time\tr2Time\r\n");
		#endif

		for (int i = 1; i <= 512; i ++)
		{
			uint32_t PACKET_SIZE = i;
			Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
			Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
			Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
			EraseMemory(SMIF_HW, (cy_stc_smif_mem_config_t*)smifMemConfigs[0], extMemAddress, &SMIF_context);
			eTime = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);

			for (int j = 0; j < 3; j ++)
			{

				/*
				 * Read the PACKET_SIZE bytes in the single mode from the address extMemAddress
				 * and send the packet to the console.
				 * Should be 0xFF, the memory was just erased
				 */

				Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
				Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
				Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
				ReadMemory(SMIF_HW, &SMIF_context, rxBuffer, PACKET_SIZE, extMemAddress);
				r1Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
	//    		PrintArray("Received Data: ",rxBuffer, PACKET_SIZE);

				/* Write the PACKET_SIZE bytes in the single mode to the address extMemAddress
				 * and send the packet to the console.
				 */
				InitBuffers(txBuffer, rxBuffer, PACKET_SIZE1);
				#if (CHECK_INDIVIDUAL_WRITE_SPEED == 0u)
				Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
				Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
				Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
				WriteMemory(SMIF_HW, &SMIF_context, txBuffer, PACKET_SIZE, extMemAddress);
				wTime = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
				#else
				w1Time = 0;
				w2Time = 0;
				w3Time = 0;
				w4Time = 0;
				w5Time = 0;
				WriteMemory(SMIF_HW, &SMIF_context, txBuffer, PACKET_SIZE, extMemAddress);
				#endif

				/* Read the PACKET_SIZE bytes in the single mode from the address extMemAddress
				 * and send the packet to the console.
				 */
				/* The value should be txBuffer */
				Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
				Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
				Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
				ReadMemory(SMIF_HW, &SMIF_context, rxBuffer, PACKET_SIZE, extMemAddress);
				r2Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);

				/* Check if the transmitted and received arrays are equal */
				if (!TxRxEqualCheck(txBuffer, rxBuffer, PACKET_SIZE))
				{
					printf("\r\nRead data does not match with written data in quad mode\r\n");
					printf("\r\nSMIF operation is failed in quad mode\r\n");

					Cy_GPIO_Clr(LED_RED_PORT, LED_RED_PIN); /* Turn on the LED */
					while(1);
				}

				#if (CHECK_INDIVIDUAL_WRITE_SPEED == 0u)
				printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", i, j, eTime, r1Time, wTime, r2Time);
		#else
				printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", i, j, eTime, r1Time, w1Time, w2Time, w3Time, w4Time, w5Time, r2Time);
		#endif
				Cy_SysLib_Delay(5);
			}
			printf("\r\n");
			Cy_GPIO_Inv(LED_RED_PORT, LED_RED_PIN); /* toggle the LED */
		}
		#else
		printf("Erase Time: %d\r\n", eTime);
		#if (CHECK_INDIVIDUAL_WRITE_SPEED == 0u)
		printf("i\t\teTime\tr1Time\twTime\tr2Time\r\n");
		#else
		printf("i\tj\teTime\tr1Time\tw1Time\tw2Time\tw3Time\tw4Time\tw5Time\tr2Time\r\n");
		#endif

		for (int i = 6; i < 10; i ++)
		{
			uint32_t PACKET_SIZE = 1 << i;
			Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
			Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
			Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
			EraseMemory(SMIF_HW, (cy_stc_smif_mem_config_t*)smifMemConfigs[0], extMemAddress, &SMIF_context);
			eTime = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);

			extMemAddress[0] = 0x00;
			extMemAddress[1] = 0x00;
			extMemAddress[2] = 0x00;

			for (int j = 0; j < 3; j ++)
			{
				uint16_t k = PACKET_SIZE * j;
				extMemAddress[2] = (uint8_t)(k & 0xFF);
				extMemAddress[1] = (uint8_t)((k & 0xFF00) >> 8);

				/*
				 * Read the PACKET_SIZE bytes in the single mode from the address extMemAddress
				 * and send the packet to the console.
				 * Should be 0xFF, the memory was just erased
				 */

				Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
				Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
				Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
				ReadMemory(SMIF_HW, &SMIF_context, rxBuffer, PACKET_SIZE, extMemAddress);
				r1Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
				//    		PrintArray("Received Data: ",rxBuffer, PACKET_SIZE);

				/* Write the PACKET_SIZE bytes in the single mode to the address extMemAddress
				 * and send the packet to the console.
				 */
				InitBuffers(txBuffer, rxBuffer, PACKET_SIZE1);
#if (CHECK_INDIVIDUAL_WRITE_SPEED == 0u)
				Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
				Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
				Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
				WriteMemory(SMIF_HW, &SMIF_context, txBuffer, PACKET_SIZE, extMemAddress);
				wTime = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
#else
				w1Time = 0;
				w2Time = 0;
				w3Time = 0;
				w4Time = 0;
				w5Time = 0;
				WriteMemory(SMIF_HW, &SMIF_context, txBuffer, PACKET_SIZE, extMemAddress);
#endif

				/* Read the PACKET_SIZE bytes in the single mode from the address extMemAddress
				 * and send the packet to the console.
				 */
				/* The value should be txBuffer */
				Cy_TCPWM_TriggerStopOrKill(Timer_HW, Timer_MASK);
				Cy_TCPWM_Counter_SetCompare0(Timer_HW, Timer_NUM, 0U);
				Cy_TCPWM_TriggerReloadOrIndex(Timer_HW, Timer_MASK);
				ReadMemory(SMIF_HW, &SMIF_context, rxBuffer, PACKET_SIZE, extMemAddress);
				r2Time = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);

				/* Check if the transmitted and received arrays are equal */
				if (!TxRxEqualCheck(txBuffer, rxBuffer, PACKET_SIZE))
				{
					printf("\r\nRead data does not match with written data in quad mode\r\n");
					printf("\r\nSMIF operation is failed in quad mode\r\n");

					Cy_GPIO_Clr(LED_RED_PORT, LED_RED_PIN); /* Turn on the LED */
					while(1);
				}

#if (CHECK_INDIVIDUAL_WRITE_SPEED == 0u)
				printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", i, j, eTime, r1Time, wTime, r2Time);
#else
				printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", i, j, eTime, r1Time, w1Time, w2Time, w3Time, w4Time, w5Time, r2Time);
#endif
				Cy_SysLib_Delay(5);
			}
			printf("\r\n");
			Cy_GPIO_Inv(LED_RED_PORT, LED_RED_PIN); /* toggle the LED */
		}
#endif


    for (;;)
    {
        Cy_GPIO_Inv(LED_RED_PORT, LED_RED_PIN); /* toggle the LED */
        Cy_SysLib_Delay(1023/*msec*/);
    }
}


