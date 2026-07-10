/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4: CAPSENSE SmartSense
 * buttons slider Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * (c) 2023-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 *******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"


/*******************************************************************************
 * Macros
 *******************************************************************************/
#define CAPSENSE_INTR_PRIORITY           (3u)
#define CY_ASSERT_FAILED                 (0u)
#if defined COMPONENT_PSOC4100SP
#define SLD_NUM_SENSORS                  (6)
#endif
#if defined COMPONENT_PSOC4000S
#define SLD_NUM_SENSORS                  (5)
#endif
#if defined COMPONENT_PSOC4000S || defined COMPONENT_PSOC4100SP
/*Slider maximum position*/
#define RESOLUTION                       (100)
/*Step size for LED control based on touch position of the slider*/
#define STEP_SIZE                        (RESOLUTION/SLD_NUM_SENSORS)
#endif

/* LED state */
#define LED_ON                           (0u)
#define LED_OFF                          (1u)

/* EZI2C interrupt priority must be higher than CAPSENSE interrupt */
#define EZI2C_INTR_PRIORITY              (2u)


/*******************************************************************************
 * Global Definitions
 *******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;


/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void initialize_capsense(void);
static void capsense_isr(void);
static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);
static void capsense_led(void);
#if defined COMPONENT_PSOC4000S || defined COMPONENT_PSOC4100SP
void smart_io_start(void);
#endif


/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE
 *  - initialize tuner communication
 *  - scan touch input continuously
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    /* Initialize CAPSENSE */
    initialize_capsense();

#if defined COMPONENT_PSOC4000S || defined COMPONENT_PSOC4100SP
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(CYBSP_PWM1_HW, CYBSP_PWM1_NUM, &CYBSP_PWM1_config))
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(CYBSP_PWM2_HW, CYBSP_PWM2_NUM, &CYBSP_PWM2_config))
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM1_HW, CYBSP_PWM1_NUM);
    Cy_TCPWM_PWM_Enable(CYBSP_PWM2_HW, CYBSP_PWM2_NUM);

    /* Then start the PWM */
    Cy_TCPWM_TriggerStart(CYBSP_PWM1_HW, CYBSP_PWM1_MASK);
    Cy_TCPWM_TriggerStart(CYBSP_PWM2_HW, CYBSP_PWM2_MASK);

    smart_io_start();
#endif

    /* Start the first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            capsense_led();

            /* Establishes synchronized communication with the CAPSENSE Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Start the next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        }
    }
}


/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CAPSENSE and configures the CAPSENSE
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CAPSENSE interrupt configuration */
    const cy_stc_sysint_t capsense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CAPSENSE interrupt */
        Cy_SysInt_Init(&capsense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(capsense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_interrupt_config.intrSrc);

        /* Initialize the CAPSENSE firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CAPSENSE sensors are tuned
         * as per procedure given in the README.md file.
         */
    }
}


/*******************************************************************************
 * Function Name: capsense_isr
 ********************************************************************************
 * Summary:
 *  Function for handling interrupts from CAPSENSE block.
 *
 *******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}


/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CAPSENSE Tuner tool.
 *
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
            .intrSrc = CYBSP_EZI2C_IRQ,
            .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CAPSENSE data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
            &ezi2c_context);

    /* Enables the SCB block for the EZI2C operation. */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

    /* EZI2C initialization failed */
    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}


/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 *  Function for handling interrupts from EZI2C block.
 *
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

/*******************************************************************************
 * Function Name: capsense_led
 ********************************************************************************
 * Summary:
 *  LED ON/OFF based on the CAPSENSE buttons status and CAPSENSE linear slider
 *  touch position
 *
 *******************************************************************************/
static void capsense_led(void)
{
#if defined COMPONENT_PSOC4000S || defined COMPONENT_PSOC4100SP
    /* LED ON/OFF based on the CAPSENSE linear slider position */
    if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context))
    {
        /* CAPSENSE Slider info */
        cy_stc_capsense_touch_t *slider_touch_info;
        uint8_t slider_touch_status;
        uint16_t slider_pos = 0;

        /* Get slider status */
        slider_touch_info = Cy_CapSense_GetTouchInfo(
            CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
        slider_touch_status = slider_touch_info->numPosition;

        if (slider_touch_status != 0)
        {
            slider_pos = slider_touch_info->ptrPosition->x;
        }

        if(slider_pos > 0 || slider_pos == 0)
        {
            Cy_GPIO_Write(CYBSP_LED_SLD0_PORT, CYBSP_LED_SLD0_NUM, LED_ON);
        }

        if(slider_pos > (1*STEP_SIZE))
        {
            Cy_GPIO_Write(CYBSP_LED_SLD1_PORT, CYBSP_LED_SLD1_NUM, LED_ON);
        }
        else
        {
            Cy_GPIO_Write(CYBSP_LED_SLD1_PORT, CYBSP_LED_SLD1_NUM, LED_OFF);
        }
        if(slider_pos > (2*STEP_SIZE))
        {
            Cy_GPIO_Write(CYBSP_LED_SLD2_PORT, CYBSP_LED_SLD2_NUM, LED_ON);
        }
        else
        {
            Cy_GPIO_Write(CYBSP_LED_SLD2_PORT, CYBSP_LED_SLD2_NUM, LED_OFF);
        }
        if(slider_pos > (3*STEP_SIZE))
        {
            Cy_GPIO_Write(CYBSP_LED_SLD3_PORT, CYBSP_LED_SLD3_NUM, LED_ON);
        }
        else
        {
            Cy_GPIO_Write(CYBSP_LED_SLD3_PORT, CYBSP_LED_SLD3_NUM, LED_OFF);
        }
        if(slider_pos > (4*STEP_SIZE))
        {
            Cy_GPIO_Write(CYBSP_LED_SLD4_PORT, CYBSP_LED_SLD4_NUM, LED_ON);
        }
        else
        {
            Cy_GPIO_Write(CYBSP_LED_SLD4_PORT, CYBSP_LED_SLD4_NUM, LED_OFF);
        }
#if defined COMPONENT_PSOC4100SP
        if(slider_pos > (5*STEP_SIZE))
        {
            Cy_GPIO_Write(CYBSP_LED_SLD5_PORT, CYBSP_LED_SLD5_NUM, LED_ON);
        }
        else
        {
            Cy_GPIO_Write(CYBSP_LED_SLD5_PORT, CYBSP_LED_SLD5_NUM, LED_OFF);
        }
#endif
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_SLD0_PORT, CYBSP_LED_SLD0_NUM, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD1_PORT, CYBSP_LED_SLD1_NUM, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD2_PORT, CYBSP_LED_SLD2_NUM, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD3_PORT, CYBSP_LED_SLD3_NUM, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD4_PORT, CYBSP_LED_SLD4_NUM, LED_OFF);
#if defined COMPONENT_PSOC4100SP
        Cy_GPIO_Write(CYBSP_LED_SLD5_PORT, CYBSP_LED_SLD5_NUM, LED_OFF);
#endif
    }

    /* LED ON/OFF based on the CAPSENSE buttons status */
    if(0 != Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON0_WDGT_ID, CY_CAPSENSE_BUTTON0_SNS0_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_NUM, LED_ON);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_NUM, LED_OFF);
    }

    if(0 != Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON1_WDGT_ID, CY_CAPSENSE_BUTTON1_SNS0_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM, LED_ON);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM, LED_OFF);
    }

    if(0 != Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON2_WDGT_ID, CY_CAPSENSE_BUTTON2_SNS0_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_LED_BTN2_PORT, CYBSP_LED_BTN2_NUM, LED_ON);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_BTN2_PORT, CYBSP_LED_BTN2_NUM, LED_OFF);
    }

#else
    /* LED ON/OFF based on the CAPSENSE linear slider position */
    if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_LED_RGB_BLUE_PORT, CYBSP_LED_RGB_BLUE_NUM, LED_ON);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_RGB_BLUE_PORT, CYBSP_LED_RGB_BLUE_NUM, LED_OFF);
    }
    /* LED ON/OFF based on the CAPSENSE buttons status */
    if(0 != Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON0_WDGT_ID, CY_CAPSENSE_BUTTON0_SNS0_ID, &cy_capsense_context))
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_ON);
    }
    else
    {
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_OFF);
    }
#endif
}

#if defined COMPONENT_PSOC4000S || defined COMPONENT_PSOC4100SP
/*******************************************************************************
 * Function Name: smart_io_start
 ********************************************************************************
 * Summary:
 *  This function initializes and enable SMART_IO block.
 *
 *******************************************************************************/
void smart_io_start(void)
{
#if defined COMPONENT_PSOC4000S
    /* Configure the SMART_IO block */
    if (CY_SMARTIO_SUCCESS != Cy_SmartIO_Init(PRGIO_PRT0, &SMART_IO_config))
    {
        CY_ASSERT(0);
    }

    /* Enable SMART_IO block */
    Cy_SmartIO_Enable(PRGIO_PRT0);
#else
    /* Configure the SMART_IO block */
    if (CY_SMARTIO_SUCCESS != Cy_SmartIO_Init(PRGIO_PRT1, &SMART_IO_config))
    {
        CY_ASSERT(0);
    }

    /* Enable SMART_IO block */
    Cy_SmartIO_Enable(PRGIO_PRT1);
#endif
}
#endif

/* [] END OF FILE */
