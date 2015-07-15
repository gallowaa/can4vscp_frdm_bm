/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//#ifdef FSL_IRQ

#include "main.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern volatile bool pitIsrFlag[2];
extern volatile uint32_t pitCounter;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief System default IRQ handler defined in startup code.
 */
#if FSL_FEATURE_PIT_HAS_SHARED_IRQ_HANDLER
void PIT_IRQHandler(void)
{
    uint32_t i;
    for(i=0; i < 2; i++)  // We only support two channels for pit example
    {
        if (PIT_HAL_IsIntPending(g_pitBase[0], i))
        {
            /* Clear interrupt flag.*/
            PIT_HAL_ClearIntFlag(g_pitBase[0], i);
            pitIsrFlag[i] = true;
        }
    }
}
#else

#if (FSL_FEATURE_PIT_TIMER_COUNT > 0U)
void PIT0_IRQHandler(void)
{
	/* Clear interrupt flag.*/
	PIT_HAL_ClearIntFlag(g_pitBase[0], 0U);
	CLOCK_PIN_TOGGLE; 	/* Used to verify 1ms clock. Period/2 should = 1ms. */

	// critical vscp
	vscp_timer++;
	vscp_configtimer++;
	pitCounter++;
	// measurement_clock++; /* optional */

#ifdef TIMER_HAS_LONG_PERIOD
	if(VSCP_LED_BLINK1 == vscp_initledfunc)
		STATUS_LED_TOGGLE;
#else

	vscp_statuscnt++;

	if( (vscp_statuscnt > 100) && (vscp_initledfunc == VSCP_LED_BLINK1) ){
		STATUS_LED_TOGGLE; // blink the vscp status led
		vscp_statuscnt = 0;
	}
#endif

	else if (VSCP_LED_ON == vscp_initledfunc){
		STATUS_LED_ON;
	}
}
#endif

#if (FSL_FEATURE_PIT_TIMER_COUNT > 1U)
void PIT1_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_HAL_ClearIntFlag(g_pitBase[0], 1U);
    pitIsrFlag[1] = true;
}
#endif
#endif /* FSL_FEATURE_PIT_HAS_SHARED_IRQ_HANDLER */

/******************************************************************************
 * 									fsl_lptmr_irq.c
 *****************************************************************************/
/* LPTMR IRQ handler that covers the same name's APIs in startup code */
void LPTMR0_IRQHandler(void)
{
    LPTMR_DRV_IRQHandler(0U);
}


/*******************************************************************************
 *									fsl_flexcan_irq.c
 ******************************************************************************/
#if (CAN_INSTANCE_COUNT > 0U)
/* Implementation of CAN0 handler named in startup code. */
void CAN0_ORed_Message_buffer_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Bus_Off_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}

/* Implementation of CAN0 handler named in startup code. */
void CAN0_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(0);
}
#endif

#if (CAN_INSTANCE_COUNT > 1U)
/* Implementation of CAN1 handler named in startup code. */
void CAN1_ORed_Message_buffer_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Bus_Off_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Error_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}

/* Implementation of CAN1 handler named in startup code. */
void CAN1_Wake_Up_IRQHandler(void)
{
    FLEXCAN_DRV_IRQHandler(1);
}
#endif


/*******************************************************************************
 * 									fsl_uart_irq.c
 ******************************************************************************/
/*******************************************************************************
 * 									Prototypes
 ******************************************************************************/
extern void UART_DRV_IRQHandler(uint32_t instance);
/*******************************************************************************
 * 									Code
 ******************************************************************************/

#if defined (KL16Z4_SERIES) || defined (KL25Z4_SERIES) || defined (KL26Z4_SERIES) || \
    defined (KL46Z4_SERIES) || defined (KV10Z7_SERIES) || defined (KW01Z4_SERIES)
/* NOTE: If a sub-family has UART0 separated as another IP, it will be handled by
 * LPSCI driver.
 */
#if !defined (UART0_INSTANCE_COUNT) && (UART_INSTANCE_COUNT > 0)
/* Implementation of UART0 handler named in startup code. */
void UART0_IRQHandler(void)
{
    UART_DRV_IRQHandler(0);
}
#endif

#if (UART_INSTANCE_COUNT > 1)
/* Implementation of UART1 handler named in startup code. */
void UART1_IRQHandler(void)
{
    UART_DRV_IRQHandler(1);
}
#endif

#if (UART_INSTANCE_COUNT > 2)
/* Implementation of UART2 handler named in startup code. */
void UART2_IRQHandler(void)
{
    UART_DRV_IRQHandler(2);
}
#endif

#elif defined (K64F12_SERIES) || defined (K24F12_SERIES) || defined (K63F12_SERIES) || \
      defined (K22F51212_SERIES) || defined (K22F25612_SERIES) || defined (K22F12810_SERIES) || \
      defined (KV31F51212_SERIES) || defined (KV31F25612_SERIES) || defined (KV31F12810_SERIES) || \
      defined (K70F12_SERIES) || defined(K60D10_SERIES) || defined(K24F25612_SERIES) || \
      defined (KV30F12810_SERIES) || defined (K02F12810_SERIES) || defined (K21DA5_SERIES) || \
      defined (K21FA12_SERIES) || defined (KW24D5_SERIES) || defined (KV46F15_SERIES) || \
      defined (K26F18_SERIES) || defined (K65F18_SERIES) || defined (K66F18_SERIES)

#if (UART_INSTANCE_COUNT > 0)
/* Implementation of UART0 handler named in startup code. */
void UART0_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(0);
}
#endif

#if (UART_INSTANCE_COUNT > 1)
/* Implementation of UART1 handler named in startup code. */
void UART1_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(1);
}
#endif

#if (UART_INSTANCE_COUNT > 2)
/* Implementation of UART2 handler named in startup code. */
void UART2_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(2);
}
#endif

#if (UART_INSTANCE_COUNT > 3)
/* Implementation of UART3 handler named in startup code. */
void UART3_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(3);
}
#endif

#if (UART_INSTANCE_COUNT > 4)
/* Implementation of UART4 handler named in startup code. */
void UART4_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(4);
}
#endif

#if (UART_INSTANCE_COUNT > 5)
/* Implementation of UART5 handler named in startup code. */
void UART5_RX_TX_IRQHandler(void)
{
    UART_DRV_IRQHandler(5);
}
#endif

#elif defined (KL27Z644_SERIES) || defined (KL17Z644_SERIES) ||  defined (KL43Z4_SERIES)

#if (UART_INSTANCE_COUNT > 0)
/* Implementation of UART1 handler named in startup code. */
void UART2_FLEXIO_IRQHandler(void)
{
    UART_DRV_IRQHandler(2);
}
#endif

#else
    #error "No valid CPU defined!"
#endif


/*******************************************************************************
 * 									fsl_dpsi_irq.c
 ******************************************************************************/
#if defined(TWR_KV46F150M)
#define SPI0_IRQHandler SPI_IRQHandler
#endif

#if (SPI_INSTANCE_COUNT == 1)
/*!
 * @brief This function is the implementation of SPI0 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI0_IRQHandler(void)
{
    DSPI_DRV_IRQHandler(SPI0_IDX);
}

#elif (SPI_INSTANCE_COUNT == 2)
/*!
 * @brief This function is the implementation of SPI0 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI0_IRQHandler(void)
{
    DSPI_DRV_IRQHandler(SPI0_IDX);
}

/*!
 * @brief This function is the implementation of SPI1 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI1_IRQHandler(void)
{
    DSPI_DRV_IRQHandler(SPI1_IDX);
}

#else
/*!
 * @brief This function is the implementation of SPI0 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI0_IRQHandler(void)
{
    DSPI_DRV_IRQHandler(SPI0_IDX);
}

/*!
 * @brief This function is the implementation of SPI1 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI1_IRQHandler(void)
{
    DSPI_DRV_IRQHandler(SPI1_IDX);
}

/*!
 * @brief This function is the implementation of SPI2 handler named in startup code.
 *
 * It passes the instance to the shared DSPI IRQ handler.
 */
void SPI2_IRQHandler(void)
{
    DSPI_DRV_IRQHandler(SPI2_IDX);
}

#endif

/*! @} */

/*******************************************************************************
 * EOF
 ******************************************************************************/
//#endif

