/**
  ******************************************************************************
  * @file    Examples/MDR32F9Q3_EVAL/UART/Interrupt/MDR32F9Qx_it.c
  * @author  Milandr Application Team
  * @version V1.2.0
  * @date    04/07/2011
  * @brief   Main Interrupt Service Routines.
  *
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MILANDR SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Milandr</center></h2>
	D:\Cortex\lib\MDR32F9_1986BE4_2015\Example_Projects\MDR1986VE91_Eval\src
  */

/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_it.h"

#include "MDR32F9Qx_config.h"
#include <MDR32F9Qx_uart.h>
#include <MDR32F9Qx_adc.h>
//#include <MDR32F9Qx_bkp.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_it.h>
//#include <MDR32F9Qx_dma.h>
#include <MDR32F9Qx_timer.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t uart1_IT_TX_flag;
extern uint32_t uart2_IT_RX_flag;

extern uint32_t ext_IT_flag;

extern uint32_t timer_tmp;

extern __IO uint32_t H_Level;


int tmpADC ;
int tmpADC2;
int tmpPORT ;



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}
/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}
/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}
/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles Debug PendSV exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}
/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
}
/*******************************************************************************
* Function Name  : CAN1_IRQHandler
* Description    : This function handles CAN1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN1_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : CAN2_IRQHandler
* Description    : This function handles CAN2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : DMA_IRQHandler
* Description    : This function handles DMA global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : UART1_IRQHandler
* Description    : This function handles UART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART1_IRQHandler(void)
{
  if (UART_GetITStatusMasked(MDR_UART1, UART_IT_TX) == SET)
  {
    UART_ClearITPendingBit(MDR_UART1, UART_IT_TX);
    uart1_IT_TX_flag = SET;
  }
}
/*******************************************************************************
* Function Name  : UART2_IRQHandler
* Description    : This function handles UART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART2_IRQHandler(void)
{
  if (UART_GetITStatusMasked(MDR_UART2, UART_IT_RX) == SET)
  {
    UART_ClearITPendingBit(MDR_UART2, UART_IT_RX);
    uart2_IT_RX_flag = SET;
  }
}
/*******************************************************************************
* Function Name  : SSP1_IRQHandler
* Description    : This function handles SSP1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SSP1_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : I2C_IRQHandler
* Description    : This function handles I2C global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : POWER_IRQHandler
* Description    : This function handles POWER global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void POWER_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : Timer1_IRQHandler
* Description    : This function handles Timer1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Timer1_IRQHandler(void)
{
	 if (TIMER_GetITStatus(MDR_TIMER1, TIMER_STATUS_CNT_ARR) == SET)
		{		
		//TIMER_ClearITPendingBit(MDR_TIMER1, TIMER_STATUS_CNT_ARR);
			  MDR_TIMER1->STATUS &= ~TIMER_STATUS_CNT_ARR;
			
			/*while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET)
			{
			}
			UART_SendData (MDR_UART2,0x35);*/
		}

}
/*******************************************************************************
* Function Name  : Timer2_IRQHandler
* Description    : This function handles Timer2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Timer2_IRQHandler(void)
{
	if (MDR_TIMER2->STATUS & (4<<TIMER_STATUS_CCR_CAP_EVENT_Pos) )
	{
		timer_tmp++;
		//ADC1_Cmd(ENABLE);
		//ADC2_Cmd(ENABLE);
	}
}
/*******************************************************************************
* Function Name  : Timer3_IRQHandler
* Description    : This function handles Timer3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Timer3_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : ADC_IRQHandler
* Description    : This function handles ADC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_IRQHandler(void)
{
	 if(ADC1_GetFlagStatus(ADCx_IT_OUT_OF_RANGE) == SET)
  {
    /* Turns LED1 On */
    //PORT_SetBits(MDR_PORTD, PORT_Pin_10);
  }
  else
  {
    /* Turns LED1 Off */
    //PORT_ResetBits(MDR_PORTD, PORT_Pin_10);
  }
  tmpADC = MDR_ADC->ADC1_RESULT & 0x0FFF;
	
	
	tmpADC2 = MDR_ADC->ADC2_RESULT & 0x0FFF;

	
  if(tmpADC > H_Level)
  {
    /* Turns LED2 On */
    //PORT_SetBits(MDR_PORTD, PORT_Pin_11);
  }
  else
  {
    /* Turns LED2 Off */
    //PORT_ResetBits(MDR_PORTD, PORT_Pin_11);
  }
  /* Clear ADC1 OUT_OF_RANGE interrupt bit */
  MDR_ADC->ADC1_STATUS = (ADCx_IT_END_OF_CONVERSION | ADCx_IT_OUT_OF_RANGE)<<2;
	
	MDR_ADC->ADC2_STATUS = (ADCx_IT_END_OF_CONVERSION | ADCx_IT_OUT_OF_RANGE)<<2;


}
/*******************************************************************************
* Function Name  : COMPARATOR_IRQHandler
* Description    : This function handles COMPARATOR global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void COMPARATOR_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : SSP2_IRQHandler
* Description    : This function handles SSP2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SSP2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : BACKUP_IRQHandler
* Description    : This function handles BACKUP global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BACKUP_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT1_IRQHandler
* Description    : This function handles EXT_INT1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT1_IRQHandler(void)
{
//	if (UART_GetITStatusMasked(MDR_UART1, UART_IT_TX) == SET)
//  {
//    UART_ClearITPendingBit(MDR_UART1, UART_IT_TX);
//    ext_IT_flag = SET;
//  }
	ext_IT_flag++;
	
	tmpPORT = PORT_ReadInputData(MDR_PORTE);
	
	NVIC_DisableIRQ(EXT_INT1_IRQn);
	
			ADC1_Start();
			ADC2_Start();
	
	switch(tmpPORT)
	{
		case 0x0FFF:
			break;
		case 0x0FFE:
			break;
		case 0x0FFA:
			break;
		default:
			break;
	}
}
/*******************************************************************************
* Function Name  : EXT_INT2_IRQHandler
* Description    : This function handles EXT_INT2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT3_IRQHandler
* Description    : This function handles EXT_INT3 global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT3_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT4_IRQHandler
* Description    : This function handles EXT_INT4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT4_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2011 Milandr *********/

/* END OF FILE MDR32F9Qx_it.c */
