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
#include <MDR32F9Qx_bkp.h>
#include <MDR32F9Qx_port.h>
//#include <MDR32F9Qx_it.h>
//#include <MDR32F9Qx_dma.h>
#include <MDR32F9Qx_timer.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t uart1_IT_TX_flag;
extern uint32_t uart1_IT_RX_flag;
extern uint32_t uart2_IT_RX_flag;

extern uint16_t uart1_IT_RX_byte;

extern uint32_t ext_IT1_flag;
extern uint32_t ext_IT2_flag;
extern uint32_t ext_IT3_flag;
extern uint32_t ext_IT4_flag;

extern uint32_t timer_tmp;

extern __IO uint32_t H_Level;

extern uint32_t tmpPORT ;
extern uint32_t adcDelay;


#define raw_adc_size   64 // for temporary storage of adc counts before filling spectra
#define raw_adc_number  2 // only two avaliable adcs
extern unsigned int raw_ADC[raw_adc_number][raw_adc_size];


extern uint32_t tmpADC;
extern uint32_t tmpADC2;

uint32_t m_error;
uint32_t error_none;
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
  if (UART_GetITStatusMasked(MDR_UART1, UART_IT_RX) == SET)
  {		
    UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
		
		uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
		
		uart1_IT_RX_flag = SET;	
		
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
  }



    // --------------------------------------------- Ошибки

    // Проверяем на ошибки - обязательно после приема!
    // К сожалению, функций SPL для этого нет
    uint32_t m_error = MDR_UART1->RSR_ECR;

    if( m_error != error_none )
    {
        // Ошибки в регистре сбрасывается
        MDR_UART1->RSR_ECR = 0;
    }
	
    // --------------------------------------------- Передача
//просто передаем записывая в буфер

}



//void Handle :: irqHandler(void)
//{
//    UMBA_ASSERT( m_isInited );

//    m_irqCounter++;

//    // --------------------------------------------- Прием

//    // do нужен только чтобы делать break
//    do
//    {
//        if ( UART_GetITStatusMasked( m_mdrUart, UART_IT_RX ) != SET )
//            break;

//        // по-факту, прерывание сбрасывается при чтении байта, но это недокументированная фича
//        UART_ClearITPendingBit( m_mdrUart, UART_IT_RX );

//        uint8_t byte = UART_ReceiveData( m_mdrUart );

//        // для 485 используется режим шлейфа, поэтому мы можем принимать эхо самих себя
//        if( m_rs485Port != nullptr && m_echoBytesCounter > 0 )
//        {
//            // эхо нам не нужно
//            m_echoBytesCounter--;

//            if( m_echoBytesCounter == 0 )
//            {
//                // после последнего байта надо __подождать__,
//                // потому что мы принимаем его эхо до того, как стоп-бит до конца вылезет на линию
//                // из-за мажоритарной логики семплирования.
//                // Если не ждать, то можно потерять около трети стоп-бита.

//                // Время ожидания зависит от бодрейта, примерное время ожидания:

//                // бодрейт | длительность бита, |  время ожидания, |
//                //         |        мкс         |       мкс        |
//                //         |                    |                  |
//                // 9600    |      105           |       32         |
//                // 57600   |       18           |       4,5        |
//                // 921600  |        1           |        0         |
//                //         |                    |                  |

//                // при использовании двух стоп бит и/или бита четности,
//                // время прополки вроде как не меняется.
//                // Видимо, пропалывается только треть последнего бита, не важно какого.

//                // блокирующе пропалываем бит
//                while( m_mdrUart->FR & UART_FR_BUSY ) {;}

//                // и только теперь можно выключать передатчик и режим шлейфа
//                rs485TransmitDisable();

//                // семафор, что передача завершена
//                #ifdef UART_USE_FREERTOS
//                    osSemaphoreGiveFromISR( m_transmitCompleteSem, NULL );
//                #endif
//            }

//            break;
//        }

//        // если в приемнике нет места - байт теряется и выставляется флаг overrun
//        #ifdef UART_USE_FREERTOS

//            BaseType_t result = osQueueSendToBackFromISR( m_rxQueue, &byte, NULL );

//            if( result == errQUEUE_FULL )
//            {
//                m_isRxOverrun = true;
//            }

//        #else

//            if( m_rxBuffer.isFull() )
//            {
//                m_isRxOverrun = true;
//            }
//            else
//            {
//                m_rxBuffer.writeHead(byte);
//            }

//        #endif


//    } while( 0 );

//    // --------------------------------------------- Ошибки

//    // Проверяем на ошибки - обязательно после приема!
//    // К сожалению, функций SPL для этого нет
//    m_error = m_mdrUart->RSR_ECR;

//    if( m_error != error_none )
//    {
//        // Ошибки в регистре сбрасывается
//        m_mdrUart->RSR_ECR = 0;
//    }

//    // --------------------------------------------- Передача

//    if( UART_GetITStatusMasked( m_mdrUart, UART_IT_TX ) != SET )
//        return;

//    // предпоследний байт в 485 - включаем режим шлейфа
//    if( m_txCount == m_txMsgSize - 1 && m_rs485Port != nullptr )
//    {
//        setEchoModeState( true );
//        m_echoBytesCounter = 2;
//    }
//    // все отправлено
//    else if( m_txCount == m_txMsgSize )
//    {
//        // явный сброс можно (и нужно) делать только для последнего байта
//        UART_ClearITPendingBit( m_mdrUart, UART_IT_TX );
//        m_pTxBuf = nullptr;

//        return;
//    }

//    // Еще есть, что отправить
//    UMBA_ASSERT( m_pTxBuf != nullptr );

//    UART_SendData( m_mdrUart, m_pTxBuf[ m_txCount ] );
//    m_txCount++;
//}
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
			
			
			
			

//MDR_TIMER1->STATUS &= ~( 1 << 8 ) ;
			
			

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
//	 if(ADC1_GetFlagStatus(ADCx_IT_OUT_OF_RANGE) == SET)
//  {
//    /* Turns LED1 On */
//    //PORT_SetBits(MDR_PORTD, PORT_Pin_10);
//  }
//  else
//  {
//    /* Turns LED1 Off */
//    //PORT_ResetBits(MDR_PORTD, PORT_Pin_10);
//  }
	
	
	if(ADC1_GetFlagStatus(ADCx_FLAG_END_OF_CONVERSION) == SET)
	{
		tmpADC = MDR_ADC->ADC1_RESULT & 0x0FFF;	
		raw_ADC[0][0] = tmpADC;
	}
	
	if(ADC2_GetFlagStatus(ADCx_FLAG_END_OF_CONVERSION) == SET)
	{
		tmpADC2 = MDR_ADC->ADC2_RESULT & 0x0FFF;
		raw_ADC[1][0] = tmpADC2;
	}
	
	

  PORT_ResetBits(MDR_PORTB, PORT_Pin_14); // Опускание ножки PB15 после считывания результата с АЦП	
	
//  if(tmpADC > H_Level)
//  {
//    /* Turns LED2 On */
//    //PORT_SetBits(MDR_PORTD, PORT_Pin_11);
//  }
//  else
//  {
//    /* Turns LED2 Off */
//    //PORT_ResetBits(MDR_PORTD, PORT_Pin_11);
//  }
  /* Clear ADC1 OUT_OF_RANGE interrupt bit */
  // MDR_ADC->ADC1_STATUS = (ADCx_IT_END_OF_CONVERSION | ADCx_IT_OUT_OF_RANGE)<<2;
	
	// MDR_ADC->ADC2_STATUS = (ADCx_IT_END_OF_CONVERSION | ADCx_IT_OUT_OF_RANGE)<<2;


	
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
//void BACKUP_IRQHandler(void)
//{
//	if (BKP_RTC_GetFlagStatus(BKP_RTC_FLAG_SECF) == SET)
//  {
//    BKP_RTC_ITConfig(BKP_RTC_IT_SECF, DISABLE);

//    /* If counter is equal to 86339: one day was elapsed */
//    tmp = BKP_RTC_GetCounter();
//    if ((tmp / 3600 == 23) &&
//        (((tmp % 3600) / 60) == 59) &&
//        (((tmp % 3600) % 60) == 59))
//    {
//      /* Wait until last write operation on RTC registers has finished */
//      BKP_RTC_WaitForUpdate();
//      /* Reset counter value */
//      BKP_RTC_SetCounter(0);
//      /* Wait until last write operation on RTC registers has finished */
//      BKP_RTC_WaitForUpdate();

//      /* Increment the date */
//      Date_Update();
//    }
//    BKP_RTC_ITConfig(BKP_RTC_IT_SECF, ENABLE);
//  }

//if (BKP_RTC_GetFlagStatus(BKP_RTC_FLAG_SECF)==SET)
//  {
//    if (PORT_ReadInputDataBit(MDR_PORTF,PORT_Pin_0)==0)
//    {
//      PORT_SetBits(MDR_PORTF,PORT_Pin_0);
//    }
//	else
//    {
//      PORT_ResetBits(MDR_PORTF,PORT_Pin_0);
//    }
//  }
//  if (BKP_RTC_GetFlagStatus(BKP_RTC_FLAG_ALRF)==SET)
//  {
//			 Srart_Uart_sending((uint8_t *)Hello_text3, 129);

//    //PORT_SetBits(MDR_PORTF,PORT_Pin_1);
//  }
//  MDR_BKP -> RTC_CS |= 0x06;
//}
/*******************************************************************************
* Function Name  : EXT_INT1_IRQHandler
* Description    : This function handles EXT_INT1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void EXT_INT1_IRQHandler(void)
{
//	if (UART_GetITStatusMasked(MDR_UART1, UART_IT_TX) == SET)
//  {
//    UART_ClearITPendingBit(MDR_UART1, UART_IT_TX);
//    ext_IT_flag = SET;
//  }
	ext_IT1_flag++;
	
	tmpPORT = PORT_ReadInputData(MDR_PORTC);
	
	NVIC_DisableIRQ(EXT_INT1_IRQn);
	
	ADC1_Start();// pd2 detector 2
	ADC2_Start(); // pd3 detector 3
	
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
}*/
/*******************************************************************************
* Function Name  : EXT_INT2_IRQHandler
* Description    : This function handles EXT_INT2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern uint32_t ext_IT_own;
uint32_t nCount;



/*void EXT_INT2_IRQHandler(void)
{	
	//this cicle gives 300ns for 1 nCount, stable enought
	nCount = adcDelay;
  for (; nCount != 0; nCount--);
	

  // for stable delay time we need to use DWT, but it is unstable!
	// but n=2 n=7 not work!	
//	DWT->CYCCNT = 0;
//  while (DWT->CYCCNT < (adcDelay * 3*80/10)); // 3*SYSCLK_MHZ/10 = 300ns
	
	
	
	ADC1_Start();// pd2 detector 2
	ADC2_Start(); // pd3 detector 3		

	
	tmpPORT = PORT_ReadInputData(MDR_PORTC);

	// Поднятие ножки PB14 перед включением АЦП			
	PORT_SetBits(MDR_PORTB, PORT_Pin_14);
	
	NVIC_DisableIRQ(EXT_INT2_IRQn);
	NVIC_DisableIRQ(EXT_INT4_IRQn);
	
	ext_IT2_flag++;	
}*/
/*******************************************************************************
* Function Name  : EXT_INT3_IRQHandler
* Description    : This function handles EXT_INT3 global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


/*void EXT_INT3_IRQHandler(void)
{
	ext_IT3_flag++;
	
	tmpPORT = PORT_ReadInputData(MDR_PORTE);
	
	NVIC_DisableIRQ(EXT_INT3_IRQn);
	
	ADC1_Start();// pd2 detector 2
	ADC2_Start(); // pd3 detector 3
}*/
/*******************************************************************************
* Function Name  : EXT_INT4_IRQHandler
* Description    : This function handles EXT_INT4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void EXT_INT4_IRQHandler(void)
//{	
////this cicle gives 300ns for 1 nCount, stable enought
//	nCount = adcDelay;
//  for (; nCount != 0; nCount--);
//	

//  // for stable delay time we need to use DWT, but it is unstable!
//	// but n=2 n=7 not work!	
////	DWT->CYCCNT = 0;
////  while (DWT->CYCCNT < (adcDelay * 80/10)); // 3*SYSCLK_MHZ/10 = 300ns
//  //no effect:
//	//__NOP;

//	ADC2_Start(); // pd3 detector 1	
//	ADC1_Start();
//	
//	tmpPORT = PORT_ReadInputData(MDR_PORTC);
//	
//	// Поднятие ножки PB14 c включением АЦП		
//	PORT_SetBits(MDR_PORTB, PORT_Pin_14);
//	
//	ext_IT4_flag++;
//	
//	NVIC_DisableIRQ(EXT_INT2_IRQn);
//	NVIC_DisableIRQ(EXT_INT4_IRQn);
//	
//	
//	
//	
//	
//}

/******************* (C) COPYRIGHT 2011 Milandr *********/

/* END OF FILE MDR32F9Qx_it.c */
