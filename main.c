/**
  ******************************************************************************
  * @file    Examples/MDR32F9Q3_EVAL/UART/Interrupt/main.c
  * @author  Milandr Application Team
  * @version V1.2.0
  * @date    04/07/2011
  * @brief   Main program body.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MILANDR SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Milandr</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"

#include "MDR32F9Qx_adc.h"

#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_it.h"



#include "MDR32F9Qx_timer.h"

/** @addtogroup __MDR32F9Qx_StdPeriph_Examples MDR32F9Qx StdPeriph Examples
  * @{
  */

/** @addtogroup __MDR32F9Q3_EVAL MDR32F9Q3 Evaluation Board
  * @{
  */

/** @addtogroup UART_Interrupt_93 UART_Interrupt
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Measurements Data  */
// ==================================================================================================
struct {// Structure for Doase and Fluxes data collecting
  uint8_t Metka1; 
  uint8_t metka2;
  uint32_t day[4];		//  month, day, hour, minute
  uint32_t Counter_of_Detector_1, Prot_1, Prot_2, Prot_Comp_1, Neut_1, Neut_2;
  uint32_t Dose_1, Dose_2, Dose_Comp_1;
  uint8_t DF[480];	//
} DOSE_AND_FLUX;
//===================================================================================================

struct { // Structure for Dose and Fluxes data current values
  uint16_t Counter_of_Detector_1, Prot_1, Prot_2, Prot_Comp_1, Neut_1, Neut_2;
  uint32_t Dose_1, Dose_2, Dose_Comp_1;
} D_AND_F;


// ===========================================================================

struct {
  uint16_t Metka1; 
  uint16_t metka2;
  uint32_t Time;
  uint32_t interr_reg;
  uint32_t FLUGS;
  uint32_t ADC_code, P1_Counter;
  uint32_t Pr1, Pr2;
  uint32_t Dt1, Dt2;
  uint32_t N1, N2;
} Temporal_buff;

// ===========================================================================

#define Spectr_size   64  
#define Spectr_number 4

//Other variant
//For forming of spectrums

unsigned short Spectra[Spectr_number][Spectr_size]; // Array for accumulation of energy deposition spectrums in detectors.

/* Private define ------------------------------------------------------------*/
#define METKA1  			0xf0ff// start 61695
#define METKA2  			0xf0ff// start 61695
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;
uint32_t uart1_IT_TX_flag = RESET;
uint32_t uart2_IT_RX_flag = RESET;

ADC_InitTypeDef sADC;
ADCx_InitTypeDef sADCx;
uint32_t tmp ;

uint32_t timer_tmp;

__IO uint32_t H_Level;

//TIMER_CntInitTypeDef sTIM_CntInit;
//TIMER_ChnInitTypeDef sTIM_ChnInit;


uint32_t ext_IT_flag;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Uart1PinCfg(void)
{
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    /* Configure PORTB pins 6 (UART1_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_6;
    PORT_Init(MDR_PORTB, &PortInit);
    /* Configure PORTB pins 5 (UART1_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_5;
    PORT_Init(MDR_PORTB, &PortInit);	
}


void Uart1Setup(void)
{
	/* Select HSI/2 as CPU_CLK source*/
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    /* Enables the CPU_CLK clock on UART1 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART1, ENABLE);
    /* Set the HCLK division factor = 1 for UART1*/
	UART_BRGInit(MDR_UART1, UART_HCLKdiv1);

    UART_InitStructure.UART_BaudRate                = 12000; // not applied! - Why??? 
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

	/* Configure UART1 parameters*/
	UART_Init (MDR_UART1,&UART_InitStructure);
    /* Enables UART1 peripheral */
    UART_Cmd(MDR_UART1,ENABLE);
}



int UartTest (void)
{
uint8_t DataByte;
//static uint8_t ReciveByte[16];
//int i;

	Uart1PinCfg();
	Uart1Setup();
 	while(1)
	{
		/* Check RXFF flag*/
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_RXFF)!= SET);
        /* Recive data*/
		DataByte = UART_ReceiveData (MDR_UART1); 
		
		
		if (DataByte == 'g')
		{


			ADC2_Start();
			
			while (!ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION));
			
			tmp = ADC2_GetResult(); // ADC_GetStatus();
			
			// tmp = MDR_UART1->FR; //Good!
			
			// MDR_ADC->ADC2_CFG = 0x1E00F020;
			
			// tmp = MDR_ADC->ADC2_CFG;
			
			//tmp = 0xfafbfcfd; //Good!
			
			/* Send Data from UART1 */
			
			
			tmp = ext_IT_flag;
			
			while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART1, (uint16_t)(tmp));
			while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART1, (uint16_t)(tmp>>8));
			while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART1, (uint16_t)(tmp>>16));
			while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART1, (uint16_t)(tmp>>24));
				
				// Enable EXT INT !	
				NVIC_EnableIRQ(EXT_INT1_IRQn);
				
				
//			
//			
//			for (i=0;i<16;i++)
//			{
//				/* Check TXFE flag */
//				while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
//				{
//				}
//				static uint8_t str[16] = "Hello from MDR!";
//				uint8_t *point_str = str;
//				UART_SendData (MDR_UART1, (uint16_t)(str[i] ));
//			}
				
				
		}
		/* Check TXFE flag*/
		while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET);
        /* Send Data */
		
        UART_SendData (MDR_UART1,DataByte);

	}
exit: return 0;
}


void MltPinCfg (void)
{
	/* PORTA ext interrupt PA0*/
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
	/* Configure PORTA pins 0..7 for mlt inout data  */
	PortInit.PORT_Pin   = PORT_Pin_0;
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW; //what does it means? maybe power connected to port?

	PORT_Init(MDR_PORTA, &PortInit);
	
	/* PORTB UART pins*/
	Uart1PinCfg();

	/* PORTE input of all counters*/
		/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
	/* Configure PORTD pins 3 for mlt output  */
	PortInit.PORT_Pin   = (PORT_Pin_3);
	PortInit.PORT_OE    = PORT_OE_OUT;

	PORT_Init(MDR_PORTE, &PortInit);

	
	/* Configure ADC1 and ADC2 pin: PD2 PD3 */
  /* Configure PORTD pin 2 */
		PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3);
		PortInit.PORT_OE    = PORT_OE_IN;
		PortInit.PORT_MODE  = PORT_MODE_ANALOG;
  PORT_Init(MDR_PORTD, &PortInit);

}

/*******************************************************************************
* Function Name  : ADC_Temp_Sensor_Config
* Description    : Configure the ADC1 for temperature sensor reading.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*******************************************************************************
* Function Name  : ADC_Config
* Description    : Configure the ADC1 for TRIM using.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_Config(uint32_t acd_ch)
{
  /* ADC Configuration */
  /* Reset all ADC settings */
  ADC_DeInit();
  ADC_StructInit(&sADC);
  ADC_Init (&sADC);

  ADCx_StructInit (&sADCx);
  sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_SINGLE_CONV;//ADC_SAMPLING_MODE_CICLIC_CONV
  sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
  sADCx.ADC_ChannelNumber    = acd_ch;//ADC_CH_ADC2;
  sADCx.ADC_Channels         = 0;
  sADCx.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;//ADC_LEVEL_CONTROL_Enable
  sADCx.ADC_LowLevel         = 0x100;//L_Level noise level
  sADCx.ADC_HighLevel        = 0x900;//H_Level
  sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
  sADCx.ADC_Prescaler        = ADC_CLK_div_8;//ADC_CLK_div_32768;
  sADCx.ADC_DelayGo          = 0xF;
  ADC2_Init (&sADCx);
	
	//uint32_t tmpreg_CFG = MDR_ADC->ADC2_CFG;

  /* Enable ADC2 EOCIF and AWOIFEN interrupts */
  ADC1_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), ENABLE);

}

void ADC2_Config(uint32_t acd_ch)
{
  	
	/* ADC Configuration */
  /* Reset all ADC settings */
  ADC_DeInit();
  ADC_StructInit(&sADC);
  ADC_Init (&sADC);

  ADCx_StructInit (&sADCx);
  sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_SINGLE_CONV;//ADC_SAMPLING_MODE_CICLIC_CONV
  sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
  sADCx.ADC_ChannelNumber    = acd_ch;//ADC_CH_ADC3;
  sADCx.ADC_Channels         = 0;
  sADCx.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;//ADC_LEVEL_CONTROL_Enable
  sADCx.ADC_LowLevel         = 0x800;//L_Level
  sADCx.ADC_HighLevel        = 0x900;//H_Level
  sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
  sADCx.ADC_Prescaler        = ADC_CLK_div_8;//ADC_CLK_div_32768;
  sADCx.ADC_DelayGo          = 0xF;
  ADC2_Init (&sADCx);
	
	//uint32_t tmpreg_CFG = MDR_ADC->ADC2_CFG;

  /* Enable ADC2 EOCIF and AWOIFEN interrupts */
  ADC2_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), ENABLE);

}


void ADC_Temp_Sensor_Config(void)
{
  /* Enable the RTCHSE clock on ADC1 */
  RST_CLK_PCLKcmd((RST_CLK_PCLK_ADC), ENABLE);

  /* ADC Configuration */
  /* Reset all ADC settings */
  ADC_DeInit();

  ADC_StructInit(&sADC);
  sADC.ADC_TempSensor           = ADC_TEMP_SENSOR_Enable;
  sADC.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Enable;
  sADC.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Enable;
  ADC_Init (&sADC);
	
  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CICLIC_CONV;
  sADCx.ADC_ChannelNumber    = ADC_CH_TEMP_SENSOR;
  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_EXACT;
  sADCx.ADC_Prescaler        = ADC_CLK_div_8;//ADC_CLK_div_32768;
  sADCx.ADC_DelayGo          = 0x7;
  ADC1_Init (&sADCx);
	
	
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main (void)
{
	/* Enables the High Speed External clock */
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
    while (RST_CLK_HSEstatus() != SUCCESS);
    
	/* Enables the clock on PORTA */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);
	/* Enables the clock on PORTB */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);	
    /* Enables the clock on PORTC */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
	/* Enables the clock on PORTD */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);
	/* Enables the clock on PORTE */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);
	/* Enables the HSI clock on PORTF */
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);
	
	/* Enables the HSI clock on ExtBus */
    RST_CLK_PCLKcmd(RST_CLK_PCLK_EBC, ENABLE);	
	/* Enables the ADC clock on ExtBus */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER2, ENABLE);

	
	MltPinCfg ();
	
	ADC1_Config(ADC_CH_ADC2);//pd2
	ADC2_Config(ADC_CH_ADC3);//pd3
	
  /* ADC2 enable */
	ADC1_Cmd (ENABLE);

  ADC2_Cmd (ENABLE);	
		
	NVIC_EnableIRQ(EXT_INT1_IRQn);	
	NVIC_EnableIRQ(EXT_INT2_IRQn);	
	NVIC_EnableIRQ(EXT_INT3_IRQn);	
	NVIC_EnableIRQ(EXT_INT4_IRQn);	

	
	UartTest ();
	
	
}

/**
  * @brief  Reports the source file ID, the source line number
  *         and expression text (if USE_ASSERT_INFO == 2) where
  *         the assert_param error has occurred.
  * @param  file_id: pointer to the source file name
  * @param  line: assert_param error line source number
  * @param  expr:
  * @retval None
  */
#if (USE_ASSERT_INFO == 1)
void assert_failed(uint32_t file_id, uint32_t line)
{
  /* User can add his own implementation to report the source file ID and line number.
     Ex: printf("Wrong parameters value: file Id %d on line %d\r\n", file_id, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#elif (USE_ASSERT_INFO == 2)
void assert_failed(uint32_t file_id, uint32_t line, const uint8_t* expr);
{
  /* User can add his own implementation to report the source file ID, line number and
     expression text.
     Ex: printf("Wrong parameters value (%s): file Id %d on line %d\r\n", expr, file_id, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_ASSERT_INFO */

/** @} */ /* End of group UART_Interrupt_93 */

/** @} */ /* End of group __MDR32F9Q3_EVAL */

/** @} */ /* End of group __MDR32F9Qx_StdPeriph_Examples */

/******************* (C) COPYRIGHT 2011 Milandr *********/

/* END OF FILE main.c */

