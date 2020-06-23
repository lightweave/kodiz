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
#include "float.h"




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

/* Private typedef -----------------------------------------------------------
–	Потоки протонов и ядер с Z>1 c энергией больше 30…50 МэВ/нуклон в диапазоне от 101 до 104 частиц/см2 × с.
–	Потоки протонов и ядер с Z>1 c энергией больше 330 МэВ/нуклон в диапазоне от 101 до 103 частиц/см2 × с.
–	Поток тепловых и эпитепловых нейтронов в диапазоне от 101 до 103 нейтронов/см2 × с.
–	Мощность поглощенной дозы заряженных частиц космического излучения в диапазоне от 10-8 до 10-5 Грей/с.

*/

// ===========================================================================
  // Table assigning symbol, that identifies array and array content
  // Symbol	array content
  // F		Collected dose and fluses values, dose dinamics for each second
  // S		Energy deposition spectra
  // T		Test information
  // H 		High Amplitude Data
  // N		Neutron burst data
// ===========================================================================

// Constants and variables for data transmission through CAN-port
#define Buffer_Size 512     // Length, in bytes, of one buffer for data transmission
#define Buffer_Number 32		// Number of buffers = 8  -- need to increase!!!!

struct Results_Buffer {// Type of structure of output buffer
  char ready;	// Flag. When information is ready - a value of parameter >0
  char buffer[Buffer_Size]; // Array of bytes, containing the information to the transmission
};
// ==== Global variables, managers by a data transmission through CAN-port =====
struct Results_Buffer Data_Buffer[Buffer_Number]; // Array of output buffers.

int Buff_Write_Index=0;                 //  Index of buffer, to which written down in progress
int Buff_Send__Index = Buffer_Number-1; //  Index of buffer, from which data transmission in progress
int Flug_Buff = -3;                     //  Flag of information transmission from a buffer
                                        // -3 means that all of buffers are empty
                                        // -2 means that all of buffers are already transmitted
                                        // -1 means that it is a buffer prepared to the transmission,
                                        // Flug_Buff >= 0 - error
/* Measurements Data  */
// ==================================================================================================
struct {// Structure for Doase and Fluxes data collecting every minute
  uint16_t Metka1; 
  uint16_t metka2;
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
  uint32_t FLAGS;
  uint32_t ADC_code, P1_Counter;
  uint32_t Pr1, Pr2;
  uint32_t Dt1, Dt2;
  uint32_t N1, N2;
} Temporal_buff;

// ===========================================================================
// buffer for memory chunks
struct {
	uint16_t Metka1; 
  uint16_t metka2;
  uint32_t Time;
	
	uint8_t memory[480];
} Memory_buff;
// ==================================================================================================

#ifdef DEBUGBUFFER

unsigned char debug_buf[512];
unsigned int debug_duf_index = 0;

#endif


//usage:
//#define DEBUGBUFFER

#ifdef DEBUGBUFFER

  memset(debug_buf, 12, Buffer_Size);
  *(unsigned int *) debug_buf = METKA1 ;
  debug_buf[2]='T';
  debug_duf_index =4;
  
#endif



// ===========================================================================

#define Spectrum_size   64  
#define Spectrum_number 4

//Other variant
//For forming of spectra

unsigned short Spectra[Spectrum_number][Spectrum_size]; // Array for accumulation of energy deposition spectrums in detectors.

// ===========================================================================

//For neutron berst registration
#define Neutron_MAX 10
struct {
  uint16_t Metka1; 
  uint16_t metka2;
  uint32_t neutron_t[127];	
} Neutron_Bunch;
uint32_t Old_Tick_2, Old_Sec, N_ind=Neutron_MAX, i_n_t=0;

uint32_t  D_tick_2=32;  //  интервал времени меньше которого считается, что идет один всплеск, мсек/6
float Proton_Fon=0, Proton_Level=2.5;	// Текущий фон потока протонов и уровень, выше которого нейтронные всплески не регистр.
float Beta=0.9, Alfa= 0.1 ; 		// Константа экспоненциального сглаживания для расчета фона протонов

/* Private define ------------------------------------------------------------*/
#define METKA1  			0xf0ff// start 61695
#define METKA2  			0xf0ff// start 61695

// all main counters declared here
uint32_t  Dose_1, Dose_2, Dose_Comp_1, Dose_Comp_2, Prot_1_Interrupt, Prot_2_Interrupt;
uint32_t  Detectors_Flugs = 0;
uint32_t  Counter_of_Detector_1, Prot_1, Prot_2, Prot_Comp_1, Prot_Comp_2 ,Neutron_1, Neutron_2, Neut_1, Neut_2;
uint32_t  tick_1, tick_2, Buff_No=0, Spec_No=0, Command_No=0;





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

// ==================================================================================================
void Delayms(__IO uint32_t nCount)
{
	nCount = 16000*nCount;// 16 MGz
  for (; nCount != 0; nCount--);
}
void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}
// ==================================================================================================
void Counters_Reset(void)
{
  Counter_of_Detector_1=0;  
  Neutron_1=0;  Neutron_2=0;
  Neut_1=0;     Neut_2=0;
  Prot_1=0;     Prot_2=0;  
  Prot_1_Interrupt=0;      Prot_2_Interrupt=0;     
}
// ==================================================================================================
void FLUX_Reset(void)
{
//memset((char *) & D_AND_F, 0, sizeof(D_AND_F));
//memset((char *) & DOSE_AND_FLUX, 0, sizeof(DOSE_AND_FLUX));
DOSE_AND_FLUX.Metka1= METKA1;
DOSE_AND_FLUX.metka2 = 0x4600; //DOSE_AND_FLUX.metka2='F';//DOSE_AND_FLUX.metka2='\0';
}

// ==================================================================================================
void Reset_Spectra_and_Counters (void)
{
 int i,j;

   for(j=0; j<Spectrum_number; j++)
   for( i=0; i < Spectrum_size; i++)   // *Spectr_size
     Spectra[j][i] =0;

 Counter_of_Detector_1=0;
 Prot_1=0; Prot_2=0; Prot_Comp_1=0;  Prot_Comp_2=0;
 Neutron_1=0;  Neutron_2=0;
 Dose_1=0;  Dose_2=0;  Dose_Comp_1=0;  Dose_Comp_2=0;
 Prot_1_Interrupt=0; Prot_2_Interrupt=0;
 // To whip off the vehicle meter of channel of P1
 Neut_1=0; Neut_2=0;
}
// ==================================================================================================
// Setting to null circle buffer for sending by CAN
void Data_Buffer_Reset(void)
{
     int i; 
     Buff_Send__Index = Buffer_Number -1;
     Buff_Write_Index=0;
     Flug_Buff = -3;
     for (i=0; i < Buffer_Number; i++) {
//       memset(Data_Buffer[i].buffer, 0, Buffer_Size);
       Data_Buffer[i].ready=0;//
     }
}
// This function must be used for initialisation of all values that need 
void Detectors_Init(void)
{
//  Clear_Registers_Bits();  
// ===========================
  //ConfigureTimer();  
  Counters_Reset();
  Data_Buffer_Reset();
  FLUX_Reset();
  Reset_Spectra_and_Counters ();  
 
  
  
  
}  
// ==================================================================================================

void Uart2PinCfg(void)
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
	
    /* Configure PORTB pins 0 (UART2_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_0;
    PORT_Init(MDR_PORTD, &PortInit);
    /* Configure PORTB pins 1 (UART2_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_1;
    PORT_Init(MDR_PORTD, &PortInit);	
	
}
void Uart2Setup(void)
{
	/* Select HSI/2 as CPU_CLK source*/
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    /* Enables the CPU_CLK clock on UART2 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
    /* Set the HCLK division factor = 1 for UART2*/
	UART_BRGInit(MDR_UART2, UART_HCLKdiv1);

    UART_InitStructure.UART_BaudRate                = 12000; // not applied! - Why??? 
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

	/* Configure UART2 parameters*/
	UART_Init (MDR_UART2,&UART_InitStructure);
    /* Enables UART2 peripheral */
    UART_Cmd(MDR_UART2,ENABLE);
}
// ==================================================================================================

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
	
    /* DELETED - cause mulfunction of controller !!! Configure PORTB pins 6 DE pin tranceiver as output for KODIZ board----------- */

    /* Configure PORTB pins 7 (DE pin tranceiver) as output */ 			
    PortInit.PORT_Pin = (PORT_Pin_7);
	  PortInit.PORT_OE    = PORT_OE_OUT;
	  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	  PortInit.PORT_SPEED = PORT_SPEED_FAST;
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

void Uart1SendByte(uint16_t Data) //void UART_SendData(MDR_UART_TypeDef* UARTx, uint16_t Data)
{
	
				PORT_SetBits(MDR_PORTB, PORT_Pin_7);
			  UART_SendData (MDR_UART1, Data);
				while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
				{
				}
				PORT_ResetBits(MDR_PORTB, PORT_Pin_7);
}

int UartTest (void)
{
uint8_t DataByte;
//static uint8_t ReciveByte[16];
//int i;

	Uart1PinCfg();
	Uart1Setup();
	
	//Uart2PinCfg();
	//Uart2Setup();
	
 	while(1)
	{
		while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
				{
				}
				Uart1SendByte('f');
				
				//Delayms(10);
			}
	
			
			
			
			
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
			
			while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART2, (uint16_t)(tmp));
			while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART2, (uint16_t)(tmp>>8));
			while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART2, (uint16_t)(tmp>>16));
			while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET)
				{
				}
			UART_SendData (MDR_UART2, (uint16_t)(tmp>>24));
				
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
		while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET);
        /* Send Data */
		
        UART_SendData (MDR_UART2,DataByte);

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
	// Uart1PinCfg();

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
  /* Configure PORTD pin 2 It ac brake JTAG B*/
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
	// it delay helps to start debugging - pragram cannot block jtag. But need to delete in production!!!	it works ~ 5 s in debug mode
	Delayms(200);
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

// ==================================================================================================
uint8_t  Counter_Compression(uint32_t j) {
  uint8_t k;
  uint32_t mask;
        if(j&0x0ffffff0){
            mask=0x08000000;
            for (k=24;k>0; k--){
                if(j & mask){
                   j>>=k; j=j& 0x0007;
                   j |=((k+1)<<3); break;
                } else mask >>= 1;
            }
        }
 return (uint8_t) j;
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
=
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

