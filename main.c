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
#include <MDR32F9Qx_bkp.h>

#include "MDR32F9Qx_adc.h"

#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_it.h"
#include "float.h"
// #include "time.h"// not that time file



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
#define Buffer_Number 2		// Number of buffers = 8  -- need to increase!!!!

struct Results_Buffer {// Type of structure of output buffer
  uint8_t ready;	// Flag. When information is ready - a value of parameter >0
  uint8_t buffer[Buffer_Size]; // Array of bytes, containing the information to the transmission
};
// ==== Global variables, managers by a data transmission through CAN-port =====
struct Results_Buffer Data_Buffer[Buffer_Number]; // Array of output buffers.

int Buff_Write_Index=0;                 //  Index of buffer, to which written down in progress
int Buff_Send__Index = 0;//Buffer_Number-1; //  Index of buffer, from which data transmission in progress
int Flug_Buff = -3;                     //  Flag of information transmission from a buffer
                                        // -3 means that all of buffers are empty
                                        // -2 means that all of buffers are already transmitted
                                        // -1 means that it is a buffer prepared to the transmission,
                                        // Flug_Buff >= 0 - error
int sendingIndex;
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
uint32_t uart1_IT_RX_flag = RESET;
uint32_t uart2_IT_RX_flag = RESET;
uint16_t sendbyte;

	uint16_t ext_IT2_flag_previous = 0;	
  uint16_t ext_IT4_flag_previous = 0;	
  uint32_t tmpPORTc = 0;

ADC_InitTypeDef sADC;
ADCx_InitTypeDef sADCx;
uint32_t tmp ;
uint32_t tmp2 ;

uint32_t timer_tmp;

__IO uint32_t H_Level;

//TIMER_CntInitTypeDef sTIM_CntInit;
//TIMER_ChnInitTypeDef sTIM_ChnInit;


uint32_t ext_IT1_flag;
uint32_t ext_IT2_flag;
uint32_t ext_IT3_flag;
uint32_t ext_IT4_flag;
uint32_t ext_IT_own;


uint16_t uart1_IT_RX_byte;

uint32_t tmpPORT ; //Interruption
uint32_t adcDelay;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// ==================================================================================================
void Delayms(__IO uint32_t nCount)
{
	nCount = 8000*nCount;// 8 MGz - speed of internal generator
  for (; nCount != 0; nCount--);
}
void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}

//-------------------------------------------------------------------------------
// Задержка полингом по DWT таймеру, вх. параметр cnt в микросекундах (10 мкс == 10)
// Параметр SYSCLK_MHZ - частота ядра в мегагерцах (8МГц == 8)
//-------------------------------------------------------------------------------
#pragma inline
void Sleep(uint32_t cnt)
{	  
	DWT->CYCCNT = 0;
	while (DWT->CYCCNT < (cnt * 80));//SYSCLK_MHZ
}

//-------------------------------------------------------------------------------
// Предварительно включить и настроить DWT
//-------------------------------------------------------------------------------
void DelayConfig()
{
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;	// включение модуля DWT
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
DWT->CYCCNT = 0;
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
     int i,j; 
     Buff_Send__Index = 0;//Buffer_Number -1;
     Buff_Write_Index=0;
     Flug_Buff = -3;
     for (i=0; i < Buffer_Number; i++) {
//       memset(Data_Buffer[i].buffer, 0, Buffer_Size);
			 for (j=0; j < Buffer_Size; j++)
					Data_Buffer[i].buffer[j] = 'f';//7 + 48;//7 just for test! need to be 0? + 48 - makes it ascii '7'
       Data_Buffer[i].ready=1;//change to 0
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




/*******************************************************************************
* Function Name  : RTC_Configuration
* Description    : Configures the RTC.
* Input          : None.
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_Configuration(void)
{
  /* Configure LSE as RTC clock source */
  RST_CLK_LSEconfig(RST_CLK_LSE_ON);
  /* Wait till LSE is ready */
  while (RST_CLK_LSEstatus() != SUCCESS)
  {
  }

  /* Select the RTC Clock Source */
  BKP_RTCclkSource(BKP_RTC_LSEclk);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();

  /* Sets the RTC prescaler */
  BKP_RTC_SetPrescaler(RTC_PRESCALER_VALUE);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();

  /* Sets the RTC calibrator */
  BKP_RTC_Calibration(RTC_CalibratorValue);
  /* Wait until last write operation on RTC registers has finished */
  BKP_RTC_WaitForUpdate();

  /* Enable the RTC Clock */
  BKP_RTC_Enable(ENABLE);

  /* Enable the Second interrupt */
  BKP_RTC_ITConfig(BKP_RTC_IT_SECF, ENABLE);
  NVIC_EnableIRQ(BACKUP_IRQn);
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
		
		/* Configure PORTB pins 15 (DE pin tranceiver) as output */ 			
    PortInit.PORT_Pin = (PORT_Pin_15);
	  PortInit.PORT_OE    = PORT_OE_OUT;
	  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	  PortInit.PORT_SPEED = PORT_SPEED_FAST;
    PORT_Init(MDR_PORTB, &PortInit);

}



void Uart1Setup(void)
{
	  /* Select HSI/2 as CPU_CLK source*/
  //RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0); // conflict with main CLK setup
    /* Enables the CPU_CLK clock on UART1 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART1, ENABLE);
    /* Set the HCLK division factor = 1 for UART1*/
	/* Set the HCLK division factor = 2 for UART1*/
	UART_BRGInit(MDR_UART1, UART_HCLKdiv1);

    UART_InitStructure.UART_BaudRate                = 115200; // not applied! - Why??? 
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_ON;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

	  /* Configure UART1 parameters*/
	UART_Init (MDR_UART1,&UART_InitStructure);

	
	NVIC_EnableIRQ(UART1_IRQn);
	
	  /* Enable Receiver interrupt*/
  UART_ITConfig (MDR_UART1, UART_IT_RX, ENABLE);
	
	    /* Enables UART1 peripheral */
  UART_Cmd(MDR_UART1,ENABLE);
}

// no need
//void Uart1SendByte(uint16_t Data) //void UART_SendData(MDR_UART_TypeDef* UARTx, uint16_t Data)
//{
//	

//			  UART_SendData (MDR_UART1, Data);
////				while (UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET)
////				{
////				}

//}

 

int UartStart (void)
{
//  uint16_t DataByte;
//	static uint16_t; ReciveByte[16];
//	int i;

	Uart1PinCfg();
	Uart1Setup();
							

	sendbyte = 'r';	
	PORT_SetBits(MDR_PORTB, PORT_Pin_7);
	
	UART_SendData (MDR_UART1, sendbyte);
	UART_SendData (MDR_UART1, sendbyte+1);
	UART_SendData (MDR_UART1, sendbyte+2);
	
	UART_SendData (MDR_UART1, sendbyte);
	UART_SendData (MDR_UART1, sendbyte+1);
	UART_SendData (MDR_UART1, sendbyte+2);
	
	UART_SendData (MDR_UART1, sendbyte);
	UART_SendData (MDR_UART1, sendbyte+1);

 
	return 0;
}


void MltPinCfg (void)
{
	/* PORTA ext interrupt PA0*/// for milndr eval board
	/* Fill PortInit structure*/
//    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
//    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
//    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
//    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
//    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
	/* Configure PORTA pins 0..7 for mlt inout data  */
//	PortInit.PORT_Pin   = PORT_Pin_0;
//	PortInit.PORT_OE    = PORT_OE_IN;
//	PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
//	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
//	PortInit.PORT_SPEED = PORT_SPEED_SLOW; //what does it means? maybe power connected to port?

//	PORT_Init(MDR_PORTA, &PortInit);
	
		/* PORTB ext interrupt PB11*/ // for kodiz board
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
	/* Configure PORTB pins 11 for mlt inout data  */
	PortInit.PORT_Pin   = PORT_Pin_11;
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW; //what does it means? maybe power connected to port?

	PORT_Init(MDR_PORTB, &PortInit);
	
	
	/* PORTC ext interrupt PC12*/ // for kodiz board
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
	/* Configure PORTC pins 12 for mlt inout data  */
	PortInit.PORT_Pin   = PORT_Pin_12;
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

	PORT_Init(MDR_PORTC, &PortInit);
	
	
	/* PORTC ext interrupt PC13*/ // for kodiz board
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
	/* Configure PORTC pins 13 for mlt inout data  */
	PortInit.PORT_Pin   = PORT_Pin_13;
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_ALTER;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

	PORT_Init(MDR_PORTC, &PortInit);
	
	
	
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
		PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5);
		PortInit.PORT_OE    = PORT_OE_IN;
		PortInit.PORT_MODE  = PORT_MODE_ANALOG;
  PORT_Init(MDR_PORTD, &PortInit);


// Инициализация ножек PB14 и PB15 на выход
	// Ножка PB15 поднимается в начале функции EXT_INT2_IRQHandler, в этой же функции после запускается АЦП1
	// Ножка PB14 поднимается в начале функции EXT_INT4_IRQHandler, в этой же функции после запускается АЦП2
	
			/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
	
	/* Configure PORTB pins 13 ( pins for ADC and uart timing debug) as output */ 			
    PortInit.PORT_Pin = (PORT_Pin_13|PORT_Pin_14|PORT_Pin_15);
	  PortInit.PORT_OE    = PORT_OE_OUT;
	  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PORT_Init(MDR_PORTB, &PortInit);

		
}

/*******************************************************************************
* Function Name  : Timer_init
* Description    : Configure the ADC1 for temperature sensor reading.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Timer_init(void)
{
	MDR_RST_CLK->PER_CLOCK |= 1 << 14; /** разрешение тактирования MDR_TIMER1 */
	MDR_RST_CLK->PER_CLOCK |= 1 << 16; /** разрешение тактирования MDR_TIMER3 */
	MDR_RST_CLK->TIM_CLOCK =( /** плак-смайл */
	0 /** делитель тактовой частоты MDR_TIMER1 */
	| (1 << 24) /** разешение тактирования MDR_TIMER1 */
	| (0 << 16) /** делитель тактовой частоты MDR_TIMER3 */
	| (1 << 26) /** разешение тактирования MDR_TIMER3 */
	);

	/** Режим захвата (для тестовой ноги с кнопкой) */
	MDR_TIMER1->CNTRL = 0x00000000; /** Режим инициализации таймера */

	/** Настраиваем работу основного счетчика */
	MDR_TIMER1->CNT = 0x00000000; /** Начальное значение счетчика */
	MDR_TIMER1->PSG = 0x00000000; /** Предделитель частоты */
	MDR_TIMER1->ARR = 0x000000FF; /** Основание счета */
	MDR_TIMER1->IE = 1<<8;
	
	/** Режим работы каналов - захват */
	MDR_TIMER1->CH4_CNTRL = 0x00008003;
	MDR_TIMER1->CNTRL = 0x00000001; /** Счет вверх по TIM_CLK. Разрешение работы таймера */
	NVIC_EnableIRQ(Timer1_IRQn);
}


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
  // ADC_DeInit();
  ADC_StructInit(&sADC);
	sADC.ADC_StartDelay = 0;
  ADC_Init (&sADC);

  ADCx_StructInit (&sADCx);
  sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_SINGLE_CONV;//ADC_SAMPLING_MODE_CICLIC_CONV
  sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
  sADCx.ADC_ChannelNumber    = acd_ch;//ADC_CH_ADC5;
  sADCx.ADC_Channels         = 0;
  sADCx.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;//ADC_LEVEL_CONTROL_Enable
  sADCx.ADC_LowLevel         = 0x800;//L_Level noise level
  sADCx.ADC_HighLevel        = 0x900;//H_Level
  sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
  sADCx.ADC_Prescaler        = ADC_CLK_div_8;//ADC_CLK_div_32768;
  sADCx.ADC_DelayGo          = 0x0;
  ADC1_Init (&sADCx);
	
	//uint32_t tmpreg_CFG = MDR_ADC->ADC2_CFG;

  /* Enable ADC2 EOCIF and AWOIFEN interrupts */
  ADC1_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), ENABLE);

}

void ADC2_Config(uint32_t acd_ch)
{
  	
	/* ADC Configuration */
  /* Reset all ADC settings */
  // ADC_DeInit();
  ADC_StructInit(&sADC);
	sADC.ADC_StartDelay = 0;
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
  sADCx.ADC_DelayGo          = 0x0;
  ADC2_Init (&sADCx);
	
	//uint32_t tmpreg_CFG = MDR_ADC->ADC2_CFG;

  /* Enable ADC2 EOCIF and AWOIFEN interrupts */
  ADC2_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), ENABLE);

}

/*******************************************************************************
* Function Name  : ADC_Temp_Sensor_Config
* Description    : Configure the ADC1 for temperature sensor reading.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Temp_Sensor_Config(void)
{
  /* Enable the RTCHSE clock on ADC1 */
  RST_CLK_PCLKcmd((RST_CLK_PCLK_ADC), ENABLE);

  /* ADC Configuration */
  /* Reset all ADC settings */
  // ADC_DeInit();

  ADC_StructInit(&sADC);
  sADC.ADC_TempSensor           = ADC_TEMP_SENSOR_Enable;
  sADC.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Enable;
  sADC.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Enable;
  ADC_Init (&sADC);
	
  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CICLIC_CONV;
  sADCx.ADC_ChannelNumber    = ADC_CH_TEMP_SENSOR;
  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_EXACT;
  sADCx.ADC_Prescaler        = ADC_CLK_div_32;//ADC_CLK_div_32768;
  sADCx.ADC_DelayGo          = 0x0;
  ADC1_Init (&sADCx);
	
	
}

/*******************************************************************************
* Function Name  : ADC_Temp_Sensor_Config
* Description    : Configure the ADC1 for temperature sensor reading.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SendUARTBuffer(void)
{
	/* Send Data from UART1 */
	// блокирующе пропалываем бит
	
	if (Data_Buffer[Buff_Send__Index].ready){
		
		while (MDR_UART1->FR & UART_FR_TXFE){	
			// push maximum number bytes to uart
			UART_SendData(MDR_UART1, Data_Buffer[Buff_Send__Index].buffer[sendingIndex]);									
			
			if(sendingIndex < Buffer_Size)
				sendingIndex++;
			else // next buffer
			{
				Data_Buffer[Buff_Send__Index].ready=0;
				
				if (Buff_Send__Index < Buffer_Number) 
					Buff_Send__Index++;
				else 	
					Buff_Send__Index = 0;						
				
				sendingIndex=0;
			}
		}
	}
}

/**@brief Setup 16MHz oscillator and PLL */
void SetupExternalOscillator()
{
    /* Set RST_CLK to default */
    RST_CLK_DeInit();

    /* Enable HSE */
    RST_CLK_HSEconfig(RST_CLK_HSE_ON);

    while (RST_CLK_HSEstatus() != SUCCESS);

    /* CPU_C1_SEL = HSE */
    RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1, RST_CLK_CPU_PLLmul5);
    RST_CLK_CPU_PLLcmd(ENABLE);
    if (RST_CLK_CPU_PLLstatus() != SUCCESS){
        while (1){/* Trap */}
    }

    /* CPU_C3_SEL = CPU_C2_SEL */
    //RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);

    /* CPU_C2_SEL = PLL */
    RST_CLK_CPU_PLLuse(ENABLE);

    /* HCLK_SEL = CPU_C3_SEL */
    RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);
		

}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main (void)
{
	// it delay helps to start debugging - pragram cannot block jtag. But need to delete in production!!!	it works ~ 5 s in debug mode
   Delayms(400);
	
	// источник для CPU_C1=HSE, С2=PLL, C3=C2, HCLK = C3;
	// MDR_RST_CLK->CPU_CLOCK=0x00000106;
	
	/* Enables the High Speed External clock */
//	 RST_CLK_HSEconfig(RST_CLK_HSE_ON);
//    while (RST_CLK_HSEstatus() != SUCCESS);
  
   SetupExternalOscillator();  
	
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
	
	//RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER2, ENABLE);

	
	MltPinCfg ();
	
	//Timer_init();
	
	// RTC_Configuration();
	
	ADC_DeInit();
	
	ADC1_Config(ADC_CH_ADC2);//pd2 detector 1
	ADC2_Config(ADC_CH_ADC3);//pd3 detector 2
	
  /* ADC2 enable */
	ADC1_Cmd (ENABLE);
  ADC2_Cmd (ENABLE);	
	


// PLL config test
//  while(1){		
//		PORT_SetBits(MDR_PORTB, PORT_Pin_15);
//		PORT_ResetBits(MDR_PORTB, PORT_Pin_15);	
//	}


	//NVIC_EnableIRQ(EXT_INT1_IRQn);	
	NVIC_EnableIRQ(EXT_INT2_IRQn);	// pc12 detector 2
	//NVIC_EnableIRQ(EXT_INT3_IRQn);	
	NVIC_EnableIRQ(EXT_INT4_IRQn);  // pc13 detector 1
	
  // NVIC_SetPriority (EXT_INT1_IRQn, 1); // установил приоритет
	DelayConfig();
	
	Data_Buffer_Reset();
	
	UartStart(); //основные моменты с инициализацией Uart
	
	
	
 	while(1)
	{
		SendUARTBuffer();
		//tmp = MDR_UART1->FR;
		// не блокирующе пропалываем бит окончания передачи
		if( !(MDR_UART1->FR & UART_FR_BUSY) ){					
			PORT_ResetBits(MDR_PORTB, PORT_Pin_7);
			PORT_ResetBits(MDR_PORTB, PORT_Pin_15);
		}
		
		
									
		// detector 1 
		if(ext_IT2_flag != ext_IT2_flag_previous)//why iterrupt pending when program starts?
			{	
				tmp  = 0;
				tmp2 = 0;
		
				while (!ADC_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION)); //always make troubles
				tmp = ADC1_GetResult(); //adc1 -> pd2


				while (!ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION));//eternal cicle program
				// tmp2 =  ADC2_GetFlagStatus(ADCx_FLAG_END_OF_CONVERSION);			
				tmp2 = ADC2_GetResult(); //adc1 -> pd2
				
				
				PORT_ResetBits(MDR_PORTB, PORT_Pin_14); // Опускание ножки PB15 после считывания результата с АЦП				
				
				ext_IT2_flag_previous = ext_IT2_flag;
				
					PORT_SetBits(MDR_PORTB, PORT_Pin_7);
					PORT_SetBits(MDR_PORTB, PORT_Pin_15);
					UART_SendData (MDR_UART1, 'I');
					UART_SendData (MDR_UART1, '2');
					UART_SendData (MDR_UART1, ext_IT2_flag);
					UART_SendData (MDR_UART1, tmp>>8);
					UART_SendData (MDR_UART1, tmp);
					UART_SendData (MDR_UART1, tmp2>>8);
					UART_SendData (MDR_UART1, tmp2);
					UART_SendData (MDR_UART1, 0);
					
				NVIC_EnableIRQ(EXT_INT2_IRQn);				
				NVIC_EnableIRQ(EXT_INT4_IRQn);	

			}

		// detector 2
		if(ext_IT4_flag != ext_IT4_flag_previous)//why iterrupt pending when program starts?
			{	
						
				tmp  = 0;
				tmp2 = 0;
				// while (!ADC_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION));//always make troubles
				tmp = ADC1_GetResult(); //adc1 -> pd2


				// while (!ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION));//eternal cicle program
				// tmp2 =  ADC2_GetFlagStatus(ADCx_FLAG_END_OF_CONVERSION);			
				tmp2 = ADC2_GetResult(); //adc1 -> pd2
				
				
				PORT_ResetBits(MDR_PORTB, PORT_Pin_14); // Опускание ножки PB14 после считывания результата с АЦП
				
				ext_IT4_flag_previous = ext_IT4_flag;
				

				PORT_SetBits(MDR_PORTB, PORT_Pin_7);
				PORT_SetBits(MDR_PORTB, PORT_Pin_15);
				UART_SendData (MDR_UART1, 'I');
				UART_SendData (MDR_UART1, '4');
				UART_SendData (MDR_UART1, ext_IT4_flag);
				UART_SendData (MDR_UART1, tmp>>8);
				UART_SendData (MDR_UART1, tmp);
				UART_SendData (MDR_UART1, tmp2>>8);
				UART_SendData (MDR_UART1, tmp2);
				UART_SendData (MDR_UART1, 0);
				
				NVIC_EnableIRQ(EXT_INT2_IRQn);				
				NVIC_EnableIRQ(EXT_INT4_IRQn);
			
		}
		
		
		if( !(MDR_UART1->FR & UART_FR_RXFE) ){
			uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
			uart1_IT_RX_flag = SET;
//			if (uart1_IT_RX_byte){
//				PORT_SetBits(MDR_PORTB, PORT_Pin_7);
//				PORT_SetBits(MDR_PORTB, PORT_Pin_15);
//				UART_SendData (MDR_UART1, 'u');		
//			}
		}
		
		
				 

								 
				if(uart1_IT_RX_flag == SET)
				{
					uart1_IT_RX_flag = RESET;
					
					/* Recive data*/
				  //uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1); // no need - handle in interrupt
					
				  /* Send back*/
				    PORT_SetBits(MDR_PORTB, PORT_Pin_7);
					UART_SendData (MDR_UART1, uart1_IT_RX_byte);// coupon

					
					
					switch (uart1_IT_RX_byte)
					{
						case '2':
							ADC2_SetChannel(ADC_CH_ADC2);
							/* ADC2 enable */
							ADC2_Cmd (ENABLE);
							ADC2_Start();
							while (!ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION));
							tmp = ADC2_GetResult(); // ADC_GetStatus();
							/* Send Data from UART1 */
							UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, tmp>>24);	
							UART_SendData(MDR_UART1, tmp>>16);
							UART_SendData(MDR_UART1, tmp>>8);
							UART_SendData(MDR_UART1, tmp);
							break;
						
						case '3':
							ADC1_SetChannel(ADC_CH_ADC3);
							/* ADC1 enable */
							ADC1_Cmd (ENABLE);
							ADC1_Start();
							while (!ADC_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION));
							tmp = ADC1_GetResult(); // ADC_GetStatus();
							/* Send Data from UART1 */			
							UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, tmp>>24);	
							UART_SendData(MDR_UART1, tmp>>16);
							UART_SendData(MDR_UART1, tmp>>8);
							UART_SendData(MDR_UART1, tmp);
							break;
						
						case '4':
							ADC2_SetChannel(ADC_CH_ADC4);
							/* ADC2 enable */
							ADC2_Cmd (ENABLE);
							ADC2_Start();
							while (!ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION));
							tmp = ADC2_GetResult(); // ADC_GetStatus();
							/* Send Data from UART1 */			
							UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, tmp>>24);	
							UART_SendData(MDR_UART1, tmp>>16);
							UART_SendData(MDR_UART1, tmp>>8);
							UART_SendData(MDR_UART1, tmp);
							break;
						
						case '5':
							ADC1_SetChannel(ADC_CH_ADC5);
							/* ADC1 enable */
							ADC1_Cmd (ENABLE);
							ADC1_Start();
							while (!ADC_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION));
							tmp = ADC1_GetResult(); // ADC_GetStatus();
							/* Send Data from UART1 */			
							UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, tmp>>24);	
							UART_SendData(MDR_UART1, tmp>>16);
							UART_SendData(MDR_UART1, tmp>>8);
							UART_SendData(MDR_UART1, tmp);
							break;
						
						
						case '8':
							PORT_SetBits(MDR_PORTB, PORT_Pin_15);
						  UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
						//PORT_Write(MDR_PORTB, 0xFFFFFFFF);
							break;
						
						
						
						case '9':
							PORT_ResetBits(MDR_PORTB, PORT_Pin_15);
						//	PORT_Write(MDR_PORTB, 0x00000000);
							break;
						
						
						
						case 't':
							tmp = ADC1_GetResult(); // ADC_GetStatus(); 0x12345678;//
							/* Send Data from UART1 */			
							UART_SendData (MDR_UART1, 0);
						  UART_SendData (MDR_UART1, 0);
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, tmp>>24);	
							UART_SendData(MDR_UART1, tmp>>16);
							UART_SendData(MDR_UART1, tmp>>8);
							UART_SendData(MDR_UART1, tmp);
							break;
						
						
						case 'u':
							Data_Buffer[Buff_Send__Index].ready=1;
							break;
							
						case '+':
							adcDelay = adcDelay+1;   
							/* Send Data from UART1 */				
							UART_SendData(MDR_UART1, 'D');			
							UART_SendData(MDR_UART1, '+');					
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, adcDelay>>24);
							UART_SendData(MDR_UART1, adcDelay>>16);
							UART_SendData(MDR_UART1, adcDelay>>8);
							UART_SendData(MDR_UART1, adcDelay);	
							break;	
						case '-':
							if(adcDelay>0) 
								adcDelay = adcDelay-1;  
							/* Send Data from UART1 */			
							UART_SendData(MDR_UART1, 'D');			
							UART_SendData(MDR_UART1, '-');				
							UART_SendData (MDR_UART1, 0);							
							UART_SendData(MDR_UART1, adcDelay>>24);
							UART_SendData(MDR_UART1, adcDelay>>16);
							UART_SendData(MDR_UART1, adcDelay>>8);
							UART_SendData(MDR_UART1, adcDelay);	
							break;				
						
						
						
						default:
							sendbyte = 'r';				
							UART_SendData (MDR_UART1, 0);				
							UART_SendData (MDR_UART1, 0);				
							UART_SendData (MDR_UART1, 0);
							UART_SendData(MDR_UART1, tmp>>24);
							UART_SendData(MDR_UART1, tmp>>16);
							UART_SendData(MDR_UART1, tmp>>8);
							UART_SendData(MDR_UART1, tmp);
							break;
					}
					

				  
				}
				
		}
	
			
			
			
		
	
	
	
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

