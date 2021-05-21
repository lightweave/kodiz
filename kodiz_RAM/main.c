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
#include "string.h"
#include "stdlib.h"
// #include "time.h"// not that time file



#include "MDR32F9Qx_timer.h"


// for RTC
#define RTC_HSE_CLK

#define COUNT_VALUE		0
#define ALARM_VALUE		60
#define PRESC_VALUE_HS	1000000 // for 80 MHz
#define PRESC_VALUE_LS	32000
#define RTCHS_PRESC		RST_CLK_HSIclkDIV16//RST_CLK_HSIclkDIV8

/** @addtogroup __MDR32F9Qx_StdPeriph_Examples MDR32F9Qx StdPeriph Examples
  * @{
  */

/** @addtogroup __MDR32F9Q3_EVAL MDR32F9Q3 Evaluation Board
  * @{
  */

/** @addtogroup UART_Interrupt_93 UART_Interrupt
  * @{
  */

int setreset = 1;

// Модификация в файлы для создания нового варианта программы КОДИЗ
 static uint32_t  Program_flags = 0;
// #define INTERUPT1_ON 0x0001
// #define INTERUPT2_ON 0x0002
// #define INTERUPT3_ON 0x0004
// #define INTERUPT4_ON 0x0008
 
 // Program_flags
 // 0000 0000  0000 0000     0000 0000  0000 0000
 //                                      1 - SenderFull_ON
 //                                          1 - Sending_ON
 #define ADC1_ON 0x0004
 #define ADC2_ON 0x0008
 #define ADCS_check 0x000c //ADC1_ON | ADC2_ON
 
 #define Sending_ON 0x1000
 #define SenderFull_ON 0x00010000
 

// static uint16_t State_of_PortC;
 // Значения пинов должны быть приведены в соответствие с распайкой сигналов, поступающих на порт !!!
 #define Si1_input 0x0001
 #define Si2_input 0x0002
// #define BIT_OF_INTERUPT1 0x0100 //1 << PORT_Pin_13 
// #define BIT_OF_INTERUPT2 0x0200

 #define  Sending_Delay 10 // Задержка выключения передатчика после освобождения буфера
 
 #define  Buffer_Size_Si 16 // размер кольцевого буфера


//uint32_t Si_result, Si_Buffer[Buffer_Size_Si]; // кольцевой буфер на 16 
 uint32_t Count_Si1, Count_Si2, Count_Interrupts_1;
 //uint16_t Put_index, Get_index;
 //uint8_t  Hello_text[20];
// const char * Hello_text1 = "Hello string";
// char Hello_text2[20];
 char Send_buffer[128];
 uint32_t Digital_test[Buffer_Size_Si];
 uint8_t *P_current,*P_Last;
 uint16_t Delay_timer = 100;
 
 uint32_t Seconds = 0;
 uint32_t Old_Second = 0;

int J_ADC = 0;  // индекс элемента массива, куда должно записываться очередное значение. 
int shift = 0;	// переключатель нижнее/верхнее полуслово





struct TKEY {
	uint8_t met1, met2, tip, mode;
	// uint32_t code;
}; //UKEY;
struct TKEY UKEY;

struct Tcounts {
	// uint32_t M[30];
	uint32_t 
	si11,          si12,       si21,        si22,       si_coins,
	Cher1,         Cher2,      SiPM1,       SiPM2,      Cher_coins_SiPM,
	n11,           n12,        n21,         n22,        ncoins,
	Ph11,          Ph12,       Ph21,        Ph22,       Phcois,
	Sum1,          Sum2,       Sum1coins,   Sum2coins, 	Interupt_Si,
	Interupt_Cher, Interupt_n, Interupt_Ph, el29,       Delta_t;
} ;

struct Flux{
	struct TKEY key;
	uint32_t time;
	struct Tcounts N;
} ;
struct Flux Flux;
struct Flux Flux_send;

struct Tspectr{
	struct TKEY key;
	uint32_t time;
	uint32_t M[30];
};
struct Tspectr Spectr[4];
struct Tspectr Spectr_send;
struct Tspectr ADC_codes;
struct Tspectr ADC_codes_send;


int INTERUPT_MODE=1; // значение по умолчанию - работа с полупроводниками

static uint32_t  INTERUPT_J_ON[5]; 


uint16_t Result_1, Result_2;








/* Private typedef -----------------------------------------------------------
–	Потоки протонов и ядер с Z>1 c энергией больше 30…50 МэВ/нуклон в диапазоне от 101 до 104 частиц/см2 × с.
–	Потоки протонов и ядер с Z>1 c энергией больше 330 МэВ/нуклон в диапазоне от 101 до 103 частиц/см2 × с.
–	Поток тепловых и эпитепловых нейтронов в диапазоне от 101 до 103 нейтронов/см2 × с.
–	Мощность поглощенной дозы заряженных частиц космического излучения в диапазоне от 10-8 до 10-5 Грей/с.

*/
/*
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
#define Buffer_Size 128     // Length, in bytes, of one buffer for data transmission
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
*/
/* Measurements Data  */
/*
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


*/



/*
// ===========================================================================

#define raw_adc_size   64 // for temporary storage of adc counts before filling spectra
#define raw_adc_number  2 // only two avaliable adcs
unsigned int raw_ADC[raw_adc_number][raw_adc_size];
*/
//uint32_t tmpADC;
//uint32_t tmpADC2;
// ===========================================================================
/*
#define Spectrum_size   64  
#define Spectrum_number 4

//Other variant
//For forming of spectra

unsigned short Spectra[Spectrum_number][Spectrum_size]; // Array for accumulation of energy deposition spectrums in detectors.
*/
// ===========================================================================




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






/*******************************************************************************
* Function Name  : EXT_INT1_IRQHandler
* Description    : This function handles EXT_INT1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Comment	 : for silicom semiconductors
*******************************************************************************/
void EXT_INT1_IRQHandler(void)
{
uint16_t R_1=0; 
uint16_t R_2=0;
	int PORTB_STATUS = PORT_ReadInputData(MDR_PORTB);
	
	if(!(PORTB_STATUS&PORT_Pin_11))	return;
		
	Flux.N.Interupt_Si++;
	
	NVIC_DisableIRQ(EXT_INT1_IRQn);  		//  Отключаем прерывание 1
	
	if(PORTB_STATUS&PORT_Pin_14					){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_10)){ //  Здесь должны быть
		R_1  =0x1000;
		Flux.N.si11++;}//si11 детектор 1 порог 1
	if(PORTB_STATUS&PORT_Pin_15						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)){ //  указаны пины
		R_1 |=0x2000;
		Flux.N.si12++;}
	if(PORTB_STATUS&PORT_Pin_13						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_11)){ //  к которым подключены
		R_2  =0x9000;
		Flux.N.si21++;}
	if(PORTB_STATUS&PORT_Pin_12						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_15)){ //  соответствующие сигналы
		R_2 |=0xA000;
		Flux.N.si22++;}
	
	if(R_1 && R_2){// наличие сигнала совпадения
		R_1 |=0x4000; 
		R_2 |=0x4000;
		Flux.N.si_coins++;
	} 

		
//	NVIC_DisableIRQ(EXT_INT1_IRQn);  		//  Отключаем прерывание 1
//	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_10)){ //  Здесь должны быть
//		R_1  =0x1000;
//    Flux.N.si11++;} 
//	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)){ //  указаны пины
//		R_1 |=0x2000;
//		Flux.N.si12++;} //si12 детектор 1 порог 2
//	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_11)){ //  к которым подключены
//		R_2  =0x9000;
//		Flux.N.si21++;} //si21 детектор 2 порог 1
//	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_15)){ //  соответствующие сигналы
//		R_2 |=0x2000;
//		Flux.N.si22++;} //si22 детектор 2 порог 2

	Program_flags |= INTERUPT_J_ON[1]; //  Устанавливаем признак        
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if(INTERUPT_MODE != 1) return; //
          Result_1=R_1;
          Result_2=R_2;
/*
	Таким образом мы заготавливаем коды по каждому из каналов в которых отмечено:
	какой из порогов превышен и было ли совпадение с сигналом другого детектора.
*/
        if(Result_1){
					ADC1_Start(); // Запускаем АЦП1
					Program_flags |= ADC1_ON;} //  Устанавливаем признак
        if(Result_2){
					ADC2_Start(); // Запускаем АЦП2
					Program_flags |= ADC2_ON;} //  Устанавливаем признак
}
// =============================================================================


/*******************************************************************************
* Function Name  : EXT_INT2_IRQHandler
* Description    : This function handles EXT_INT2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Comment	 : for FEU and SiPM detectors
*******************************************************************************/
void EXT_INT2_IRQHandler(void)
{
uint16_t R_1=0; 
uint16_t R_2=0;
 Flux.N.Interupt_Cher++;
	NVIC_DisableIRQ(EXT_INT2_IRQn);  		//  Отключаем прерывание 2
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_10)){ //  Здесь должны быть
		R_1  =0x1000;
		Flux.N.Cher1++;}
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)){ //  указаны пины
		R_1 |=0x2000;
		Flux.N.Cher2++;}
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_11)){ //  к которым подключены
		R_2  =0x9000;
		Flux.N.SiPM1++;}
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_15)){ //  соответствующие сигналы
		R_2 |=0xA000;
		Flux.N.SiPM2++;}
	
	if(R_1 && R_2){
		R_1 |=0x4000;
		R_2 |=0x4000;
		Flux.N.Cher_coins_SiPM++;} // наличие сигнала совпадения
  Program_flags |= INTERUPT_J_ON[2]; //  Устанавливаем признак        
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if(INTERUPT_MODE != 2) return; //
          Result_1=R_1;
          Result_2=R_2;
/*
	Таким образом мы заготавливаем коды по каждому из каналов в которых отмечено:
	какой из порогов превышен и было ли совпадение с сигналом другого детектора.
*/
        if(Result_1){
          ADC1_Start(); // Запускаем АЦП1
          Program_flags |= ADC1_ON;} //  Устанавливаем признак
        if(Result_2){
          ADC2_Start(); // Запускаем АЦП2
          Program_flags |= ADC2_ON;} //  Устанавливаем признак
}
// =============================================================================


/*******************************************************************************
* Function Name  : EXT_INT3_IRQHandler
* Description    : This function handles EXT_INT3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Comment	 : for neutron FEU
*******************************************************************************/
void EXT_INT3_IRQHandler(void)
{
uint16_t R_1=0; 
uint16_t R_2=0;
 Flux.N.Interupt_n++;
	NVIC_DisableIRQ(EXT_INT3_IRQn);  		//  Отключаем прерывание 3
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_10)){ //  Здесь должны быть
		R_1  =0x1000;
		Flux.N.n11++;}
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)){ //  указаны пины
		R_1 |=0x2000;
		Flux.N.n12++;}
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_11)){ //  к которым подключены
		R_2  =0x9000;
		Flux.N.n21++;}
	if(PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_15)){ //  соответствующие сигналы
		R_2 |=0xA000;
		Flux.N.n22++;}
	
	if(R_1 && R_2) {
		R_1 |=0x4000; 
		R_2 |=0x4000;
	  Flux.N.ncoins++;} // наличие сигнала совпадения
  Program_flags |= INTERUPT_J_ON[3]; //  Устанавливаем признак        
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if(INTERUPT_MODE != 3) return; //
          Result_1=R_1;
          Result_2=R_2;
/*
	Таким образом мы заготавливаем коды по каждому из каналов в которых отмечено:
	какой из порогов превышен и было ли совпадение с сигналом другого детектора.
*/
        if(Result_1){
          ADC1_Start(); // Запускаем АЦП1
          Program_flags |= ADC1_ON;} //  Устанавливаем признак
        if(Result_2){
          ADC2_Start(); // Запускаем АЦП2
          Program_flags |= ADC2_ON;} //  Устанавливаем признак
}
// =============================================================================


/*******************************************************************************
* Function Name  : EXT_INT4_IRQHandler
* Description    : This function handles EXT_INT4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Comment	 : for silicom Photomultipliers
*******************************************************************************/
void EXT_INT4_IRQHandler(void)
{
//		test = PORT_ReadInputData(MDR_PORTB);
//	PORT_SetBits(MDR_PORTB, PORT_Pin_15); // тест по ножкам	
//		test = PORT_ReadInputData(MDR_PORTB);
//	PORT_ResetBits(MDR_PORTB, PORT_Pin_15);	
//		test = PORT_ReadInputData(MDR_PORTB);
		
	int PORTC_STATUS = PORT_ReadInputData(MDR_PORTC);
	
	if(!(PORTC_STATUS&PORT_Pin_13))return;

  uint16_t R_1=0; 
  uint16_t R_2=0;
	
  Flux.N.Interupt_Ph++;
	
	NVIC_DisableIRQ(EXT_INT4_IRQn);  		//  Отключаем прерывание 4
	if(PORTC_STATUS&PORT_Pin_5						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_10)){ //  Здесь должны быть
		R_1  =0x1000;
		Flux.N.Ph11++;}
	if(PORTC_STATUS&PORT_Pin_7						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_14)){ //  указаны пины
		R_1 |=0x2000;
		Flux.N.Ph12++;}
	if(PORTC_STATUS&PORT_Pin_3						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_11)){ //  к которым подключены
		R_2  =0x9000;
		Flux.N.Ph21++;}
	if(PORTC_STATUS&PORT_Pin_4						){		//PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_15)){ //  соответствующие сигналы
		R_2 |=0xA000;
		Flux.N.Ph22++;}
	
	if(R_1 && R_2){
		R_1 |=0x4000;
		R_2 |=0x4000;
		Flux.N.Phcois++;} // наличие сигнала совпадения
  Program_flags |= INTERUPT_J_ON[4]; //  Устанавливаем признак
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if(INTERUPT_MODE != 4) return; //
          Result_1=R_1;
          Result_2=R_2;
/*
	Таким образом мы заготавливаем коды по каждому из каналов в которых отмечено:
	какой из порогов превышен и было ли совпадение с сигналом другого детектора.
*/
        if(Result_1){
          ADC1_Start(); // Запускаем АЦП1
          Program_flags |= ADC1_ON;} //  Устанавливаем признак
        if(Result_2){
          ADC2_Start(); // Запускаем АЦП2
          Program_flags |= ADC2_ON;} //  Устанавливаем признак
}
// =============================================================================
void EnableINTERUPT(int I){
switch (I){
	case 1:
		NVIC_EnableIRQ(EXT_INT1_IRQn);    // Bключаем прерывание 1
		break;
	case 2:
		NVIC_EnableIRQ(EXT_INT2_IRQn);    // Bключаем прерывание 2
		break;
	case 3:
		NVIC_EnableIRQ(EXT_INT3_IRQn);    // Bключаем прерывание 3
		break;
	case 4:
		NVIC_EnableIRQ(EXT_INT4_IRQn);    // Bключаем прерывание 4
		break;
}
}
// =============================================================================
// сжатие 12 бит в 30 каналов
int convers(uint32_t X) {

	uint16_t J= (uint16_t) X & 0x0fff;
	if(J == 0x0fff) return 29;
	if(J > 4090) return 28;
	if(J < 5) return 0; // нижняя граница преобразования 40 кэВ
	int I, m=J;
	
		for(I=26; I>11;I-=2){ // log scale
			if(J & 0x0800) 
				break;
			J = J << 1;
		}
		if(I<11) return  m-4; // lin scale
		else if(J & 0x0400) I++;
		 return I;
}


// =============================================================================

void Put_to_CODE(uint16_t x) {
	if(J_ADC >29) {
		Program_flags |= SenderFull_ON;
		return;
	}
	uint32_t Z=0;
	Z |=x;
	if(shift){
		Z <<=16; 
		Z &= 0xffff0000; 
		ADC_codes.M[J_ADC] |= Z;
		J_ADC++; 
		shift=0;
	} else {
		ADC_codes.M[J_ADC] = Z;
		shift=1;
	}
}

// =============================================================================
void Put_to_CODE_2(uint16_t x, uint16_t y) {
	if(J_ADC >29) {
		Program_flags |= SenderFull_ON;
		return;
	}
	uint32_t Z=0;
	Z |=y;
	Z <<=16; Z &= 0xffff0000;
	Z |=x;
	ADC_codes.M[J_ADC] = Z;
	J_ADC++;
	shift=0;
//	memcpy(Send_buffer, &ADC_codes, 128);
}







/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
// ===================================================================
//  void Put_to_circular_Buffer (uint32_t Si_result) {
//		Si_Buffer[Put_index]= Si_result; 
//		Put_index++; 
//		Put_index &= 0x000f; //сброс значения на 0 если буфер заполнен 
//}
// ===================================================================


// ===================================================================
// Запуск процесса передачи массива байтов
void Start_Uart_sending(uint8_t * pData, int NData){
   P_current = pData;
   P_Last = P_current + NData;
   PORT_SetBits(MDR_PORTB, PORT_Pin_7); // Переключить канал на передачу 
   Program_flags |= Sending_ON;
   UART_SendData (MDR_UART1,*P_current);
   P_current++;
}
// Передача очередного байта или завершение процесса передачи
void Next_Uart_sending(void){
		/* Check TXFE flag*/
   if(UART_GetFlagStatus (MDR_UART1, UART_FLAG_TXFE)!= SET) return;
   if(P_current<P_Last){
        UART_SendData (MDR_UART1,*P_current);
        P_current++;
        Delay_timer=0;
   } else {
    //    if( Delay_timer< Sending_Delay ) return;
		 if(UART_GetFlagStatus (MDR_UART1, UART_FLAG_BUSY)!= SET)
		 {
			 PORT_ResetBits(MDR_PORTB, PORT_Pin_7); // Переключить канал на прием 
       Program_flags &= ~Sending_ON;
		 }
   }
}

// посылка спектра с 1 до 4
void SpectrSend(int i)
{
		if (i < 1 | i > 4) i = 1;
	
		Spectr_send = Spectr[i-1]; 														// 1) копируем текущий массив в отправочный
		Start_Uart_sending((uint8_t *)&Spectr_send, 128);   // 2) запускаем отправку данных с буферного массива
//		memset(&Spectr[i], 0, sizeof(Spectr[i])); 								  // 3) обнуляем текущий массив						
//		Spectr[i].key  = UKEY; 														// 4) добавляем в него текущую шапку, а нужно ли - ведь она уже заполнена?
//		Spectr[i].key.tip  = i + 1;     									// тип массива, он по идее уже записан
		Spectr[i-1].time = Seconds;														// 5) добавляем в него текущее время		  

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
  // ADC1_ITConfig((ADC1_IT_END_OF_CONVERSION  | ADC1_IT_OUT_OF_RANGE), ENABLE);

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
  // ADC2_ITConfig((ADC2_IT_END_OF_CONVERSION  | ADC2_IT_OUT_OF_RANGE), ENABLE);

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
//void SendUARTBuffer(void)
//{
//	/* Send Data from UART1 */
//	// блокирующе пропалываем бит
//	
//	if (Data_Buffer[Buff_Send__Index].ready){
//		
//		while (MDR_UART1->FR & UART_FR_TXFE){	
//			// push maximum number bytes to uart
//			UART_SendData(MDR_UART1, Data_Buffer[Buff_Send__Index].buffer[sendingIndex]);									
//			
//			if(sendingIndex < Buffer_Size)
//				sendingIndex++;
//			else // next buffer
//			{
//				Data_Buffer[Buff_Send__Index].ready=0;
//				
//				if (Buff_Send__Index < Buffer_Number) 
//					Buff_Send__Index++;
//				else 	
//					Buff_Send__Index = 0;						
//				
//				sendingIndex=0;
//			}
//		}
//	}
//}

// ===================================================================
 void Command_Handler(uint8_t DataByte){
  switch (DataByte) {
			case 't':   
				Start_Uart_sending((uint8_t *)Send_buffer,128);
				break;

			
			// Переключение режимов прерываний для работы с разными детекторами
			case 'v': // for scintillator (silicon Photomultipliers?)
				INTERUPT_MODE = 1;			// нужно обнулить массивы данных и спектры???
				ADC1_Config(ADC_CH_ADC11);//PD12 detector 1
				ADC2_Config(ADC_CH_ADC10);//PD13 detector 2
				break;
			case 'b': // for FEU and SiPM detectors
				INTERUPT_MODE = 2;			  // нужно обнулить массивы данных и спектры???
				break;
			case 'n': // for neutron FEU
				INTERUPT_MODE = 3;			  // нужно обнулить массивы данных и спектры???
				break;
			case 'm': // for silicom semiconductors
				INTERUPT_MODE = 4;			  // нужно обнулить массивы данных и спектры???
				ADC1_Config(ADC_CH_ADC13);//PD10 detector 1
				ADC2_Config(ADC_CH_ADC12);//PD11 detector 2
				break;
			case 'p': // in case of interrupt failture
				NVIC_EnableIRQ(EXT_INT4_IRQn);  // PC13 detector 1
				break;
			
			case 'k': // переключение АЦП на потенциометры для калибровки
				ADC1_Config(ADC_CH_ADC7);//PD7 potentiometr 1
				ADC2_Config(ADC_CH_ADC8);//PD8 potentiometr 2
				break;
			
			case 'a':// вывод массива кодов АЦП
				ADC_codes_send = ADC_codes; 												// 1) копируем текущий массив в отправочный
				Start_Uart_sending((uint8_t *)&ADC_codes_send, 128);// 2) запускаем отправку данных с буферного массива
				memset(&ADC_codes, 0, sizeof(ADC_codes)); 					// 3) обнуляем текущий массив		
				ADC_codes.key  			= UKEY; 												// 4) добавляем в новый текущую шапку
				ADC_codes.key.tip  	= 5;     												// 											тип массива
				ADC_codes.key.mode 	= INTERUPT_MODE;     						// 											режим работы прибора	
				ADC_codes.time 			= Seconds;											// 											текущее время
				break;
			
			case 'f':// вывод массива потоков
				Flux_send = Flux; 															// 1) копируем текущий массив в отправочный
				Start_Uart_sending((uint8_t *)&Flux_send, 128); // 2) запускаем отправку данных с буферного массива
				memset(&Flux, 0, sizeof(Flux)); 								// 3) обнуляем текущий массив						
				Flux.key  		= UKEY; 													// 4) добавляем в него текущую шапку
				Flux.key.tip  = 0;     													// 										 тип массива
				Flux.key.mode = INTERUPT_MODE;     							// 										 режим работы прибора
				Flux.time 		= Seconds;												// 										 текущее время
				break;			
			
			case 's':	// вывод массива статуса
				memcpy(Send_buffer, &UKEY, 4);
				Start_Uart_sending((uint8_t *)Send_buffer,4); // терминал не очень хорошо это ловит, надежнее запрашивать тестовый
				// memcpy(Send_buffer + 8, "", 120);
				break;
			
			case 'w':
				setreset = 0;					  
				break;
			case 'q':
				setreset = 1;					  
				break;
			
			case '1':
				if(setreset)  PORT_SetBits  (MDR_PORTC, PORT_Pin_15);
				else				  PORT_ResetBits(MDR_PORTC, PORT_Pin_15);
				break;
			case '2':
				if(setreset)	PORT_SetBits  (MDR_PORTC, PORT_Pin_14);
				else  			  PORT_ResetBits(MDR_PORTC, PORT_Pin_14);	  
				break;
			case '3':
				if(setreset)  PORT_SetBits  (MDR_PORTC, PORT_Pin_2);
				else 			    PORT_ResetBits(MDR_PORTC, PORT_Pin_2);				  
				break;
			case '4':
				if(setreset)  PORT_SetBits  (MDR_PORTC, PORT_Pin_1);
				else  			  PORT_ResetBits(MDR_PORTC, PORT_Pin_1);		  
				break;
			case '5':
				if(setreset)  PORT_SetBits  (MDR_PORTB, PORT_Pin_9);
				else  				PORT_ResetBits(MDR_PORTB, PORT_Pin_9);
				break;
			case '6':
				if(setreset)  PORT_SetBits  (MDR_PORTB, PORT_Pin_10);
				else  			  PORT_ResetBits(MDR_PORTB, PORT_Pin_10);	  
				break;
			case '7':
				if(setreset)  PORT_SetBits  (MDR_PORTD, PORT_Pin_3);
				else				  PORT_ResetBits(MDR_PORTD, PORT_Pin_3);		  
				break;
			case '8':
				if(setreset)	PORT_SetBits  (MDR_PORTD, PORT_Pin_5);
				else  			  PORT_ResetBits(MDR_PORTD, PORT_Pin_5);				  
				break;
			
				
				
			// Вывод отдельных массивов спектров
//			case '1':
//				SpectrSend(1);	  
//				break;
//			case '2':
//				SpectrSend(2);	  
//				break;
//			case '3':
//				SpectrSend(3);				  
//				break;
//			case '4':
//				SpectrSend(4);		  
//				break;
			
			
			
			}
 }

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
	DWT-> CYCCNT = 0;
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





// не используется 

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
//    PortInit.PORT_Pin = (PORT_Pin_15);
//	  PortInit.PORT_OE    = PORT_OE_OUT;
//	  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
//	  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
//	  PortInit.PORT_SPEED = PORT_SPEED_FAST;
//    PORT_Init(MDR_PORTB, &PortInit);

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

	
	//NVIC_EnableIRQ(UART1_IRQn);
	
	  /* Enable Receiver interrupt*/
  //UART_ITConfig (MDR_UART1, UART_IT_RX, ENABLE);
	
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
	PORT_ResetBits(MDR_PORTB, PORT_Pin_7);// это управляющий пин приемопередатчика, прием данных
 
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
	PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

	PORT_Init(MDR_PORTB, &PortInit);

	
		/* PORTC ext interrupt PB 12,-15*/ // for kodiz board
		/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
		/* Configure PORTB pins  for mlt inout data  */
		PortInit.PORT_Pin   = (PORT_Pin_12|PORT_Pin_13|PORT_Pin_14|PORT_Pin_15);
		PortInit.PORT_OE    = PORT_OE_IN;
		PortInit.PORT_FUNC  = PORT_FUNC_PORT;
		PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
		PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

		PORT_Init(MDR_PORTB, &PortInit);
	/* PORTC ext interrupt PB9,PB10 */ // for prohibit
		/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
		/* Configure PORTC pins 13 for mlt inout data  */
		PortInit.PORT_Pin   = (PORT_Pin_9|PORT_Pin_10);
		PortInit.PORT_OE    = PORT_OE_OUT;
		PortInit.PORT_FUNC  = PORT_FUNC_PORT;
		PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
		PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

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
	
	/* PORTC ext interrupt PC10,11,14,15*/ // for kodiz board
		/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_OPEN; // here we make this port open for tri state, so we can connect more than one inputs to each pin
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
		/* Configure PORTC pins 13 for mlt inout data  */
		PortInit.PORT_Pin   = (PORT_Pin_3|PORT_Pin_4|PORT_Pin_5|PORT_Pin_7|PORT_Pin_10|PORT_Pin_11|PORT_Pin_14|PORT_Pin_15);
		PortInit.PORT_OE    = PORT_OE_IN;
		PortInit.PORT_FUNC  = PORT_FUNC_PORT;
		PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
		PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

		PORT_Init(MDR_PORTC, &PortInit);
		/* PORTC ext interrupt PC1,2, 14 15*/ // for prohibit
		/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER; 
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
		/* Configure PORTC pins 13 for mlt inout data  */
		PortInit.PORT_Pin   = (PORT_Pin_1|PORT_Pin_2|PORT_Pin_14|PORT_Pin_15);
		PortInit.PORT_OE    = PORT_OE_OUT;
		PortInit.PORT_FUNC  = PORT_FUNC_PORT;
		PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
		PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

		PORT_Init(MDR_PORTC, &PortInit);
		
		
	
		/* PORTD ext interrupt PD3,PD5 */ // for prohibit
		/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER; 
    PortInit.PORT_GFEN = PORT_GFEN_OFF; //filter off!
		/* Configure PORTC pins 13 for mlt inout data  */
		PortInit.PORT_Pin   = (PORT_Pin_3|PORT_Pin_5);
		PortInit.PORT_OE    = PORT_OE_OUT;
		PortInit.PORT_FUNC  = PORT_FUNC_PORT;
		PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
		PortInit.PORT_SPEED = PORT_SPEED_MAXFAST; //what does it means? maybe power connected to port?

		PORT_Init(MDR_PORTD, &PortInit);
	
		/* PORTB UART pins*/
		// Uart1PinCfg();



	
		/* Configure PORTD pin 2 It ac brake JTAG */		
		/* Configure ADC pin: PD7, 8 10 11 12 13  */
		PortInit.PORT_Pin   = (PORT_Pin_7 | PORT_Pin_8 | PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 | PORT_Pin_13);
		PortInit.PORT_OE    = PORT_OE_IN;
		PortInit.PORT_MODE  = PORT_MODE_ANALOG;
		PORT_Init(MDR_PORTD, &PortInit);

		PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
		PortInit.PORT_Pin   = ( PORT_Pin_12 | PORT_Pin_13);
		PortInit.PORT_OE    = PORT_OE_IN;
		PortInit.PORT_MODE  = PORT_MODE_ANALOG;
		PORT_Init(MDR_PORTD, &PortInit);
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
		// Инициализация ножек PB14 и PB15 на выход
		// Ножка PB15 поднимается в начале функции EXT_INT2_IRQHandler, в этой же функции после запускается АЦП1
		// Ножка PB14 поднимается в начале функции EXT_INT4_IRQHandler, в этой же функции после запускается АЦП2
	
	  /* Fill PortInit structure*/
//    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
//    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
//    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
//    PortInit.PORT_PD = PORT_PD_DRIVER;
//    PortInit.PORT_GFEN = PORT_GFEN_OFF;
//    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
//    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
//    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
//	
//	  /* Configure PORTB pins 13 ( pins for ADC and uart timing debug) as output */ 			
//    PortInit.PORT_Pin = (PORT_Pin_13|PORT_Pin_14|PORT_Pin_15);
//	  PortInit.PORT_OE    = PORT_OE_OUT;
//	  PortInit.PORT_FUNC  = PORT_FUNC_PORT;
//	  PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
//	  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
//    PORT_Init(MDR_PORTB, &PortInit);




//		/* Configure all unused PORTF pins to low power consumption */
//		PORT_StructInit(&PortInit);
//		PortInit.PORT_Pin = (PORT_Pin_All & ~(PORT_Pin_0 | PORT_Pin_1));
//		PORT_Init(MDR_PORTF, &PortInit);


//		/* Configure PORTF pins 0..1 for output to switch LEDs on/off */
//		PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);
//		PortInit.PORT_OE    = PORT_OE_OUT;
//		PortInit.PORT_FUNC  = PORT_FUNC_PORT;
//		PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
//		PortInit.PORT_SPEED = PORT_SPEED_SLOW;

//		PORT_Init(MDR_PORTF, &PortInit);
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
* Function Name  : SetupRTC
* Description    : Configure the RTC for .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetupRTC(void)
{
	/* Enables the HSI clock on PORTF */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);

  /* Configure all unused PORT pins to low power consumption */
//  PORT_StructInit(&PORT_InitStructure);
//  PORT_InitStructure.PORT_Pin = (PORT_Pin_All & ~(PORT_Pin_0 | PORT_Pin_1));
//  PORT_Init(MDR_PORTF, &PORT_InitStructure);


  /* Configure PORTF pins 0..1 for output to switch LEDs on/off */
//  PORT_InitStructure.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);
//  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
//  PORT_InitStructure.PORT_FUNC  = PORT_FUNC_PORT;
//  PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
//  PORT_InitStructure.PORT_SPEED = PORT_SPEED_SLOW;

//  PORT_Init(MDR_PORTF, &PORT_InitStructure);

  /* Enables the HSI clock for BKP control */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP,ENABLE);

  /* RTC reset */
  BKP_RTC_Reset(ENABLE);
  BKP_RTC_Reset(DISABLE);

#ifdef RTC_HSI_CLK
  /* Configure RTCHSI as RTC clock source */
  RST_CLK_HSIadjust(25);
  RST_CLK_RTC_HSIclkEnable(ENABLE);
  RST_CLK_HSIclkPrescaler(RTCHS_PRESC);
  BKP_RTCclkSource(BKP_RTC_HSIclk);
#endif
#ifdef RTC_HSE_CLK
  /* Configure RTCHSE as RTC clock source */
  RST_CLK_HSEconfig(RST_CLK_HSE_ON);
  while (RST_CLK_HSEstatus()!=SUCCESS);
  RST_CLK_RTC_HSEclkEnable(ENABLE);
  RST_CLK_HSEclkPrescaler(RTCHS_PRESC);
  BKP_RTCclkSource(BKP_RTC_HSEclk);
#endif
#ifdef RTC_LSI_CLK
  /* Configure LSI as RTC clock source */
  RST_CLK_LSIadjust(12);
  BKP_RTCclkSource(BKP_RTC_LSIclk);
  while (RST_CLK_LSIstatus()!=SUCCESS);
#endif

  /* Enable all RTC interrupts */
  BKP_RTC_ITConfig(BKP_RTC_IT_ALRF | BKP_RTC_IT_SECF | BKP_RTC_IT_OWF,ENABLE);
  NVIC_EnableIRQ(BACKUP_IRQn);

  /* Set the RTC counter value */
  BKP_RTC_WaitForUpdate();
  BKP_RTC_SetCounter(COUNT_VALUE);

  /* Set the RTC prescaler value */
  BKP_RTC_WaitForUpdate();
#ifdef RTC_HSI_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_HS);
#endif
#ifdef RTC_HSE_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_HS);
#endif
#ifdef RTC_LSI_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_LS);
#endif

  /* Set the RTC alarm value */
  BKP_RTC_WaitForUpdate();
  BKP_RTC_SetAlarm(ALARM_VALUE);

  /* RTC enable */
  BKP_RTC_WaitForUpdate();
  BKP_RTC_Enable(ENABLE);
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
/*******************************************************************************
* Function Name  : BACKUP_IRQHandler
* Description    : This function handles BACKUP global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BACKUP_IRQHandler(void)
{
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

if (BKP_RTC_GetFlagStatus(BKP_RTC_FLAG_SECF)==SET)
  {
		Seconds++;
//		if (PORT_ReadInputDataBit(MDR_PORTF,PORT_Pin_0)==0)
//    {			
//      PORT_SetBits(MDR_PORTF,PORT_Pin_0);
//    }
//	else
//    {
//      PORT_ResetBits(MDR_PORTF,PORT_Pin_0);
//    }
  }
  if (BKP_RTC_GetFlagStatus(BKP_RTC_FLAG_ALRF)==SET)
  {
    //PORT_SetBits(MDR_PORTF,PORT_Pin_1);
  }
  MDR_BKP -> RTC_CS |= 0x06;
}

void test_init_Tcounts(struct Tcounts *  tcounts)
{	 
//	 tcounts.M = 0;

	 
}

///**
//  * @brief  Main program.
//  * @param  None
//  * @retval None
//  */
// ===================================================================
 int main (void)
{
/*

Здесь начальная инициализация всех требуемых устройств и т.д.

*/
	// it delay helps to start debugging - program cannot block jtag. But need to delete in production!!!	it works ~ 5 s in debug mode
  Delayms(400);
	
	// источник для CPU_C1=HSE, С2=PLL, C3=C2, HCLK = C3;
	// MDR_RST_CLK->CPU_CLOCK=0x00000106;
	
	/* Enables the High Speed External clock */
//	 RST_CLK_HSEconfig(RST_CLK_HSE_ON);
//    while (RST_CLK_HSEstatus() != SUCCESS);
  
  SetupExternalOscillator();  
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE); 	/* Enables the clock on PORTA */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);	/* Enables the clock on PORTB */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);  /* Enables the clock on PORTC */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);	/* Enables the clock on PORTD */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);	/* Enables the clock on PORTE */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);	/* Enables the HSI clock on PORTF */

  RST_CLK_PCLKcmd(RST_CLK_PCLK_EBC, ENABLE);		/* Enables the HSI clock on ExtBus */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC, ENABLE);		/* Enables the ADC clock on ExtBus */
  //RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER2, ENABLE);

	
	MltPinCfg();
	
	PORT_ResetBits(MDR_PORTB, PORT_Pin_9);
	PORT_ResetBits(MDR_PORTB, PORT_Pin_10);
	
	PORT_ResetBits(MDR_PORTC, PORT_Pin_1);
	PORT_ResetBits(MDR_PORTC, PORT_Pin_2);
	PORT_ResetBits(MDR_PORTC, PORT_Pin_14);
	PORT_ResetBits(MDR_PORTC, PORT_Pin_15);
	
	PORT_ResetBits(MDR_PORTD, PORT_Pin_3);
	PORT_ResetBits(MDR_PORTD, PORT_Pin_5);
	
	SetupRTC();
	
	//Timer_init();
	
	// RTC_Configuration();
	
	
	/* Enable ADC interrupt  */
  NVIC->ISER[0] = (1<<ADC_IRQn);
	
	ADC_DeInit();
	
	ADC1_Config(ADC_CH_ADC11);//PD12 detector 1
	ADC2_Config(ADC_CH_ADC10);//PD13 detector 2
	
  /* ADC2 enable */
	ADC1_Cmd (ENABLE);
  ADC2_Cmd (ENABLE);	
	


// PLL config test
//  while(1){		
//		PORT_SetBits(MDR_PORTB, PORT_Pin_15);
//		PORT_ResetBits(MDR_PORTB, PORT_Pin_15);	
//	}



	
  // NVIC_SetPriority (EXT_INT1_IRQn, 1); // установил приоритет
	DelayConfig();
	
	
	UartStart(); // основные моменты с инициализацией Uart
	
		
	uint32_t tmp;
//		Put_index=0; Get_index=0;
//		for(int i=0; i<Buffer_Size_Si; i++) {Si_Buffer[i]=0; Digital_test[i]=i;}
	 
//		strcpy( Hello_text2 , "Hello srt test");
	 


	// =============================================================================
	// Фрагмент main()

	 //создадим тестовый struct Tcounts
//	 struct Tcounts tcounts;
	 
	 // шапка  приветственный кадр со списком команд
	 UKEY.met1 = 0xCC;  // метка
	 UKEY.met2 = 0x55;  // метка
	 UKEY.tip  = 16;    // тип массива
	 UKEY.mode = 1;     // режим работы прибора
	 
	 // копирование первой шапки
	 memcpy(Send_buffer, &UKEY, 4);
	 memcpy(Send_buffer + 8, "   KODIZPROGRAM V.0.6   AVALUABLE CMDs: (t)est; (z); (a)(f)lux; (s)pectrINTERRUPT MODES: N1 (v); N2 (b)  N3 (n); N4 (m) ", 120);
   // memcpy(Send_buffer, "KODIZ   SINP MSUPROGRAM v.0.6   AVAILABLE CMDs: (t)est; (z); (a)(f)lux; (s)pectrINTERRUPT MODES: N1 (v); N2 (b)  N3 (n); N4 (m) ", 128);
//		case 't'//Send_buffer, 128
//		case 'z'//Digital_test, Buffer_Size_Si
//		// Переключение режимов прерываний для работы с разными детекторами

//		case 'v'//INTERUPT_MODE = 1;
//		case 'b'//INTERUPT_MODE = 2;
//		case 'n'//INTERUPT_MODE = 3;
//		case 'm'//INTERUPT_MODE = 4;				

//		case 'a'// вывод массива кодов АЦП
//		case 'f'// вывод массива потоков
//		case 's'// вывод массива спектра





	 // шапка  массива выдачи информации по умолчанию 
	 UKEY.met1 = 0xCC;  // метка
	 UKEY.met2 = 0x55;  // метка
	 UKEY.tip  = 0;     // тип массива - массив потоков и доз
	 UKEY.mode = 1;     // режим работы прибора - подключение к полупроводникам
		
 


		
	// структура FLUX пример начальной инициализации	
	Flux.key  		= UKEY; 
	Flux.key.tip 	= 0;
	Flux.key.mode = INTERUPT_MODE;
	Flux.time 		= Seconds;
//	Flux.N    		= tcounts; // это вообще что??
	 
		
// инициализация спектров правильными шапками
	for(int i=0; i<=3; i++){
		Spectr[i].key = UKEY; 
		Spectr[i].key.tip = i + 1;  // 1-4 спектры
		Spectr[i].key.mode = INTERUPT_MODE;
		Spectr[i].time = Seconds;
		for(int j=0; j<30; j++) 
			Spectr[i].M[j]=0;
	}
	
  INTERUPT_J_ON[0]=0x0000; 
	INTERUPT_J_ON[1]=0x0100;
	INTERUPT_J_ON[2]=0x0200;
	INTERUPT_J_ON[3]=0x0400;
	INTERUPT_J_ON[4]=0x0800;
 
 
	// структура ADC_codes пример начальной инициализации
	ADC_codes.key      = UKEY;
	ADC_codes.key.tip  = 5; 
	ADC_codes.key.mode = INTERUPT_MODE;
	ADC_codes.time     = Seconds;
	for(int i=0; i<30; i++) // кривой сброс информации
		ADC_codes.M[i]=0;
	
	J_ADC=0;  // индекс элемента массива, куда должно записываться очередное значение. Одновременно критерий заполнености и готовности массива к передаче.
	shift=0;	// переключатель нижнее/верхнее полуслово





	
	
	/* Рабочий цикл, в котором висит */	 //=================================================================================================
	


	Program_flags = 0;
	
	 NVIC_EnableIRQ(EXT_INT1_IRQn);	
	NVIC_EnableIRQ(EXT_INT2_IRQn);	// PC12 detector 2
//	 NVIC_EnableIRQ(EXT_INT3_IRQn);	
	NVIC_EnableIRQ(EXT_INT4_IRQn);  // PC13 detector 1
	
	Start_Uart_sending((uint8_t *)Send_buffer, 128); // отправка нулевого кадра, в котором записана шапка и команды

     while (1)
     {
			 for(int interrupt_J=1; interrupt_J<5;interrupt_J++){
				 if(interrupt_J != INTERUPT_MODE){ //это часть работает тогда, когда к этому прерыванию не подключено АЦП
					 if(Program_flags & INTERUPT_J_ON[interrupt_J]){
						 // Добавить контроль того, что сигнал прерывания закончился
						 
//							 Program_flags &= ~INTERUPT_J_ON[INTERUPT_J];
//							 EnableINTERUPT(INTERUPT_J);
						 
						 if(interrupt_J != 4){ 														// если у нас не 4, то делаем стандартное включение прерывания
							 Program_flags &= ~INTERUPT_J_ON[interrupt_J];		// (нужно сделать подобное для остальных)
							 EnableINTERUPT(interrupt_J);
						 } else{ 																					// если у нас 4, то
							 int test = PORT_ReadInputData(MDR_PORTC); 			// считываем полностью порт С
							 if(!(test&0x2000)){														// проверяем значение порта и включаем прерывание
								 Program_flags &= ~INTERUPT_J_ON[interrupt_J];
								 EnableINTERUPT(interrupt_J);
							 }}
					 }
				 }else {// когда к этому прерыванию  АЦП подключено
					 // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
					 if(Program_flags & ADC1_ON){
						 if(ADC_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION)){
							 tmp =0x00000fff & ADC1_GetResult();
							 Result_1 |=  tmp;
							 Flux.N.Sum1 += tmp; 
							 int k=convers(tmp);
							 Spectr[0].M[k]++;
							 
							 if(Result_1 & 0x4000){
								 Flux.N.Sum1coins += tmp; 
								 Spectr[2].M[k]++;
							 }
							 Program_flags &= ~ADC1_ON; //  Снимаем признак
							 }
					 }
					
					 if(Program_flags & ADC2_ON){
						 if(ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION)){
							 tmp =0x00000fff & ADC2_GetResult();
							 Result_2 |=  tmp;
							 Flux.N.Sum2 += tmp;  
							 int k=convers(tmp);
							 Spectr[1].M[k]++;
							 
							 if(Result_2 & 0x4000) {
								 Flux.N.Sum2coins += tmp; 
								 Spectr[3].M[k]++;
							 }	
							 Program_flags &= ~ADC2_ON; //  Снимаем признак
							 }
					 }
					 if(Program_flags & SenderFull_ON){
						 if(ADC_codes.time != Seconds){// посылка не чаще чем 1 раз в секунду
							 if(!(Program_flags & Sending_ON)) {
								 
								 memcpy(Send_buffer, &ADC_codes, 128); 							// 1) копируем текущий массив в отправочный		
								 Start_Uart_sending((uint8_t *)Send_buffer,128);			// 2) запускаем отправку данных по ADC_codes
								 //memset(&ADC_codes, 0, sizeof(ADC_codes)); 					// 3) обнуляем текущий массив						
									ADC_codes.key  			= UKEY; 												// 4) добавляем в новый текущую шапку
									ADC_codes.key.tip  	= 5;     												// 											тип массива
									ADC_codes.key.mode 	= INTERUPT_MODE;     						// 											режим работы прибора	
									ADC_codes.time 			= Seconds;											// 											текущее время
					
								 J_ADC = 0;
								 Program_flags &= ~SenderFull_ON;
						 }
						 }
					 }
					 
					 if((!(Program_flags & ADCS_check)) &&  // Нет не прочитанных данных АЦП
						 (Program_flags & INTERUPT_J_ON[interrupt_J])) {// Признак обработки последствий прерывания не снят
							 Program_flags &= ~INTERUPT_J_ON[interrupt_J];  //  Снимаем признак
							 if(Result_1 & 0x4000) {
								 
															// тест 1: срабатывание совпадений -> выдает 2+2 байта (по 2 на каждый канал)
					//											Res_tmp1++;
					//											Put_to_CODE(Res_tmp1);
					//									//----	
															
					//									// тест 2: нет совпадений -> выдает 2 байта по одному из каналов
					//										Res_tmp1++;
					//										//Res_tmp2++;
					//										if(Res_tmp1 ) Put_to_CODE(Res_tmp1); 
					//										if(Res_tmp2 ) Put_to_CODE(Res_tmp2); 
															//----
															
														//	Put_to_CODE(Result_1); 
															Put_to_CODE_2(Result_1, Result_2); // НУЖНОЕ
															}
															else { // сюда заходит если не было совпадения
															if(Result_1 ) Put_to_CODE(Result_1); 
															if(Result_2 ) Put_to_CODE(Result_2); 
														}
														EnableINTERUPT(interrupt_J);   // Bключаем прерывание 
										}

										// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
										
						 }  // else
	}   // for


// =============================================================================

/*
			 
        // ========== Обработка данных полупроводниковых детекторов ==========
        if(Program_flags & ADC1_ON){
          if(ADC_GetFlagStatus(ADC1_FLAG_END_OF_CONVERSION)){
            tmp = ADC1_GetResult();
            Si_result |=  tmp & 0x00000fff;
            Program_flags &= ~ADC1_ON; //  Снимаем признак
          }
          if(!(Program_flags & ADCS_check)){ // Нет не прочитанных данных АЦП
            Put_to_circular_Buffer (Si_result);
            Program_flags &= ~INTERUPT1_ON;  //  Снимаем признак
          }
        }
        if(Program_flags & ADC2_ON){
          if(ADC_GetFlagStatus(ADC2_FLAG_END_OF_CONVERSION)){
            tmp = ADC2_GetResult();
            Si_result |=  (tmp << 16) & 0x0fff0000;
            Program_flags &= ~ADC2_ON; //  Снимаем признак
          }
          if(!(Program_flags & ADCS_check)){ // Нет не прочитанных данных АЦП
            Put_to_circular_Buffer (Si_result);
            Program_flags &= ~INTERUPT1_ON;  //  Снимаем признак
          }
        }
        if((!(Program_flags  & INTERUPT1_ON)) &     // Когда все действия, связанные с прерыванием закончены
					
				   (!(State_of_PortC & BIT_OF_INTERUPT1)))// И сигнал прерывания тоже закончился					
				//!PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_13))  

              NVIC_EnableIRQ(EXT_INT4_IRQn);    // Bключаем прерывание 4
                // Добавить проверку, что прерывание уже не включено
        // ===== Конец обработки данных полупроводниковых детекторов ======
				
				*/
				
				
				
				// ===== Обработка следующей секунды = ===============
				if(Old_Second != Seconds)
				{
					Old_Second = Seconds;
					if(Seconds%10 == 0){
						Command_Handler('f');
					}
//					if(Seconds%(10*60) == 0){
//						Command_Handler(1);						
//					}
//					if(Seconds%(10*60+1) == 0){
//						Command_Handler(2);						
//					}
//					if(Seconds%(10*60+2) == 0){
//						Command_Handler(3);						
//					}
//					if(Seconds%(10*60+3) == 0){
//						Command_Handler(4);						
//					}
				}
			
				
				
				
				
        // ===== Обработка принятого по UART байта = ===============
			if( !(MDR_UART1->FR & UART_FR_RXFE) ){
					uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
					uart1_IT_RX_flag = SET;
				  Command_Handler(uart1_IT_RX_byte);
					uart1_IT_RX_byte = 0;
			}
				
				//	if ((PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3)) &
				//	UART_GetFlagStatus (MDR_UART1, UART_FLAG_RXFF)== SET )
				//	if( !(MDR_UART1->FR & UART_FR_RXFE) ){
					uart1_IT_RX_byte = UART_ReceiveData(MDR_UART1);
				
					// uart1_IT_RX_flag = SET;
          
				

				
				
				
				
				
        // ====== Передача очередного байта по UART  ===============
        if(Program_flags & Sending_ON)   Next_Uart_sending();

     }  // End  while (1)
}



/*******************************************************************************
* Function Name  : EXT_INT1_IRQHandler
* Description    : This function handles EXT_INT1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void EXT_INT4_IRQHandler(void)
//{
//	NVIC_DisableIRQ(EXT_INT4_IRQn);  // Отключаем прерывание 1
//        State_of_PortC =      PORT_ReadInputData(MDR_PORTC);          // Read Port C
//				int tmp0 = PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_0);
//				int tmp1 = PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_1);
//				int tmp2 = PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2);
//        if(State_of_PortC & Si1_input){
//          ADC1_Start(); // Запускаем АЦП1
//          Program_flags |= ADC1_ON; //  Устанавливаем признак
//          Count_Si1++;
//        }
//        //if(State_of_PortC & Si2_input){
//          ADC2_Start(); // Запускаем АЦП2
//          Program_flags |= ADC2_ON; //  Устанавливаем признак
//          Count_Si2++;
//        //}
//        if(Program_flags & ADCS_check){
//          Si_result= (ADCS_check <<28) & 0xf0000000; // Заготовка для кодов АЦП
//          Program_flags |= INTERUPT1_ON; //  Устанавливаем признак
//        }
//        Count_Interrupts_1 ++;
//}
// ===================================================================

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

