/*
	23 Juni 2013
	Furkan Jadid

	Daun Biru Engineering

	Fungsi2 awal sesaat setelah main dipanggil
	* 
	
	6 Des 2013
	UART3 diberi interupt receive

*/

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "./include/mainku.h"

void RCC_Configuration(void)
{
  	/* deinit semua GPIO dulu */
  	GPIO_DeInit(GPIOA);
  	GPIO_DeInit(GPIOB);
  	GPIO_DeInit(GPIOC);
  	GPIO_DeInit(GPIOD);
  	GPIO_DeInit(GPIOE);
  	GPIO_DeInit(GPIOF);
  	GPIO_DeInit(GPIOG);
  	GPIO_DeInit(GPIOH);
  	GPIO_DeInit(GPIOI);
  	
  	/* --------------------------- System Clocks Configuration -----------------*/
  	/* USART3 clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
 
  	/* GPIOB clock enable */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
}

void GPIO_Configuration(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
 
  	/*-------------------------- GPIO Configuration ----------------------------*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  	/* Connect USART pins to AF */
  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

  	/* init GPIO untuk ENC1, CILIWUNG */
  	/* GPIOD Periph clock enable */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* PORT GPIO MESTI SAMA */
  	GPIO_InitTypeDef gpio_enc;
  	gpio_enc.GPIO_Pin = BIT_CS_ENC1;
  	gpio_enc.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_enc.GPIO_OType = GPIO_OType_PP;
  	gpio_enc.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_enc.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIO_ETHER1, &gpio_enc);

	/* CS ENC2, CILIWUNG */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
  	GPIO_InitTypeDef gpio_enc2;
  	gpio_enc2.GPIO_Pin = BIT_CS_ENC2;
  	gpio_enc2.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_enc2.GPIO_OType = GPIO_OType_PP;
  	gpio_enc2.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_enc2.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIO_ETHER2, &gpio_enc2);


	/* GPIO untuk cek interrupt ENC */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* PORT GPIO MESTI SAMA */
  	GPIO_InitTypeDef gpio_enc_int1;
  	gpio_enc_int1.GPIO_Pin = BIT_INT_ENC1 | BIT_INT_ENC2;
  	gpio_enc_int1.GPIO_Mode = GPIO_Mode_IN;
  	gpio_enc_int1.GPIO_OType = GPIO_OType_PP;
  	gpio_enc_int1.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_enc_int1.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIO_ETHER_INT1, &gpio_enc_int1);

  	
#if 0
 	/* untuk blink saja */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

   	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif
}

void GPIO_Configuration22(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
 
  	/*-------------------------- GPIO Configuration ----------------------------*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  	/* Connect USART pins to AF */
  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

#if 1
  	/* init GPIO untuk ENC */
  	/* GPIOD Periph clock enable */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	/* PORT GPIO MESTI SAMA */
  	GPIO_InitTypeDef gpio_enc;
  	gpio_enc.GPIO_Pin = BIT_CS_ENC1 | BIT_CS_ENC2;
  	gpio_enc.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_enc.GPIO_OType = GPIO_OType_PP;
  	gpio_enc.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_enc.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIO_ETHER1, &gpio_enc);


	/* GPIO untuk cek interrupt ENC */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	/* PORT GPIO MESTI SAMA */
  	GPIO_InitTypeDef gpio_enc_int1;
  	gpio_enc_int1.GPIO_Pin = BIT_INT_ENC1;
  	gpio_enc_int1.GPIO_Mode = GPIO_Mode_IN;
  	gpio_enc_int1.GPIO_OType = GPIO_OType_PP;
  	gpio_enc_int1.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_enc_int1.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIO_ETHER_INT1, &gpio_enc_int1);
  	#endif
}

struct t_buf_serial buf_serial;
 
void USART3_Configuration(void)
{
    memset( &buf_serial, 0, sizeof (buf_serial));
    
    USART_InitTypeDef USART_InitStructure;
 
  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 9600 baud
        - Word Length = 8 Bits
        - Two Stop Bit
        - Odd parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  	USART_InitStructure.USART_BaudRate = 115200;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  	USART_Init(USART3, &USART_InitStructure);
 
	/* uart receive interupt */
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff
	
	/* dienable */
  	USART_Cmd(USART3, ENABLE);
}

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  		GPIO_SetBits(GPIOD, GPIO_Pin_14);	// biar kelihatan delaynya
  }
}

#if 0
void TIM5_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* GPIOC Configuration: TIM5 CH2 (PA1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
  //GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  //GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
}

void TIM3_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  
  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
}

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 333;
uint16_t CCR2_Val = 249;
uint16_t CCR3_Val = 166;
uint16_t CCR4_Val = 83;
uint16_t PrescalerValue = 0;

void TIM5_Jalan()
{
	//printf("System core clock %d\r\n", SystemCoreClock );

	/* Compute the prescaler value */
 // PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
  // printf("Prescale value %d\r\n", PrescalerValue);

  /* Time base configuration */
  //TIM_TimeBaseStructure.TIM_Period = 665;
  //TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	// dapat 42 kHz
  TIM_TimeBaseStructure.TIM_Period = 8;			// period 8, prescale 1, dapat 4.7 MHz
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	#if 1
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  //TIM_OC1Init(TIM5, &TIM_OCInitStructure);

  //TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  #endif

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OCInitStructure.TIM_Pulse = 4;

  TIM_OC2Init(TIM5, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

#if 0
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
#endif
  TIM_ARRPreloadConfig(TIM5, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM5, ENABLE);
}

void TIM3_Jalan()
{
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}
#endif

void init_enc_port()
{
	// sudah di init diatas
}

#if 0
/* config TIM9 untuk terima dari clock /master clock ADC */
void TIM9_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM9 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

  /* GPIOE clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  /* GPIOC Configuration: TIM9 CH2 (PE6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure);  

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
}

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 333;
uint16_t CCR2_Val = 249;
uint16_t CCR3_Val = 166;
uint16_t CCR4_Val = 83;
uint16_t PrescalerValue = 0;

void TIM9_Jalan()
{
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM9, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	

  TIM_ARRPreloadConfig(TIM9, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}
#endif


void TIM3_Config(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;

  	/* TIM3 clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  	/* GPIOC and GPIOB clock enable */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
  
  	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
  	/* Connect TIM3 pins to AF2 */  
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
}

//http://www.farrellf.com/projects/hardware/2012-08-11_STM32F4_Basics:_Timers_(Part_1)/

void TIM3_Jalan()
{
	/* CH1 sebagai input, CH2 sebagai OUTPUT 
		input dari CH1 akan sekitar dibagi 4
		untuk dapat output.

		Kemudian dipakai untuk clocking SPI
		untuk ambil data dari ADS1274

	*/
    TIM3->CCMR1 = 0;
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1;  	// Togle, fail
	
    TIM3->SMCR |= TIM_SMCR_SMS;                         // External clock mode 1
    TIM3->SMCR |= TIM_SMCR_TS_0 | TIM_SMCR_TS_2;        // Trigger selection: TI1
#if 1
	/* set output */
	TIM3->PSC = 0;                                	// Set prescaler to 41999
    TIM3->ARR = 1;                                  // jika, 1 maka seperempatnya
    TIM3->CCR1 = 10;
	TIM3->CR1 &= ~TIM_CR1_OPM;

	TIM3->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC2E;	// falling dan rising detect, output pada ch2
	
	TIM3->EGR |= TIM_EGR_UG | TIM_EGR_CC2G; 
    TIM3->SR &= ~TIM_SR_UIF;                            // Clear the update flag
    #endif  

    /* biar dienable oleh interupt */
    //TIM3->CR1 |= TIM_CR1_CEN;   
}

