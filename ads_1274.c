/*
	28 Juni 2013
	ADS 1274 dengan STM32F4 Discovery

	Furkan Jadid
	Daun Biru Engineering

	TIM10/11 maks 84 MHz ==> tidak bisa input
	TIM9 jg 84 MHz, lihat hal 422

	10 Juli 2013
	============
	- ADS1274 mode SPI
	- SPI mode slave, dengan clock diperoleh dari TIM
	- TIM diclock external dari master clock ADS1274
	- TIM akan men-devide menjadi 1/2 atau 1/4 dari master clock
	- DRDY akan interupt STM, start timer, timer output akan clock SPI
	- SCLK SPI akan start stop berdasarkan timer

	30 Des 2013
	===========
	- mode SPI dengan interrupt, terbatas pada samplingrate 20 kS/s
	- itu sudah seperti interrupt 20 kS/s * (1 (tim start) + 12) (3byte * 4 kanal) = 260.000 interrupt per detik.
	- dengan MODE DMA, nampaknya juga masih salah pada format data. 
	- mode DMA akan perlu 20 kS/s * ( 1 (tim start) + 1 DMA Stop ) = 40.000 interrupt per detik
	- coba mode SSI

	SKENARIO MODE SSI
	- TIM diclock external = master clock ADS1274
	- TIM akan devide master clock menjadi SCLK SSI (Continues) 
	- TIM yang lain, mendapat masukan dari SCLK untuk membuat FSYNC ke ADC
	- SSI akan jalan dengan DMA
	- harusnya tidak perlu interrupt lagi 
	- pin FORMAT2 perlu dikasih resistor pemrogram juga
	
	HIGH RES : Mode_0 = 1, Mode_1 = 0, CLKDIV = 1. CLOCK = 512x output, oversampling 128x
	*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_exti.h"

void EXTILine0_Config(void);

void init_gpio_ads()
{
	GPIO_InitTypeDef gpio_sync, gpio_pwdn, gpio_clkdiv, gpio_mode1;
	GPIO_InitTypeDef gpio_clr_pu1;
	
	/* GPIO untuk SYNC PD13 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  	gpio_sync.GPIO_Pin = GPIO_Pin_13;
  	gpio_sync.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_sync.GPIO_OType = GPIO_OType_PP;
  	gpio_sync.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_sync.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOD, &gpio_sync);	

	/* GPIO untuk PWDN PB6 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  	gpio_pwdn.GPIO_Pin = GPIO_Pin_6;
  	gpio_pwdn.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_pwdn.GPIO_OType = GPIO_OType_PP;
  	gpio_pwdn.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_pwdn.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOB, &gpio_pwdn);	

	/* GPIO untuk CLKDIV
	 * mesti diset HIGH untuk dapat 512x clock
	 * 
	 * CLKDIV : PE3
	 * MODE 0 : PE2
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  	gpio_clkdiv.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;
  	gpio_clkdiv.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_clkdiv.GPIO_OType = GPIO_OType_PP;
  	gpio_clkdiv.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_clkdiv.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOE, &gpio_clkdiv);
  	
  	/* MODE 1, PI7 */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

  	gpio_mode1.GPIO_Pin = GPIO_Pin_7;
  	gpio_mode1.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_mode1.GPIO_OType = GPIO_OType_PP;
  	gpio_mode1.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_mode1.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOI, &gpio_mode1);
  	
	/* CLR_PU1 PC4 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  	gpio_clr_pu1.GPIO_Pin = GPIO_Pin_4;
  	gpio_clr_pu1.GPIO_Mode = GPIO_Mode_OUT;
  	gpio_clr_pu1.GPIO_OType = GPIO_OType_PP;
  	gpio_clr_pu1.GPIO_Speed = GPIO_Speed_100MHz;
  	gpio_clr_pu1.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOC, &gpio_clr_pu1);

	/* external interrupt DRDY */
	EXTILine0_Config();
	
}

/*
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
	
	PA0 sebagai data ready ADS1274
	DRDY akan LOW jika data siap diambil
	akan kembali HIGH saat clock SPI masuk

  */
void EXTILine0_Config(void)
{
  
  	GPIO_InitTypeDef   GPIO_InitStructure;
  	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

  	/* Enable GPIOA clock */
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  	/* Enable SYSCFG clock */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  	/* Configure PA0 pin as input floating */
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	/* Connect EXTI Line0 to PA0 pin */
  	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  	/* Configure EXTI Line0 */
  	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // mestinya falling tapi kayaknya terlalu cepat
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

void sync_high()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
}

void sync_low()
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
}

void pwdn_high()
{
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
}

void pwdn_low()
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

void clkdiv_high()
{
	GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

void clkdiv_low()
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
}

void mode0_high()
{
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
}

void mode0_low()
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
}

void mode1_high()
{
	GPIO_SetBits(GPIOI, GPIO_Pin_7);
}

void mode1_low()
{
	GPIO_ResetBits(GPIOI, GPIO_Pin_7);
}

//HIGH RES : Mode_0 = 1, Mode_1 = 0, CLKDIV = 1. CLOCK = 512x output, oversampling 128x
void set_high_res_ads1274()
{
	mode0_high();
	mode1_low();
	clkdiv_high();
}

void clr_pu1_high()
{
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
}

void clr_pu1_low()
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}
