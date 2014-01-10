

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"


void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
unsigned int tick_count;
  
void SysTick_Handler(void)
{
	tick_count++;	

	if (tick_count == 0xFFFFFFF0)
		reset_timer();

	//clr_pu1_low();		/* biar tahu kalau tick sempat keluar */
}

unsigned int get_tick_count()
{
	return tick_count;
}

void reset_tick_count()
{
	tick_count = 0;
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
#include "mainku.h"

void togle_led(void);
unsigned int spi_count;
static int toggle_tim3;
int data_adc;
//int lagi_int;

#if (BUF_ADC_PAKAI_POINTER == 1)
extern float *buf_data_adc[ JUM_KANAL ];	/* ada di main.c */
#else
#if (BUF_ADC_SRAM_LUAR == 1)
//extern float buf_data_adc[ JUM_KANAL ][SIZE_BUF_ADC] __attribute__ ((section (".sram_luar")));
//extern float buf_data_adc2[ JUM_KANAL ][SIZE_BUF_ADC] __attribute__ ((section (".sram_luar")));
extern float buf_data_adc[ NUM_BUF_ADC ][ JUM_KANAL ][ SIZE_BUF_ADC ] __attribute__ ((section (".sram_luar")));

#else
extern float buf_data_adc[ JUM_KANAL ][SIZE_BUF_ADC];
#endif
#endif


int count_data_adc;
int total_count_adc;
int count_buf_adc;

float pecahan_32 = (float) (2.5 / 2147483391); // 2,147,483,392 dikurangi 1 ??	

int spi_kanal;
int cur_kanal;

static int jalankan_adc = 0;

void set_jalankan_adc()
{
	/* disable sistick dulu, TIDAK PERLU !! */
	//NVIC_DisableIRQ(SysTick_IRQn);
	
	#if (ADC_START_STOP == 1)
	/* Enable external interrupt oleh DRDY ADC */
	NVIC_EnableIRQ(EXTI0_IRQn);
	sss
	#endif
	
	jalankan_adc = 1;
}

void unset_jalankan_adc()
{
	jalankan_adc = 0;
}

int get_jalankan_adc()
{
	return jalankan_adc;
}

/* ini adalah interrupt ADC ready, kemudian mengaktifkan SPI 
 * 
 * 26 Des 2013
 * interrupt oleh GPIO akan start timer dan kemudian 
 * SPI DMA sebanyak 12 spi
 * 
 * Jika DMA selesai, interrupt lagi untuk stop timer
 * 
 * */
void EXTI0_IRQHandler(void)
{  
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {	
	if (jalankan_adc)
	{					
		/* DMA ENABLE */
		DMA_Cmd(DMA1_Stream2, ENABLE);			
		
		/* Reset NSS pin internally by software */
		SPI3->CR1 &= SPI_NSSInternalSoft_Reset;
		
		/* enable SPI3 */
		SPI3->CR1 |= (uint16_t) SPI_CR1_SPE;	
		
		#if (TIMER_START_STOP == 1)
		/* enable timer 3 */
		TIM3->CR1 |= TIM_CR1_CEN; 	
		#endif
	}
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

//extern unsigned char 	ADC_ConvertedValue[24];	// 31kS/s
//extern unsigned int 	ADC_ConvertedValue[6];	/* aslinya char */
extern unsigned short 	ADC_ConvertedValue[6];

void DMA1_Stream2_IRQHandler(void)
{
	__disable_irq();
	
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2)) 
	{
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);				
			
		#if (TIMER_START_STOP == 1)
		/* timer 3 juga disable */
		TIM3->CR1 &= ~TIM_CR1_CEN; 
		#endif 
		
		/* SPI NSS HIGH */
		SPI3->CR1 |= SPI_NSSInternalSoft_Set;
		
		/* ngatasi bug SPI DMA RX 
		 * Entah kenapa kalau SPI-DR dibaca
		 * data jadi normal dan bagus
		 * 
		 * */
		short aa = SPI3->DR;
		
		/* Disable SPI3 */
		SPI3->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);	
		
		{		
			#if 1				
			/* kanal 1 */
			int e_adc = (ADC_ConvertedValue[0] << 16) | (ADC_ConvertedValue[ 1 ] & 0xFF00);
			buf_data_adc[ count_buf_adc ][ 0 ][ count_data_adc ] = e_adc;
			
			/* kanal 2 */
			e_adc = (ADC_ConvertedValue[1] << 24) | (ADC_ConvertedValue[2] << 8);  		
			//buf_data_adc[ 1 ][ count_data_adc ] = (float) (e_adc * pecahan_32);
			buf_data_adc[ count_buf_adc ][ 1 ][ count_data_adc ] = e_adc;
			
			/* kanal 3 */
			e_adc = (ADC_ConvertedValue[3] << 16) | (ADC_ConvertedValue[ 4 ] & 0xFF00);
			buf_data_adc[ count_buf_adc ][ 2 ][ count_data_adc ] = e_adc;
			
			/* kanal 4 */
			e_adc = (ADC_ConvertedValue[4] << 24) | (ADC_ConvertedValue[ 5 ] << 8);
			buf_data_adc[ count_buf_adc ][ 3 ][ count_data_adc ] = e_adc;	
			#endif
		}
		
		//total_count_adc++;
		count_data_adc++;
		if (count_data_adc >=  SIZE_BUF_ADC) 
		{
			count_data_adc = 0;	
			total_count_adc++;	/* dipindahkan kesini, sehingga untuk sampling rate perlu dikali dengan size block */	
			
			count_buf_adc++;
			if (count_buf_adc >= NUM_BUF_ADC)
			count_buf_adc = 0;
			
			#if (ADC_START_STOP == 1)
			/* mode start stop */	
			jalankan_adc = 0;					
			/* disable external interrupt oleh DRDY ADC */
			NVIC_DisableIRQ(EXTI0_IRQn);
			#endif
		}
	}

	__enable_irq();
}

#if 0
/* SPI 3 */
void SPI3_IRQHandler(void)
{	
	//char aa;
	short aa;
	if (SPI_GetITStatus(SPI3, SPI_I2S_IT_RXNE) != RESET)
  	{
		spi_count++;
		
		printf("s");
		/* Set NSS pin internally by software */
		//SPI3->CR1 |= SPI_NSSInternalSoft_Set;
		

		aa = SPI3->DR;
		aa = (short) (aa & 0x00FF);
		data_adc = (int) ((data_adc << 8) | aa);

		if (spi_kanal == 3)
		{
			buf_data_adc[cur_kanal][ count_data_adc ] = (float) (data_adc * pecahan_32);

			cur_kanal++;
			if (cur_kanal == 4) cur_kanal = 0;
			
			spi_kanal = 0;
			data_adc = 0;
		}

		spi_kanal++;
		
		/*	
		if (spi_count < 12)		// 12, 3 per kanal
		{
			//aa = SPI3->DR;
			//data_adc = (int) ((data_adc << 8) | aa);
			//data_adc = aa;
		}
		else
		*/
		//if (spi_count == 12)
		
		if (spi_count >= 12)
		{
			//SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_RXNE, DISABLE);
			SPI_Cmd(SPI3, DISABLE);
			
			/* timer 3 juga disable */
			TIM3->CR1 &= ~TIM_CR1_CEN;   
			
			/* HIGH */
			SPI3->CR1 |= SPI_NSSInternalSoft_Set;
			
			count_data_adc++;
			if (count_data_adc > SIZE_BUF_ADC ) 
			{
				count_data_adc = 0;
				jalankan_adc = 0;
			}
			
			total_count_adc++;
		}
		
		/* Reset NSS pin internally by software */
		//SPI3->CR1 &= SPI_NSSInternalSoft_Reset;
			
		SPI_I2S_ClearITPendingBit( SPI3, SPI_I2S_IT_RXNE );
	}
}
#endif

/* ditambahkan 6 Des 2013 */
extern struct t_buf_serial buf_serial;
void USART3_IRQHandler(void)
{
	if( USART_GetITStatus(USART3, USART_IT_RXNE) )
	{
		char t = USART3->DR; // the character from the USART1 data register is saved in t
				
		buf_serial.rx_buf[ buf_serial.rx_in ] = t;
		buf_serial.rx_in++;
		
		//printf("kar\r\n");
	}
}

