/* 
	1 agt 2012

	coba2 SPI

	SPI1 ==> ENC1 pooling
	SPI2 ==> ENC2 pooling
	SPI3 ==> ADC interrupt 
	* 
	
	26 Des 2013
	Test SPI ADC dengan DMA
	
	

*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#include "./include/mainku.h"


// ini di dapat dari :
//	http://eliaselectronics.com/stm32f4-tutorials/stm32f4-spi-tutorial/

// this function initializes the SPI1 peripheral
void init_SPI1(void)
{
	debug_entry;
		
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	/* Configure the chip select pin
	   in this case we will use PE7 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 high
	
	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	//SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	// gagal /* speed hanya 14 MHz */
	//SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct); 
	
	SPI_Cmd(SPI1, ENABLE); // enable SPI1

	debug_leave;
}

/* CILIWUNG */
void init_SPI2(void)
{
	debug_entry;
	
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
	/* configure pins used by SPI2
	 * PI1 = SCK
	 * PI2 = MISO
	 * PI3 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOI, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource1, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource2, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource3, GPIO_AF_SPI2);

	// enable peripheral clock
	//RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);		// SPI2 pada APB1, lihat di rcc driver stm32f4xx_spi.c
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);		// hati2 nama fungsi
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI2, &SPI_InitStruct); 
	
	SPI_Cmd(SPI2, ENABLE); // enable SPI2

	debug_leave;
}



#define 	SPI3_DR_ADDRESS ((uint32_t) 0x40003C0C)	/* lihat halaman 709 dan table 1 hal 50 */  
unsigned short 	ADC_ConvertedValue[7];	// 24

void init_SPI3_slave(void)
{
	debug_entry;
		
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	SPI_I2S_DeInit(SPI3);
	
	/* disable untuk reset dulu */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, DISABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/* configure pins used by SPI3

		Ingat, untuk slave .. MOSI jadi data input, MISO jadi data output
	
	 * PC10 = SCK
	 * PC11 = MISO
	 * PC12 = MOSI	// mungkin untuk ADC tidak diperlukan
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | /*GPIO_Pin_11 |*/ GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// bisa juga 50 Hz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);				
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	//GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
	/* enable peripheral clock SPI3 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	/* MULAI URUSAN DMA */
	/* SPI3_RX bisa di DMA1, Channel 0, Stream 0 atau Stream 2*/
	DMA_InitTypeDef       DMA_InitStructure;
	
	/* DMA CLOCK Enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	DMA_StructInit( &DMA_InitStructure );
	 
	DMA_DeInit(DMA1_Stream0); // disable stream 0
	while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE)
	{
	}
   
	DMA_DeInit(DMA1_Stream2); 
	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE)
	{
	}
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;	  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) SPI3_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = &ADC_ConvertedValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 6;	// 12 = 4 kanal * 3 byte (24 bit)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //DMA_PeripheralDataSize_Byte
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //DMA_MemoryDataSize_Byte; 
	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	/* biar auto stop */
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	//DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;	// 28 kS/s ok
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // 30 kS/s OK       
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //DMA_PeripheralBurst_INC4;
	
	DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	/* DMA SELESAI */
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	 SPI_StructInit( &SPI_InitStruct );
#if 1
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_RxOnly;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave; 
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b; // one packet of data is 8 bits wide
	
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle	30 ks/S OK
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at second edge
	
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI3, &SPI_InitStruct); 
#endif

	#if 0  
	/* SPI Interrupt */
  	NVIC_InitTypeDef   NVIC_InitStructure;

  	NVIC_InitStructure.NVIC_IRQChannel = SPI3_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	/* interrupt SPI RX */
	SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_RXNE, ENABLE);
  	#endif
	
	#if 1
	/* DMA 12 byte end Interrupt */
  	NVIC_InitTypeDef   NVIC_InitStructure;

  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;			
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	/* enable interrupt DMA ketika penuh stream 0 */	
	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE ); 	// hati2 dengan DMA_IT_TC ini ... 
	#endif
	
	/* enable DMA dilakukan di interrupt */
	/* enable SPI_RX DMA request */
	SPI_I2S_DMACmd(SPI3,  SPI_I2S_DMAReq_Rx, ENABLE);

	debug_leave;
}



/* This funtion is used to transmit and receive data 
 * with SPI1
 * 			data --> data to be transmitted
 * 			returns received value
 */
uint8_t SPI1_send(uint8_t data)
{

	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	//while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	//while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	//return SPI1->DR; // return received data from SPI data register
}

uint8_t SPI1_send_read(uint8_t data)
{

	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI1->DR; // return received data from SPI data register
}

uint8_t SPI2_send_read(uint8_t data)
{

	SPI2->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI2->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI2->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI2->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI2->DR; // return received data from SPI data register
}

uint8_t SPI3_send_read(uint8_t data)
{

	SPI3->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI3->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI3->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI3->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI3->DR; // return received data from SPI data register
}

#if 0
int main(void){
	
	uint8_t received_val = 0;
	
	init_SPI1();
	
	while(1){
		
		GPIOE->BSRRH |= GPIO_Pin_7; // set PE7 (CS) low
		SPI1_send(0xAA);  // transmit data
		received_val = SPI1_send(0x00); // transmit dummy byte and receive data
		GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 (CS) high
	}
} 
#endif
