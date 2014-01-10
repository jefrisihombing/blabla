#ifndef __MAINKU__
#define __MAINKU__

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#include "./enc/enc424j600.h"

#define FIRMWARE_REV	"1.0"
#define PCB_REV			"Ciliwung 1.9"

#define BUF_ADC_PAKAI_POINTER	0	/* jika 0, berarti langsung pakai section, lihat di ld */
#define BUF_ADC_SRAM_LUAR		1	/* jika 1, berarti memakai SRAM diluar chip */
#define NUM_BUF_ADC				10

#define ADC_START_STOP			0	/* jika 1, ADC distart, tunggu data cukup kemudian stop */
#define TIMER_START_STOP		1	/* jika 1, timer untuk ADC start stop */

/* lokasi SRAM 512 kB */
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)  

#if (ADC_START_STOP == 1)
#define SIZE_BUF_ADC	1200	/* tadinya 1024 */
ddss
#else
#define SIZE_BUF_ADC	256	/*1024 disamakan dengan buffer ethernet ? */
#endif

#define JUM_KANAL		4

//#define F_HZ_TICK				10000u	/* periode tick = 0.1 ms */
#define F_HZ_TICK				100u
//#define DELAY_SERIAL			10u
//#define DELAY_CONTROL_GPIO		5u
//#define DELAY_ETHER			10u
//#define DELAY_BLINK			5000u	// 0.5 detik
//#define DELAY_ARP				100000u	// 10 detik
//#define DELAY_KALIB			2000u 	// ingat frekuensi adalah 10 kHz

#define DELAY_SERIAL			2u
#define DELAY_CONTROL_GPIO		2u
#define DELAY_ETHER				2u
#define DELAY_BLINK				50u		// 0.5 detik
#define DELAY_ARP				1000u	// 10 detik
#define DELAY_CONFIG_FLASH		(DELAY_BLINK * 120)		// 1 menit
#define DELAY_KALIB				20u

/* setting LWIP */
#define JUM_UDP_PCB				20


void delay_loop(int loop);
uint8_t SPI1_send_read(uint8_t data);
uint8_t SPI2_send_read(uint8_t data);
uint8_t SPI3_send_read(uint8_t data);

/**************** PORT2 untuk ENC *************/
/**** MUNGKIN AKAN ADA 2 ENC/DUAL ETHERNET ****/
#define spiInit()				init_SPI1()
#define GPIO_ETHER1				GPIOA
#define GPIO_ETHER2				GPIOI

#define GPIO_ETHER_INT1			GPIOC
#define GPIO_ETHER_INT2			GPIOC

#define BIT_INT_ENC1			GPIO_Pin_1	// PC1
#define BIT_INT_ENC2			GPIO_Pin_3	// PC3

#define BIT_CS_ENC1				GPIO_Pin_4	// PA4
#define BIT_CS_ENC2				GPIO_Pin_0	// PI0

//#define BIT_CS_ENC2				GPIO_Pin_3


#define AssertChipSelect()		GPIO_ResetBits(GPIO_ETHER1, BIT_CS_ENC1)  
#define DeassertChipSelect()	GPIO_SetBits(GPIO_ETHER1, BIT_CS_ENC1)

#define AssertChipSelect_2()	GPIO_ResetBits(GPIO_ETHER2, BIT_CS_ENC2)  
#define DeassertChipSelect_2()	GPIO_SetBits(GPIO_ETHER2, BIT_CS_ENC2)

#define spiPut					SPI1_send_read
#define spiPut_2				SPI2_send_read

#define _delay_us				delay_loop_us

#define ENCX24J600_Reset()		GPIO_SetBits(GPIOA, BIT_CS_ENC1)
#define ENCX24J600_Unreset()	GPIO_ResetBits(GPIOA, BIT_CS_ENC1)  

#define portENTER_CRITICAL()	__disable_irq()
#define portEXIT_CRITICAL()		__enable_irq()


/****************** END ENC *******************/


/* MEMS Microphone SPI Interface */
//#define SPI_SCK_PIN                 GPIO_Pin_10

#if 0
#define SPI_SCK_PIN                   GPIO_Pin_13
#define SPI_SCK_GPIO_PORT             GPIOB
#define SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                GPIO_PinSource13	//10
#define SPI_SCK_AF                    GPIO_AF_SPI2

#define SPI_MOSI_PIN                  GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT            GPIOC
#define SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE               GPIO_PinSource3
#define SPI_MOSI_AF                   GPIO_AF_SPI2
#endif

#define debug_entry	 printf("%s(): ___ENTRY___\r\n", __FUNCTION__)
#define debug_leave	 printf(" ___LEAVE___ :%s()\r\n", __FUNCTION__)
#define debug_error	 printf("%s(): ___ERROR___\r\n", __FUNCTION__)

/* struct urusan serial */
struct t_buf_serial {
	char rx_buf[32];
	char tx_buf[32];
	int rx_in;
	int rx_out;
	int tx_in;
	int tx_out;
};



int time_for_serial();
int time_for_gpio();
int time_for_ether();
int time_for_blink();
int time_for_arp();
unsigned int get_tick_count();
void reset_tick_count();
int boleh_tulis_env();

void togle_led(void);

void proses_ethernet(int *loop);
void set_sampling_rate(int s_rate);


int get_jalankan_adc();

void proses_kirim_data(int loop_5);

#endif
