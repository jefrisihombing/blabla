// STM32 USART3 (Tx PB.10, Rx PB.11) STM32F4 Discovery - sourcer32@gmail.com
/* dicopy dari 

	1 agt 2012
	printf pakai library kecil

	23 Juni 2013
	Furkan Jadid, Daun Biru engineering

	Tugas utama :
	1. Ambil data dari ADC lewat SPI
	2. Monitor ethernet, mungkin ada request dan balas seperlunya
	3. Monitor serial (modbus)
	4. Kontrol GPIO

	Interupt (bukan sesuai prioritas)
	1. Ethernet (ada paket)
	2. Serial
	3. Systick
	4. Adc ready data

	27 Juni 2013
	============
	INT ENC_1 dipindah dari PB14 ke PC1
	SPI2 dipakai untuk ENC_2
	SPI3 untuk ADC
	OK, SPI interrupt sudah Ok.

	28 Juni 2013
	============
	ADS 1274
	DRDY akan menginterrupt lewat PA0

	9 Juli 2013
	===========
	- floating point dipakai

	10 Juli 2013
	============
	- ADS1274 mode SPI
	- SPI mode slave, dengan clock diperoleh dari TIM
	- TIM diclock external dari master clock ADS1274
	- TIM akan men-devide menjadi 1/2 atau 1/4 dari master clock
	- DRDY akan interupt STM, start timer, timer output akan clock SPI

	12 Jul 2013
	===========
	- SPI slave OK
	- data sudah terbaca di MOSI
	- tinggal dikalibrasi
	
	TODO :
	- jika ENC_1 init tidak ketemu, MAC tetap memakai aslinya ENC_2
	- pindahkan inisiasi ethernet ke file lain
	- ADC akan trigger external interupt, start SPI3, dan jika dapat 4 kanal stop interrupt SPI3
	  dan tunggu lagi ADC ready

	26 Sept 2013
	============
	Adapt ke Ciliwung 1.9

	7 Oktober 2013
	==============
	I2C untuk CLOCK ADC disesuaikan dengan ciliwung

	sebelum dipasang SRAM (regulator + max232) perlu 25 mA dengan tegangan suply 9.2 V
	pada 168 MHz

	109 mA, sudah ada ENC624J komplit 2 buah
	
	5 Des 2013
	==========
	PORT configurasi dirubah ke 5006, supaya tidak bentrok dengan Monita lama
	
	20 Des 2013
	===========
	//printf dll pakai stdlib saja (belajar dari PM_Kapal STM)
	* 
	
	24 Des 2013
	===========
	ADC mode interrupt sudah bisa normal sampai 24 kS/s
	untuk 44 kS/s, belum normal, kayaknya sudah terlalu banyak interrup
	44 * 3 irq/kanal * 4 kanal = 528 ribu interrupt/detik
	
	Coba solusi DMA, lihat DMA ADC pada PM_Kapal 
	
	2 Jan 2014
	==========
	SPI DMA Mulai SIP
	FSMC mulai di optimasi
	OK sip, sudah bisa sampling 50 kS/s (mode start stop)
	
	3 Jan 2014
	==========
	ADC dari 500 Hz sampai 60 kHz ok 
	
	jika BUF_DATA_ADC ditaruh di internal RAM, data jadi halus
	kecepatan tetap di 60 kS/s, di 65 kS sudah kacau
	
	di mainku.h terdapat setting posisi memory BUF_DATA_ADC
	
	akses kepada memory (SRAM) nampaknya menimbulkan noise tegangan (40 mV)
	
	** masih mencari kenapa kalau di external SRAM jadi noise
	sudah OK, dipindah saja init_sram nya.
	* 
	
	7 Jan 2014
	==========
	NVIC_EnableIRQ(SysTick_IRQn), menjadikan sistem lockup.
	lihat di : http://forums.arm.com/index.php?/topic/14203-temporarily-block-systick-interrupt-but-not-lose-it/
	http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/60082/215015.aspx
	
	104 kS/s .. start/stop ok
	104 kS/s .. continue juga ok, 60 kS/s bagus
	
	* 
	TODO :
	- gimana supaya multicast tidak terbuka terus, misalnya setiap 5 detik
	  baru bisa diping lagi.	
	- masih ada error pada pastikan_paket (enc624j600.c), dimana masih
	  terus menerus ada paket palsu ??	 
	- Jumper untuk set default env	
	- setelah menulis konfigurasi baru, mestinya reboot
	
*/
 
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#include "./include/mainku.h"
#include "./include/awal.h"
#include "./include/enviro.h"

#include "./include/ads_1274.h"
#include "./include/fsmc_sram.h"


void enc624Init();
void serial_putc(unsigned char c);

int init_timer_tick();
extern struct t_buf_serial buf_serial;

float akar(float AA);

#if (BUF_ADC_PAKAI_POINTER == 1)
float *buf_data_adc[ JUM_KANAL ];
#else
#if (BUF_ADC_SRAM_LUAR == 1)
//float buf_data_adc[ JUM_KANAL ][SIZE_BUF_ADC] __attribute__ ((section (".sram_luar")));
//float buf_data_adc2[ JUM_KANAL ][SIZE_BUF_ADC] __attribute__ ((section (".sram_luar")));
float buf_data_adc[ NUM_BUF_ADC ][ JUM_KANAL ][ SIZE_BUF_ADC ] __attribute__ ((section (".sram_luar")));
#else
float buf_data_adc[ JUM_KANAL ][SIZE_BUF_ADC];
#endif
#endif

int loop_5_detik = 0;	/* akan berhenti kirim data setelah 5 detik */

/* environment global */
struct t_set_ciliwung *set_cil; 
 
/**************************************************************************************/
 
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
  	USART3_Configuration();

	int i;
	int loop_flush;
	int count_lama = 0;
	

	GPIO_InitTypeDef  GPIO_InitStructure;

	/* untuk blink saja */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

   	/* Configure PG6 & PG8 in output pushpull mode */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOG, &GPIO_InitStructure);

	debug_entry;
	printf("\r\n");
	printf("Monita Vibration Modul\r\n");
	printf("xsxsxsxsxsxsxsxsxsxsxsxsxsxsxsxsxsxsxsxsx\r\n");
	printf("STM32F4 : Build %s - %s\r\n", __DATE__, __TIME__);
	printf("System core clock %d\r\n", SystemCoreClock );

	/* SRAM ini jika ditaruh setelah init yang lain2, ADC jadi tidak jalan */
	//SRAM_Init();
	
	#if (BUF_ADC_PAKAI_POINTER == 1)
	buf_data_adc[0] = (int *) Bank1_SRAM2_ADDR;
	buf_data_adc[1] = (int *) (Bank1_SRAM2_ADDR + (SIZE_BUF_ADC * sizeof(float)));
	buf_data_adc[2] = (int *) (Bank1_SRAM2_ADDR + 2*(SIZE_BUF_ADC * sizeof(float)));
	buf_data_adc[3] = (int *) (Bank1_SRAM2_ADDR + 3*(SIZE_BUF_ADC * sizeof(float)));
	#endif
	
	printf("Setelah SRAM_init\r\n");
	printf("Data ADC pointer : 0x%X, 0x%X, 0x%X, 0x%X\r\n", \
		buf_data_adc[0], buf_data_adc[1], buf_data_adc[2], buf_data_adc[3]);
	

	init_timer_tick();
	
	/* ganti priority timier tick */
	NVIC_SetPriority(SysTick_IRQn, 72);	// tadinya 6
	
	printf("GET KONFIGURASI\r\n");
	struct t_env *env;
	//struct t_set_ciliwung *set_cil;
	if ( (env = baca_env()) == 0)
	{
		printf("Setting ENV Error, direset default\r\n");
		set_default_env();
		printf("Simpan ENV\r\n");
		simpan_env();
	}
	set_cil = (struct t_set_ciliwung *) env->buf;
	
	/* cek apakah jumper set default sedang aktif 
	 * mestinya disini
	 * */
	init_sistem_lwip();
	
	int loop_idle = 0;
 	int loop_paket = 0;

 	int loop_k = 0;
 	int tick_cek = 0;	/* untuk cek tick jika overflow */

	SRAM_Init();		/* ditaruh disini hasil ADC jadi bagus ?? */
	TIM3_Config();
	TIM3_Jalan();
	
	/* persiapan ADS */
	init_gpio_ads();
	pwdn_low();		// biar sempat total reset

	/* SPI3 untuk ADS 1274 */
	init_SPI3_slave();
			
 	printf("sebelum loop besar !\r\n");

 	//SPI3_send_read(0xAA);
	SPI3->DR = 0x01;
	printf("After SPI\r\n");

	/* start sinkronise ADS1274 */
	pwdn_high();
	sync_low();
	set_high_res_ads1274();
	init_clock_khz();
	set_sampling_rate( set_cil->sampling_rate );

	float aa = akar( 45.43 );
	printf("Coba cek akar !\r\n");

	int flag_test = 0;
	sync_high(); 	// ADS1274, aktif
	
	set_jalankan_adc();
	printf("ADC paksa jalan\r\n");
	
	// entah kenapa DMA kayaknya gak stabil, kadang OK/nggak
	// data masih kacau
	/* timer jalan terus */
	//TIM3->CR1 |= TIM_CR1_CEN; 
	//SPI_Cmd(SPI3, ENABLE);	
	
	loop_flush = 0;	
  	while(1)
  	{
		loop_idle++;
		
		remove_gain_flag();
		//set_jalankan_adc();
		
		if (time_for_serial())
		{
			// proses serial
			if (buf_serial.rx_in)
			{
				buf_serial.rx_in--;
				serial_putc( buf_serial.rx_buf[ buf_serial.rx_in ] );
			}
		}

		if (time_for_gpio())
		{
			// proses GPIO
		}

		proses_lwip();
		proses_kirim_data( loop_5_detik );

		if (time_for_blink())
		{
			//extern unsigned int tick_count;
			unsigned int temp_tick = get_tick_count();
			//printf(".*");
			//printf("loop %d, paket_loop %d, tick %d\n", loop_idle, loop_paket, temp_tick);
			//printf("TIM3 CNT %d\n", TIM3->CNT);
			//printf("SPI3_SR 0x%X, CR1 0x%X, CCER 0x%X\n", SPI3->SR, SPI3->CR1, TIM3->CCER);
			//extern int data_adc;
			//printf("data 0x%X\n", data_adc);

			extern int total_count_adc;
			int rate = (total_count_adc - count_lama) * SIZE_BUF_ADC;
			
			printf("\r\n%d: total count %d, rate %d\r\n", loop_idle, total_count_adc, rate);
			count_lama = total_count_adc;
			
			loop_idle = 0;
			loop_paket = 0;

			togle_led();

			if (flag_test)
			{
				flag_test = 0;
			}
			else
			{
				flag_test = 1;
			}

			loop_flush++;

			if (loop_flush > 10)
			{
				enc424j600MACFlush_ku();
				//printf("flush\r\n");
				//printf("rr");
				loop_flush = 0;
			}
			
			loop_5_detik++;
		}	/* blink */
  	} /* while */
}

static int tog;

void togle_led(void)
{
	if (tog)
	{
		/* PG8 to be toggled */
    	GPIO_SetBits(GPIOG, GPIO_Pin_8);
    	tog = 0;
	}
	else
	{
		tog = 1;
		GPIO_ResetBits(GPIOG, GPIO_Pin_8);
	}
}

#if 0
void serial_puts(char *s)
{      
   while (*s)
   {
		USART_SendData(USART3, *s++);

		//Loop until the end of transmission
      	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
      	{
      	}
   }
}
#endif

void serial_putc(unsigned char c)
{
	USART_SendData(USART3, c);

	//Loop until the end of transmission
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
    }
}

char serial_getc()
{
	
}

/* untuk testing vfp */
float akar(float AA)
{
	return sqrtf( AA );
}




