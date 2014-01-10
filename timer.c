/*
	23 Juni 2013
	Furkan Jadid, Daun Biru Engineering

	Timer

*/

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "./include/mainku.h"

static void kalibrasi_loop();

extern unsigned int tick_count;
static unsigned int tick_serial;
static unsigned int tick_control_gpio;
static unsigned int tick_ether;
static unsigned int tick_blink;
static unsigned int tick_arp;
static unsigned int tick_config_flash;

void reset_timer()
{
	debug_entry;
	
	//tick_count = 0;
	reset_tick_count();
	tick_serial = 0;
	tick_control_gpio = 0;
	tick_ether = 0;
	tick_blink = 0;
	tick_arp = 0;	
	tick_config_flash = 0;
}

/* tick interrupt akan increment tick_count (ada di stm32f4xx_it.c) */
int init_timer_tick()
{
	debug_entry;
	reset_timer();
	
	if (SysTick_Config(SystemCoreClock / F_HZ_TICK))
	{
		debug_error;
    	/* Capture error */ 
    	return -1;
  	}

	kalibrasi_loop();
	
  	debug_leave;
  	return 0;
}

/* serial dicek setiap ms */
int time_for_serial()
{
	int t_count = tick_count;
	
	if ((t_count - tick_serial) > DELAY_SERIAL)
	{
		tick_serial = t_count;
		return 1;
	}
	else
		return 0;
}

int time_for_gpio()
{
	int t_count = tick_count;

	if ((t_count - tick_control_gpio) > DELAY_CONTROL_GPIO)
	{
		tick_control_gpio = t_count;
		return 1;
	}
	else
		return 0;
}

int time_for_ether()
{
	int t_count = tick_count;
	if ((t_count - tick_ether) > DELAY_ETHER)
	{
		tick_ether = t_count;
		return 1;
	}
	else
		return 0;	
}

int time_for_blink()
{
	int t_count = tick_count;
	
	if ((t_count - tick_blink) > DELAY_BLINK)
	{
		tick_blink = t_count;
		return 1;
	}
	else
		return 0;	
}

int boleh_tulis_env()
{
	extern unsigned int tick_count;
	
	int t_count = tick_count;	
	if ((t_count - tick_config_flash) > DELAY_CONFIG_FLASH)
	{
		tick_config_flash = t_count;
		return 1;
	}
	else
		return 0;
}

int time_for_arp()
{
	int t_count = tick_count;
	
	if ((t_count - tick_arp) > DELAY_ARP)
	{
		tick_arp = t_count;
		return 1;
	}
	else
		return 0;	
}

/* kalibrasi loop selama 200 ms */

static int tick_kalib;
static int loop_ms;
static int loop_us;

static int time_for_kalib()
{
	int t_count = get_tick_count();
	//printf("t %d\n", t_count);
	
	if ((t_count - tick_kalib) > DELAY_KALIB)
	{
		tick_kalib = t_count;
		return 1;
	}
	else
		return 0;	
}

/*
	untuk melakukan kalibrasi loop
	dicoba untuk melakukan loop sebanyak 10000 kali
	dan itu dicek perlu waktu berapa detik

	ingat 1 clock tick adalah 0.1 ms
	*/

static void kalibrasi_loop()
{
	debug_entry;
	
	/* tunggu 0.5 detik untuk mendapatkan loop */
	int loop_5 = 0;
	int try = 0;
	tick_kalib = get_tick_count();

	printf("Tick_kalib %d\r\n", tick_kalib);
	
	while(1)
	{
		loop_5++;		
		if (time_for_kalib())
		{
			break;	
		}

		/*
		if (try > 10) 
		{
			printf("Kalibrasi loop gagal\n");
			loop_ms = 6000;
			loop_us = 6;

			return;
		}*/
		loop_5++;
		loop_5++;
	}
	printf("loop 200 ms : %d\r\n", loop_5);
	
	loop_ms = (int) (loop_5 / 200);
	loop_us = (int) (loop_5 / 200000);
	printf("%s(): loop 1ms = %d loop, loop_us = %d\r\n", __FUNCTION__, loop_ms, loop_us);
}

/* simple delay dengan loop dalam ms*/
void delay_loop_ms(int ms)
{
	if (ms <= 0) return;
	
	int loop = ms * loop_ms;

	while( loop-- );	
}

void delay_loop_us(int us)
{
	if (us <= 0) return;

	int loop = us * loop_us;

	while( loop-- );
}



