/*
	9 Juli 2013
	Furkan Jadid
	Daun Biru Engineering

	Driver untuk LTC6904
	banyak dicopy dari versi linux/haliza/beaglebone filter_clock.c

	*/
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

#include "./include/awal.h"
#include "./include/mainku.h"

#include <math.h>
#include <float.h>

//#define ADDR_LTC6904	0x17
//#define ADDR_LTC6904	0x68 	// ATA2538

#define ADDR_LTC6904	0x2E	// entah kenapa harus digeser ke kiri 1 bit dibanding address linux

static int status;

int init_clock_khz(void)
{
	debug_entry;
	
	status = 0;
	//init_I2C3();
	init_I2C2();
	
	//I2C_stop(I2C3); 
	
	debug_leave;
	return status;
}

/* minimum adalah 1 kHz */
void set_clock_khz(int khz)
{
	double f_oct = 0;
	double f_dac = 0;
	//float f_oct = 5.0;
	//float f_dac = 5.0;
	int oct;
	int dac;
	unsigned char bf[4];

	printf("%s(): __entri %d kHz\r\n", __FUNCTION__, khz);

	if (khz < 25)
	{
		printf("WARNING, clock yang diminta akan lebih rendah dari 25 kS/s !\n");
	}

	if (khz > 0)
	{		
		f_oct = 3.322 * log10( (double) (khz * 1000.00 / 1039) );
		oct = (int) f_oct;
		
		f_dac = 2048.00 - (2078.00 * pow(2.00, 10.00+oct) /(khz * 1000.00));
		dac = (int) f_dac;

		//printf("%s(): f oct %f, %d, dac %f, %d\n", __FUNCTION__, f_oct, oct, f_dac, dac); 
		printf("%s(): oct %d, dac %d\r\n", __FUNCTION__, oct, dac); 
	}
	else
	{
		oct = 0;
		dac = 0;
	}

	bf[0] = oct << 4;
	dac = dac << 2;
	
	bf[0] |= ((dac & 0x0F00) >> 8);
	bf[1] = dac & 0x00FE;		// lihat manual LTC6904 halaman 10, kita hanya pakai CLK ! jadi bukan FC, tetapi FE

	#if 1
	if (I2C_start(I2C2, ADDR_LTC6904, I2C_Direction_Transmitter))
	{
		printf("GAGAL ..... \r\n");
		I2C_stop(I2C2); 
		return;
	}

	I2C_write(I2C2, bf[0]); // write another byte to the slave
	I2C_write(I2C2, bf[1]);
	I2C_stop(I2C2); 
	#endif
	
	printf("%s(): OK __finish\r\n", __FUNCTION__);
}

/* ini dalam Hz */
void set_sampling_rate(int s_rate)
{
    double new_rate;

    printf("%s(): __entri %d sample/s\r\n", __FUNCTION__, s_rate);

    /* high resolution adalah 512x clock, sedangkan oversampling adalah 128x */
    new_rate = (double) (512 * s_rate);
    set_clock_khz( (int) (new_rate / 1000.00) );
}
