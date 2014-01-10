/* 
 * 	28 Nov 2013
 *  furkan jadid
 *  daun biru engineering
 * 
 * 
 */

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"

#include "./include/mainku.h"
#include "./include/awal.h"
#include "./include/enviro.h"

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_7   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_10   /* End @ of user Flash area */

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* ditaruh di sektor terakhir */
#define POSISI_ALAMAT_ENV	ADDR_FLASH_SECTOR_11

uint32_t StartSector = 0, EndSector = 0, Address = 0, EndAddress = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0 ;

static uint32_t GetSector(uint32_t Address);

#if 0
void test_flash()
{
	int i;
	
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();
    
	/* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

	/* Get the number of the start and end sectors */
	StartSector = GetSector(FLASH_USER_START_ADDR);
	EndSector = GetSector(FLASH_USER_END_ADDR);

	printf("start testing flash\r\n");
	
	for (i = StartSector; i < EndSector; i += 8)
	{
		printf("Mulai %d\n", i);
    
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    if (FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE)
    { 
      /* Error occurred while sector erase. 
         User can add here some code to deal with this error  */
		while (1)
		{
		}
		}
	}
	printf("Test OK\n\r");
}
#endif

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;
	

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_Sector_0;  
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_Sector_1;  
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_Sector_2;  
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_Sector_3;  
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_Sector_4;  
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_Sector_5;  
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_Sector_6;  
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_Sector_7;  
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_Sector_8;  
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_Sector_9;  
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_Sector_10;  
	}
	else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
	{
		sector = FLASH_Sector_11;  
	}

	return sector;
}

static struct t_env t_env;

struct t_env *baca_env()
{
	/* 
	 * baca env ini adalah mendapat environment yang benar, bukan
	 * hanya pointer kepada flash !
	 */
	unsigned int *p = (unsigned int *) POSISI_ALAMAT_ENV;	
	memcpy( &t_env, p, sizeof (t_env));
	
	if (t_env.magic1 != 0xaa) return 0;
	if (t_env.magic2 != 0x77) return 0;
	
	return &t_env;
}

struct t_set_ciliwung *baca_set_ciliwung()
{
	/* ini hanya mendapatkan pointer kepada flash */
	unsigned int *p = (unsigned int *) POSISI_ALAMAT_ENV;	
	memcpy( &t_env, p, sizeof (t_env));
	
	return (struct t_set_ciliwung *) &t_env.buf;
}

void print_env()
{
	struct t_set_ciliwung *set_ciliwung = (struct t_set_ciliwung *) &t_env.buf;
	
	printf("Monita Vibration Module\r\n");
	printf("Build %s, %s\r\n", __DATE__, __TIME__);
	printf("PCB : %s, Firmware : %s\r\n", set_ciliwung->pcb_rev, set_ciliwung->firmware_rev);
	
	printf("%d.%d.%d.%d\r\n", t_env.IP0, t_env.IP1, t_env.IP2, t_env.IP3);
}

int simpan_env()
{
	FLASH_Unlock();
	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
                  
    StartSector = GetSector(POSISI_ALAMAT_ENV);
    if (FLASH_EraseSector(StartSector, VoltageRange_3) != FLASH_COMPLETE)
	{
		printf("%s(): Hapus flash ERROR !\r\n", __FUNCTION__);
		return 0;
	}
	
	/* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Address = POSISI_ALAMAT_ENV;
	EndAddress = Address + sizeof (t_env);
	
	unsigned int *p_env = &t_env;
	
	while (Address < EndAddress)
	{
		if (FLASH_ProgramWord(Address, *p_env) == FLASH_COMPLETE)
		{
			Address = Address + 4;
			*p_env++;
		}
		else
		{ 
		/* Error occurred while writing data in Flash memory. 
         User can add here some code to deal with this error */
			printf("%s(): Simpan env ERROR !\n", __FUNCTION__);
			return 0;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock(); 

	/* Check if the programmed data is OK 
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
	
	Address = POSISI_ALAMAT_ENV;
	MemoryProgramStatus = 0x0;
	
	p_env = &t_env;
	int data;
  
	while (Address < EndAddress)
	{
		data = *(__IO uint32_t*)Address;
		if (data != *p_env)
		{
			MemoryProgramStatus++;  
		}
		
		p_env++;
		Address = Address + 4;
	}  
	
	if (MemoryProgramStatus)
	{
		printf("%s(): Env tidak cocok !\r\n", __FUNCTION__);
		return 0;
	}
	printf("Simpan ENV OK\r\n");
	
	return 1;
}

/* membuat default setting pada flash 
 * IP : 192.168.1.240
 * 
 * Kanal enable semua : 4 atau 8 kanal
 * Sampling rate 40 kS/s
 * Fase input, disable
 * Panjang data = 0
 */
  
void set_default_setting()
{
	struct t_set_ciliwung *set_ciliwung = (struct t_set_ciliwung *) &t_env.buf;
	
	set_ciliwung->kanal_enable = (0x1 | 0x2 | 0x4 | 0x8);
	set_ciliwung->fase_enable = 0;
	set_ciliwung->mode_fase = 1;		/*1 =  pickup, 2 = 360 derajat, 3 = 720 derajat, 4 = pjg tertentu */
	set_ciliwung->sampling_rate = 1000;/* 1 kS/s */
	set_ciliwung->len_data = 8192;
	
	sprintf(set_ciliwung->firmware_rev, FIRMWARE_REV);
	sprintf(set_ciliwung->pcb_rev, PCB_REV); 
}

void set_default_ip(void)
{
	int i;	

	printf(" Default IP = ");	
	t_env.IP0 = 192;
	t_env.IP1 = 168;
	t_env.IP2 = 1;
	t_env.IP3 = 240;	
	printf("%d.%d.%d.%d\r\n", t_env.IP0, t_env.IP1, t_env.IP2, t_env.IP3);
	
	printf(" Default GW = ");	
	t_env.GW0 = 192;
	t_env.GW1 = 168;
	t_env.GW2 = 1;
	t_env.GW3 = 1;
	printf("%d.%d.%d.%d\r\n", t_env.GW0, t_env.GW1, t_env.GW2, t_env.GW3);

	t_env.NET_MASK0 = 255;
	t_env.NET_MASK1 = 255;
	t_env.NET_MASK2 = 255;
	t_env.NET_MASK3 = 0;
}

void set_default_env()
{
	printf("SET DEFAULT ENVIRONMENT & SETTING \n");
	
	set_default_ip();
	set_default_setting();
	
	t_env.magic1 = 0xAA;
	t_env.magic2 = 0x77;
}


