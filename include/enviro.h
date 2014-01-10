#ifndef _ENVIRO_H_
#define _ENVIRO_H_

#include "mainku.h"
//#define IAP_LOCATION 0x7FFFFFF1
//typedef void (*IAP)(unsigned int [],unsigned int[]);

struct t_kalib {
	float m;
	float y;
	char ket[64];
};

struct t_env {
	char nama_board[32];
	unsigned char IP0;
	unsigned char IP1;
	unsigned char IP2;
	unsigned char IP3;
	unsigned char GW0;
	unsigned char GW1;
	unsigned char GW2;
	unsigned char GW3;
	struct t_kalib kalib[20];
	unsigned char NET_MASK0;
	unsigned char NET_MASK1;
	unsigned char NET_MASK2;
	unsigned char NET_MASK3;
	
	unsigned char buf[2048];	/* untuk simpan2 data general */
	
	int magic1;
	int magic2;
	char passwd[32];		/* proteksi untuk write ke flash */
};

//void save_env(int argc, char **argv);
//void print_env(int argc, char **argv);
//static void set_default_ip(void);
//int baca_env(char tampil);

struct t_set_kanal
{
    float sensi;        /* sensitivity */
    float c_sensi;      /* konstanta jika ada */
    char  flag_icp;
    char  flag_auto_gain;
    char  range;
    char  flag_kontrol;
    char  satuan[12];
    char  note[16];
};

struct t_set_ciliwung
{
	int kanal_enable;
	int fase_enable;
	int mode_fase;
	int sampling_rate;
	int len_data;
	char firmware_rev[32];
	char pcb_rev[32];
	char keterangan[128];
	char passwd[32];	/* untuk menumpangkan password yang akan dikirim */
	
	 /* ditambahkan 9 Januari 2014 */
    struct t_set_kanal set_kanal[ JUM_KANAL ];
};


struct t_env *baca_env();
struct t_set_ciliwung *baca_set_ciliwung();
int simpan_env();

#endif
