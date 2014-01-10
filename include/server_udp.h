#ifndef _H_MOBAR_
#define _H_MOBAR_


#define L_DATA_PRESSURE			6000
#define BESAR_DATA_RECIP		60000
#define MAKS_SIKLUS				20		// 14
#define SAMPLING_RATE_MOBAR		80000	/* harusnya hanya 76800 = 2.56 * 30 kHz */

#define DATA_NOT_READY			100
#define DATA_READY				101
#define DATA_SEND				121		/* akan di increment jika lebih dari batas, maka direset untuk not ready */
#define DATA_SEND_LIMIT			125

#define SUDAH_KONEK				201
#define KIRIM_ULANG				202
#define KIRIM_SELESAI			203

#define BESAR_PAKET				1024
#define BESAR_PAKET_F			(BESAR_PAKET / 4)

#define REQ_PRESS				1
#define REQ_VIB					2
#define REQ_PRESS_VIB			3
#define REQ_NO					0
//#define JUM_KANAL				2

#define JUM_CLIENT				32			// jumlah client yang boleh akses bersamaan

#define	pecahan_terkecil	0.000030519		// 1 : 32767, 0.000030518 = 1/32768

#define resistor_pembagi	0.27			// 27 k dan 100 k
#define	pengali_resistor	3.703703704		// 1 / 0.27
#define faktor_pengali_ana		0.00028257		/* = 3.703 * 0.000076294 =  2.5 Volt dibagi 32768 */



/* 	struct ini tidak ikut disimpan dalam MMC, hanya sebagai pembantu 
	analisa dan proses ambil data 
*/
struct t_data_recip {
	unsigned int current_siklus;
	unsigned int tipe;						// tipe data
	unsigned int total_putaran;
	unsigned int total_putaran_lama;
	unsigned int akumulasi_rpm;				// akumulasi rpm dalam 10 putaran
	unsigned int min_rpm;					// batas bawah rpm yang valid
	unsigned int maks_rpm;					// batas atas rpm yang valid
	unsigned int jum_sample[MAKS_SIKLUS];	// jumlah sample raw masing2 siklus
	
	/* 	
		rpm, dihitung dengan integer, supaya cepat (menghindari floating point)
		hasil seharusnya adalah dibagi 10, misalnya rpm seharusnya 
		500.4 rpm, maka akan ditulis disini sebagai 5004 rpm
	*/
	unsigned int rpm[MAKS_SIKLUS];
	/* 
		misalnya rpm invalid, harus dibatasi misalnya 10 %
		dari normal
	*/
	unsigned int flag[MAKS_SIKLUS];
	//unsigned int tambahan;	/* data tambahan yang diperlukan untuk melengkapi proses filter */
	//unsigned int len_filter;
	//struct t_stat_data_recip stat[MAKS_SIKLUS];
	unsigned int jum_deg[ MAKS_SIKLUS ];		/* jumlah degree yang terdeteksi */	
};

struct tt_req {
		char head[10];			// getdata, ressend
		int  state;             // SUDAH_KONEK, SELESAI, ULANG
		int  urutan;			// urutan paket data
		//int  tipe_data;			// 10 : kanal_A1, 11 : kanal_B1,
		int  pos_data;			// posisi data yang sudah ditarik dari buffer
		int  total;				// total paket
		int  last_kanal;		// tadinya nomer_siklus
		int  jum_sample;		/* jumlah sampel pressure */
		int  jum_paket;			/* tadinya jum_paket_p */
		int  kanal;				/* ditambahkan 6 sept 2012, nomer kanal sedang dikirim, mulai 1 */
		int  urut_kanal;        /* memudahkan manipulasi array */
		int  lost_paket;
		int  fase_enabled;
		int  flag;				/* 1 master, 2 slave */
		int  bat_level;			/* mulai 2 Des 2012 */
		int  arus_charge;
		int  buck_level;
		int	 	sampling_rate;		// real sampling rate modul
		float 	pecahan_waveform;	// pecahan waveform
		unsigned char buf[1024];
};

/* protocol request versi 2, 21 Des 2013 
 * 
 * client/HMI/monitor meminta sejumlah data
 * kemudian server (ciliwung) mengirimkan data dimaksud.
 * 
 * Jika request terlalu cepat (misal sampling pelan), maka
 * tidak dikirim sampai requested sample terpenuhi (tetapi
 * dikasih kabar untuk nunggu).
 * 
 * data akan ditaruh pada buf, selang seling sesuai kanal
 * yang aktif. mulai dari kanal 0
 * 
 * server akan langsung membalas sesuai dengan data yang diminta
 * 
 * 
 * */
struct tt_req2 {
		char head[10];	
		char cur_kanal;			/* data yang sedang dikirim milik kanal berapa, mulai dari 0 */
		int kanal_enable;       /* jumlah kanal aktif, format mengikuti enviro */
		int request_sample;		/* sample diminta, ini akan diganti nomer paket oleh ciliwung */
		unsigned char buf[BESAR_PAKET];
};

struct t_client {
	int port;
	
};

/* tipe2 data */
#define 	tipe_P			2
#define 	tipe_V   	   	3
#define		tipe_Bv			7
#define		tipe_Ws			4
#define  	tipe_Pkomp   	8
#define  	tipe_PsPd    	9
#define  	tipe_vibvalve   10 
#define  	tipe_Vkomp  	11

//31 agustus 2007, tipe Ultrasonik & ignition secondary
#define 	tipe_ult_eng    25
#define 	tipe_ult_valv   26
#define 	tipe_ult_komp   27
#define 	tipe_ign_sec    28

// 1 Mei 2009, tipe_data
#define		TIPE_DATA_P			tipe_P
#define		TIPE_DATA_V			tipe_V
#define		TIPE_DATA_WS		tipe_Ws
#define		TIPE_DATA_BV		tipe_Bv
#define		TIPE_DATA_PKOMP		tipe_Pkomp
#define 	TIPE_DATA_PSPD		tipe_Ps_Pd		/* tekanan suction & dicharge kompresor */
#define 	TIPE_DATA_VIBVALVE	tipe_vibvalve
#define		TIPE_DATA_VKOMP		tipe_Vkomp		/* vibrasi silinder kompresor */

#define		TIPE_DATA_DEG		91				/* derajat engkol mobar */

#endif
