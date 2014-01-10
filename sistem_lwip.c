/*
	Bagian LWIP
	furkan jadid
	11 Oktober 2013

	Daun Biru Engineering
	* 
	* 
	
	8 Januari 2014
	Model request data dirubah.
	
	Jika ada request, kita akan balas sejumlah paket selama 4 detik
	jika tidak ada request baru, maka stop
	
	kirim paket akan diserahkan pada loop utama

*/

#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "./enc/enc424j600.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "./include/server_udp.h"
#include "./include/enviro.h"
#include "./include/mainku.h"


//static void set_kirim_data(struct udp_pcb *upcb, struct ip_addr *addr);
void set_kirim_data(struct ip_addr *addr);

#if 0
struct t_prot {
    char head[ 16 ];
    char buf[ 280 ];
    int ip_baru;		/* 32 bit IP */
}; 
#endif

struct t_prot {
    char head[ 16 ];
    int ip_baru;		/* 32 bit IP */
    char buf[ 600 ];
};



#define PORT_CILIWUNG_LOKAL		5006
#define PORT_CILIWUNG_REMOTE	5007
#define PORT_CILIWUNG_DATA      5008

//static struct tt_req t_req;
static struct tt_req2 t_req2;

extern struct t_set_ciliwung *set_cil; 

void proses_paket(struct tt_req *p_req);

void udp_server_init(void);
void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);

__IO uint32_t LocalTime = 0; 

void init_sistem_lwip()
{
	printf("%s():\r\n", __FUNCTION__);
	
	/* Initilaize the LwIP stack */
  	LwIP_Init();	/* ada di netconf.c */
	
  	/* Http webserver Init */
  	//httpd_init();
	udp_server_init();
	
}

void proses_lwip()
{
	/* check if any packet received */
    if ( cek_paket() )
    { 
      	/* process received ethernet packet */
      	LwIP_Pkt_Handle();
    }
    
    /* handle periodic timers for LwIP 
     * 
     * ini harus dipanggil setiap 1 ms
     */
    if (time_for_ether())
		LwIP_Periodic_Handle(LocalTime);

}


void udp_server_init(void)
{
   struct udp_pcb *upcb;
   err_t err;
   
   /* Create a new UDP control block  */
   upcb = udp_new();
   
   if (upcb)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      //err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);
      err = udp_bind(upcb, IP_ADDR_ANY, PORT_CILIWUNG_LOKAL);
      
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_server_receive_callback, NULL);
      }
      else
      {
        udp_remove(upcb);
        printf("can not bind pcb");
      }
   }
   else
   {
     printf("can not create pcb");
   } 
}

//extern float *buf_data_adc[ JUM_KANAL ];
#if (BUF_ADC_PAKAI_POINTER == 1)
extern float *buf_data_adc[ JUM_KANAL ];	/* ada di main.c */
#else
#if (BUF_ADC_SRAM_LUAR == 1)
//extern float buf_data_adc[ JUM_KANAL ][SIZE_BUF_ADC] __attribute__ ((section (".sram_luar")));
//extern float buf_data_adc2[ JUM_KANAL ][SIZE_BUF_ADC] __attribute__ ((section (".sram_luar")));
extern float buf_data_adc[ NUM_BUF_ADC ][ JUM_KANAL ][ SIZE_BUF_ADC ] __attribute__ ((section (".sram_luar")));

//unsigned char temp_buf[ JUM_KANAL ][ BESAR_PAKET ];	/* 4 x 1024 */
float temp_buf_data[ JUM_KANAL ][ SIZE_BUF_ADC ];		/* 4 x 256 x 4 byte(float) = 4096 */

#else
extern float buf_data_adc[ JUM_KANAL ][SIZE_BUF_ADC];
#endif
#endif

extern int count_data_adc;
extern int lagi_int;
extern int flag_buffer_adc;
extern int count_buf_adc;

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
	struct pbuf *p2;
	printf("%s(): len %d, dari (%X)\r\n", __FUNCTION__, p->len, addr->addr);

	err_t ret;

	if ( strncmp( p->payload, "getdata", 7) == 0)
	{		
		/* hanya set flag */
		set_kirim_data( addr );
		
	} /* getdata */
	else if ( strncmp( p->payload, "config", 6) == 0)
	{
		/* kemungkinan config */
		printf("Konfigurasi Read (%d byte)\r\n", sizeof (struct t_set_ciliwung));
		
		struct t_env *s_en = baca_env();
		if (s_en == 0)
		{
			printf("%s(): Baca ENV ERROR 1\r\n", __FUNCTION__);	
			while(1);
		}
		
		struct t_set_ciliwung *s_cil;
		s_cil = (struct t_set_ciliwung *) s_en->buf;
			
		p2 = pbuf_alloc(PBUF_TRANSPORT, sizeof (struct t_set_ciliwung), PBUF_POOL);
   		if (p2 == NULL) 
   		{
			printf("KOK NULL\r\n");
			goto selesai;
   		}
		p2->len = sizeof (struct t_set_ciliwung);
	
		//udp_connect(upcb, addr, port);
		udp_connect(upcb, addr, PORT_CILIWUNG_REMOTE);
		
		memcpy( p2->payload, s_cil, sizeof (struct t_set_ciliwung));
		ret = udp_send(upcb, p2);
		
		pbuf_free(p2);					   
	}
	else if ( strncmp( p->payload, "tulis_konfig", 12) == 0)
	{
		printf("Menulis konfigurasi baru !\r\n");
		int flag_ip;
		
		struct t_env *s_en = baca_env();
		if (s_en == 0)
		{
			printf("%s(): Baca ENV ERROR 2\r\n", __FUNCTION__);	
			while(1);
		}
		struct t_set_ciliwung *s_cil = (struct t_set_ciliwung *) s_en->buf;
		
		/* firmware rev dan pcb rev disimpan dulu */
		char firmware_rev[32];
		char pcb_rev[32];
		
		memcpy(firmware_rev, s_cil->firmware_rev, sizeof (firmware_rev));
		memcpy(pcb_rev, s_cil->pcb_rev, sizeof (pcb_rev));
		
		struct t_prot *prot = (struct t_prot *) p->payload;
		memcpy( s_cil, prot->buf, sizeof (struct t_set_ciliwung));
		
		/* firmware rev & pcb rev dikembalikan lagi */
		memcpy(s_cil->firmware_rev, firmware_rev, sizeof (firmware_rev));
		memcpy(s_cil->pcb_rev, pcb_rev, sizeof (pcb_rev));
		
		/* cek jika IP berubah */
		flag_ip = 0;
		printf("ip_lama %X, baru %X\r\n", s_en->IP0, prot->ip_baru);
		if (s_en->IP3 !=  (0x000000FF & prot->ip_baru)) flag_ip = 1;
		if (s_en->IP2 != ((0x0000FF00 & prot->ip_baru) >> 8)) flag_ip = 2;
		if (s_en->IP1 != ((0x00FF0000 & prot->ip_baru) >> 16)) flag_ip = 3;
		if (s_en->IP0 != ((0xFF000000 & prot->ip_baru) >> 24))	flag_ip = 4;
		
		if (flag_ip)
		{
			printf("IP Berubah %d!\r\n", flag_ip);
			
			s_en->IP3 = (unsigned char)  (0x000000FF & prot->ip_baru);
			s_en->IP2 = (unsigned char) ((0x0000FF00 & prot->ip_baru) >> 8);
			s_en->IP1 = (unsigned char) ((0x00FF0000 & prot->ip_baru) >> 16);
			s_en->IP0 = (unsigned char) ((0xFF000000 & prot->ip_baru) >> 24);
		}
		
		if (strncmp ( s_cil->passwd, "diesel", 6 ) == 0)
		{
			printf("Password benar ! .. OK akan diflash !\r\n");
			
			#if 0
			/* cek jika waktu untuk menulis konfigurasi sudah lewat */
			if ( boleh_tulis_env() == 0 )	/* ada di timer.c */
			{
				printf("ERR: Belum boleh tulis FLASH !!\r\n");
				
				sprintf(prot->head, "TIME NOT OK !");
				udp_connect(upcb, addr, PORT_CILIWUNG_REMOTE);
				ret = udp_send(upcb, p);
				
				goto selesai;
			}
			#endif
			
			if (simpan_env() == 0)
			{
				sprintf(prot->head, "SAVE NOT OK !");
				udp_connect(upcb, addr, PORT_CILIWUNG_REMOTE);
				ret = udp_send(upcb, p);
				
				goto selesai;
			}
			
			sprintf(prot->head, "OK !");
			udp_connect(upcb, addr, PORT_CILIWUNG_REMOTE);
			ret = udp_send(upcb, p);
			
			if (flag_ip)
			{
				/* mestinya restart */
				printf("Mestinya restart .... drrrrrrr ... \r\n");
			}
		}
		else
		{
			printf("Password salah (%s) !\r\n", s_cil->passwd);
			sprintf(prot->head, "PASSWD ERR !");
			udp_connect(upcb, addr, PORT_CILIWUNG_REMOTE);
			ret = udp_send(upcb, p);
		}
	}
	
	//printf("Kirim Balik\r\n");
  	/* Connect to the remote client */
  	//udp_connect(upcb, addr, UDP_CLIENT_PORT);	// << temporari
    //udp_connect(upcb, addr, port);
    
  	/* Tell the client that we have accepted it */
 	//udp_send(upcb, p);					   		// << temporari

	selesai:
  	/* free the UDP connection, so we can accept new clients */
  	udp_disconnect(upcb);
	
  	/* Free the p buffer */
  	pbuf_free(p);
   
}

#if 0
void proses_paket(struct tt_req *p_req)
{
	int i;
	
	if (strncmp( p_req->head, "getdata", 7) == 0)
	{
		printf("GETDATA !\n");

		/* jika dari urutan = 0, berarti awal permintaan */
		if (p_req->urutan == 0)
		{
			p_req->urut_kanal = 0;

			//udp_connect(upcb, addr, UDP_CLIENT_PORT);
			//udp_send(upcb, p);
    

		}		
	} /* getdata */
}
#endif

/* hanya menyimpan informasi */
static struct ip_addr l_addr;
static int flag_udp_req;
static int last_num_buf;

struct udp_pcb *udp2;
static int p_data;		/* pointer ke temp buf data data adc */
extern int loop_5_detik;	
static int data_valid = 0;
extern int total_count_adc;
int count_adc_saat_req;		/* count adc saat request data dimulai */
int cnt_kirim;				/* mulai dari detik ke 0, dan bertambah setiap telah dikirim */

static void copy_buf_adc()
{
	if (count_buf_adc > 0)
		last_num_buf = count_buf_adc - 1;
	else
		last_num_buf = (NUM_BUF_ADC - 1);
	
	/* TODO saat overflow !! */
	count_adc_saat_req = total_count_adc;
	
	/* mengkopy 4096 byte ini perlu 125 uS */
	memcpy( &temp_buf_data[0][0], &buf_data_adc[ last_num_buf ][ 0 ][ 0 ], sizeof (temp_buf_data) );
}

void set_kirim_data(struct ip_addr *addr)
{
	int i,y;
	
	/* 	
	 * 	cek jika direquest dari alamat yang sama, maka tidak dibuatkan
		udp baru, hanya direset loop_5_detiknya saja
	*/
	
	if (memcmp( &l_addr, addr, sizeof (l_addr)) == 0)
	{
		loop_5_detik = 0;
		
		printf("req sama\r\n");
		return;
	}
	 
	udp2 = udp_new();
	if ( !udp2 )
	{
		printf("ERROR : can not create pcb udp2");
		return ;
	}
	
	/* menyimpan nomer buffer terakhir, dari nomer buffer inilah data
	 * akan dikirim */
	
	memcpy( &l_addr, addr, sizeof (l_addr));	
	flag_udp_req = 1;	// tadinya 4 untuk sekali kirim 1000 data
	
	copy_buf_adc();
	data_valid = 1;
	
	p_data = 0;
	cnt_kirim = 0;
	loop_5_detik = 0;
	printf("%s():__ num_buf %d\r\n", __FUNCTION__, last_num_buf);
}

void set_stop_kirim_data()
{
	flag_udp_req = 0;
	data_valid = 0;
	memset( &l_addr, 0, sizeof (l_addr));
	udp_remove( udp2 );
	
	printf("%s():__ \r\n", __FUNCTION__);
}

/* ini akan dipanggil oleh main loop 
 * 
 * jika ada request data, akan dikirim terus
 * dengan increment block data yang belum dikirim
 * 
 * jika belum siap, ditinggalkan dan dipanggil lagi
 * berikutnya
 * 
 * */
void proses_kirim_data(int loop_5)
{
	int i_kanal;
	int y;
	int bit_kanal;
	int num_paket_to_send = 0;
	struct pbuf *p2;
	struct tt_req2 *p_req_payload;
	int ret;
	
	if (!flag_udp_req) return;
	
	if (loop_5_detik > 5)
	{
		if (flag_udp_req)
			set_stop_kirim_data();
	}
		
	if (flag_udp_req > 0)
	{
		if ( !data_valid ) goto tunggu;
	
		data_valid = 0;
		//printf("%s():%d, %X, 5s: %d, sr %d\r\n", __FUNCTION__, flag_udp_req, l_addr.addr, loop_5_detik, last_num_buf);
		
		udp_connect( udp2, &l_addr, PORT_CILIWUNG_DATA);
		t_req2.request_sample = 0;
				
		/* loop kanal yang dikirim */
		i_kanal = 0x1;
		for (y=0; y< JUM_KANAL; y++)
		{
			bit_kanal = (int) (i_kanal << y);	
			//printf("%d: bit_kanal %X, kanal %X\r\n", y, bit_kanal, set_cil->kanal_enable);
									
			if ( set_cil->kanal_enable & bit_kanal)
			{
				t_req2.cur_kanal = y;
						
				/* alloc untuk sending */
				p2 = pbuf_alloc(PBUF_TRANSPORT, sizeof (t_req2), PBUF_POOL);	// 800 nS
						
				p_req_payload = (struct tt_req2 *) p2->payload;
				p_req_payload->request_sample = cnt_kirim;
				p_req_payload->cur_kanal = y;

				if (p2 == NULL) 
				{
					printf("KOK NULL %d\n", (4 - flag_udp_req)+1);
					return;
				}

				//printf("kanal %d, send %d: n_buf %d:", y, (4 - flag_udp_req)+1, count_buf_adc);
				p2->len = sizeof (t_req2);
						
				/*  memasukkan datanya, cara ini perlu 32 uS 
				 *  memcpy( t_req2.buf, &buf_data_adc[ y ][ p_data ], BESAR_PAKET );										
					memcpy( p2->payload, &t_req2, sizeof (t_req2));
				 */
				
				/* ini 26 uS */										
				memcpy( p_req_payload->buf, &temp_buf_data[ y ][ 0 ], BESAR_PAKET );	
				ret = udp_send( udp2, p2);
											
				pbuf_free(p2);	// 1.2 uS
			} 
		} /* loop_kanal */
		
		cnt_kirim++;
	}

tunggu :	
	/* jika count_buf_adc > 2 posisi didepan, artinya bisa dicopy sekarang */
	if ( total_count_adc > count_adc_saat_req )
	{
		copy_buf_adc();
		data_valid = 1;
	}
	else
	{
		//printf("ww");
		//data_valid = 0;
	}
}
