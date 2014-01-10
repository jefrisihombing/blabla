EXECUTABLE=blinku.elf
BIN_IMAGE=blinku.bin

#ARCH = ../toolchain/arm-2011.03/bin/./arm-none-eabi
#ARCH = /home/jadid/mrogram/STM32F4/gcc-arm-none-eabi-4_6-2012q2/bin/./arm-none-eabi
ARCH = /home/jadid/mrogram/STM32F4/gcc-arm-none-eabi-4_7-2013q1/bin/./arm-none-eabi

# Tool definitions
CC      = $(ARCH)-gcc
LD      = $(ARCH)-ld
AR      = $(ARCH)-ar
AS      = $(ARCH)-as
CP      = $(ARCH)-objcopy
DD		= $(ARCH)-objdump
SIZE	= $(ARCH)-size
TOUCH	= touch

PENULIS= ../texane-stlink/flash/./st-flash

#Cortex-M4 targets: -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

#CFLAGS=-g -O2 -mlittle-endian -mthumb
#CFLAGS+=-mcpu=cortex-m4	

CFLAGS	=	-g -mlittle-endian -mthumb
#CFLAGS	=	-g -mbig-endian -mthumb
#CFLAGS	=	-mthumb
CFLAGS	+= 	-mcpu=cortex-m4	-w
#CFLAGS 	+= -W -Wall 
#CFLAGS	+= 	--std=gnu99 
CFLAGS	+= 	-DUSE_STDPERIPH_DRIVER

# supaya pakai vfp dan calling yang benar, jika tidak, akan pakai stdlib yang emulasi		
#CFLAGS  += -mfpu=vfp -mfloat-abi=hard 
#CFLAGS  += -mfpu=vfp -mfloat-abi=softfp 
CFLAGS  += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O

#hati2 dengan optimization, delay_loop bisa tidak berfungsi !!

#CFLAGS+=-ffreestanding -nostdlib
#CFLAGS+=-ffunction-sections -fdata-sections -mno-sched-prolog -fno-hosted
# membuang code yang tidak perlu
#CFLAGS  += -Wl,--gc-sections


#source assembly
#ARMS=../firmware_set/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/gcc_ride7/


#CFLAGS+=-I../../../Projects/STM32F4-Discovery_FW_V1.1.0/Utilities/STM32F4-Discovery
CFLAGS	+=	-I../firmware_set/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS	+=	-I../firmware_set/Libraries/CMSIS/Include
CFLAGS	+=	-I../firmware_set/Utilities/STM32F4-Discovery
CFLAGS	+=	-I../firmware_set/Libraries/STM32F4xx_StdPeriph_Driver/inc
CFLAGS 	+= 	-I.
CFLAGS	+=  -I./include

# MULAI INCLUDE LWIP
CFLAGS  +=  -I./lwip-1.4.1/src/include
CFLAGS	+=  -I./lwip-1.4.1/port/STM32F4x7
CFLAGS	+=  -I./lwip-1.4.1
CFLAGS	+= 	-I./lwip-1.4.1/src/include/ipv4
#ASFLAGS  =	-mfpu=vfp -mfloat-abi=hard
#ASFLAGS  =	-mfpu=vfp -mfloat-abi=softfp

# to run from FLASH
CFLAGS	+= -Wl,-T,stm32_flash.ld
CFLAGS  += -D PAKAI_ETH -D PAKAI_ENCX24J600
CFLAGS	+= -ffunction-sections -fdata-sections -Wl,--cref,--gc-sections 

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(CP) -O binary $^ $@
	$(DD) -D $^ > disam
	$(SIZE) $^

#CLAIN = ../firmware_set/Utilities/STM32F4-Discovery/stm32f4_discovery.c

CLAIN = awal.c timer.c spi.c newlib_stubs.c
CLAIN += printf.c
CLAIN += ads_1274.c
CLAIN += i2c.c ltc_6904.c
CLAIN += fsmc_sram.c
CLAIN += flash.c
CLAIN += gain.c

#CLAIN += udp_callback.c

ETH_SOURCE = 	./enc/enc424j600.c	\
				./enc/enc424j600_2.c

#				sistem_uip.c	\

				
#ETH_SOURCE = 	../uip/hardware/enc624.c	\
#				sistem_uip.c


LWIP_DIR = ./lwip-1.4.1

LWIP_CORE	 = $(LWIP_DIR)/src/core/init.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/def.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/dhcp.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/dns.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/mem.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/memp.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/netif.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/pbuf.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/raw.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/stats.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/sys.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/tcp.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/tcp_in.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/tcp_out.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/timers.c
LWIP_CORE	+= $(LWIP_DIR)/src/core/udp.c


LWIP_API  = $(LWIP_DIR)/src/api/api_lib.c
LWIP_API += $(LWIP_DIR)/src/api/tcpip.c
LWIP_API += $(LWIP_DIR)/src/api/api_msg.c
LWIP_API += $(LWIP_DIR)/src/api/err.c
LWIP_API += $(LWIP_DIR)/src/api/netbuf.c
LWIP_API += $(LWIP_DIR)/src/api/netdb.c
LWIP_API += $(LWIP_DIR)/src/api/netifapi.c
#LWIP_API += $(LWIP_DIR)/src/api/dhcp.c
LWIP_API += $(LWIP_DIR)/src/api/sockets.c


LWIP_IP  = $(LWIP_DIR)/src/core/ipv4/autoip.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/icmp.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/igmp.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/inet.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/inet_chksum.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/ip.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/ip_addr.c
LWIP_IP += $(LWIP_DIR)/src/core/ipv4/ip_frag.c
LWIP_IP += $(LWIP_DIR)/src/netif/etharp.c


LWIP_APP  = ./lwip_app/netconf.c		
LWIP_APP += ./lwip_app/ethernetif.c
		
# $(addprefix lwip/contrib/port/FreeRTOS/MCF5235/, sys_arch.c netif/fec.c netif/nbuf.c) 		
		
			

#LWIP_SOURCE = $(LWIP_CORE) $(LWIP_API) $(LWIP_IP) $(LWIP_NETIF)

CLAIN 	+= $(ETH_SOURCE)
#CLAIN	+= $(UIP_SOURCE_10)
CLAIN	+= $(LWIP_API) $(LWIP_IP) $(LWIP_CORE) 
CLAIN	+= sistem_lwip.c $(LWIP_APP)

LIBS	= -L../firmware_set/Libraries/STM32F4xx_StdPeriph_Driver/buildku -lSTM32F4xx_StdPeriph_Driver
LIBS 	+= -lc -lm -lgcc

LFLAGS 	= -Xlinker -M -Xlinker -Map=rtosdemo.map

#$(EXECUTABLE): main.c system_stm32f4xx.c $(ARMS)startup_stm32f4xx.s stm32f4xx_it.c $(CLAIN)

$(EXECUTABLE): main.c system_stm32f4xx.c startup_stm32f4xx.s stm32f4xx_it.c $(CLAIN)
	$(CC) $(CFLAGS)  $^ -o $@ $(LIBS) $(LFLAGS)


clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

tulis :
	$(PENULIS) write $(BIN_IMAGE) 0x08000000

suntik :
	$(PENULIS) write $(BIN_IMAGE) 0x08000000

.PHONY: all clean
