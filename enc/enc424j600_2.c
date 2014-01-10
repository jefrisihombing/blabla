/******************************************
 * Title        : Microchip ENCX24J600 Ethernet Interface Driver
 * Author       : Jiri Melnikov
 * Created      : 29.03.2010
 * Version      : 0.2
 * Target MCU   : Atmel AVR series
 *
 * Description  : * This driver provides initialization and transmit/receive
 *                  functions for the Microchip ENCX24J600 100Mb Ethernet
 *                  Controller and PHY.
 *                * As addition, userspace access and hardware checksum
 *                  functions are available.
 *                * Only supported interface is SPI, no PSP interface available
 *                  by now.
 *                * No security functions are supported by now.
 *
 *                * This driver is inspired by ENC28J60 driver from Pascal
 *                  Stang (2005).
 *
 *                * Some lines of code are rewritten from Microchip's TCP/IP
 *                  stack.
 * 
 * ****************************************/

/* 25 Juni 2013, porting untuk STM32F */

#include "enc424j600.h"
#include "enc424j600conf.h"

#include "./include/mainku.h"

#define debug_printf	do{} while(0);

// Binary constant identifiers for ReadMemoryWindow() and WriteMemoryWindow()
// functions
#define UDA_WINDOW		(0x1)
#define GP_WINDOW		(0x2)
#define RX_WINDOW		(0x4)

// Promiscuous mode, uncomment if you want to receive all packets, even those which are not for you
// #define PROMISCUOUS_MODE

// Auto answer to ICMP requests, uncomment if you want to ICMP echo packets be managed by hardware.
// (if ENC2_PacketReceive is called and ICMP packet is waiting, function returns null lenght and provides automatic response)
// #define AUTO_ICMP_ECHO

// Hardware checksum computation
#define HARDWARE_CHECKSUM  //Comment to disable automatic hardware checksum in ip/icmp/tcp/udp packets
#ifdef HARDWARE_CHECKSUM

// #define HARDWARE_CHECKSUM_NULL  //Comment to disable cleraing checksums
#define IP_PROTOCOL1 0x08
#define IP_PROTOCOL2 0x00
#define ICMP_PROTOCOL 0x01
#define TCP_PROTOCOL 0x06
#define UDP_PROTOCOL 0x11
#define ETH_HEADER 14
#define IP_PROTOCOL_POS 23

#endif

// Internal MAC level variables and flags.
static u08 currentBank;
static u16 nextPacketPointer;

// Static functions
static void ENC2_SendSystemReset(void);

static bool ENC2_MACIsTxReady(void);
//static void ENC2_MACFlush(void);
void ENC2_MACFlush(void);

static u16 ENC2_ChecksumCalculation(u16 position, u16 length, u16 seed);

static void ENC2_WriteMemoryWindow(u08 window, u08 *data, u16 length);
static void ENC2_ReadMemoryWindow(u08 window, u08 *data, u16 length);

static u16 ENC2_ReadReg(u16 address);
static void ENC2_WriteReg(u16 address, u16 data);
static u16 ENC2_ReadPHYReg(u08 address);
static void ENC2_WritePHYReg(u08 address, u16 Data);
static void ENC2_ReadN(u08 op, u08* data, u16 dataLen);
static void ENC2_WriteN(u08 op, u08* data, u16 dataLen);
static void ENC2_BFSReg(u16 address, u16 bitMask);
static void ENC2_BFCReg(u16 address, u16 bitMask);

static void ENC2_ExecuteOp0(u08 op);
static u08 ENC2_ExecuteOp8(u08 op, u08 data);
static u16 ENC2_ExecuteOp16(u08 op, u16 data);
static u32 ENC2_ExecuteOp32(u08 op, u32 data);


unsigned int cek_paket_2(void)
{
	//printf("%s():\r\n", __FUNCTION__);
	if (GPIO_ReadInputDataBit( GPIO_ETHER_INT2, BIT_INT_ENC2 ))
		return 0;
	else
		return 1;
}

/********************************************************************
 * INITIALIZATION
 * ******************************************************************/
void ENC2_Init(void) {

	debug_entry;
	
    //Set default bank
    currentBank = 0;

   	init_SPI2();

   	DeassertChipSelect_2();
	ENCX24J600_Reset();
	delay_loop_ms (100);
	ENCX24J600_Unreset ();

    // Perform a reliable reset
    ENC2_SendSystemReset();

    // Initialize RX tracking variables and other control state flags
    nextPacketPointer = RXSTART;

    // Set up TX/RX/UDA buffer addresses
    ENC2_WriteReg(ETXST, TXSTART);
    ENC2_WriteReg(ERXST, RXSTART);
    ENC2_WriteReg(ERXTAIL, RAMSIZE - 2);
    ENC2_WriteReg(EUDAST, USSTART);
    ENC2_WriteReg(EUDAND, USEND);

    // If promiscuous mode is set, than allow accept all packets
#ifdef PROMISCUOUS_MODE
    ENC2_WriteReg(ERXFCON, (ERXFCON_CRCEN | ERXFCON_RUNTEN | ERXFCON_UCEN | ERXFCON_NOTMEEN | ERXFCON_MCEN));
    dd
#endif

    // Set PHY Auto-negotiation to support 10BaseT Half duplex,
    // 10BaseT Full duplex, 100BaseTX Half Duplex, 100BaseTX Full Duplex,
    // and symmetric PAUSE capability
    ENC2_WritePHYReg(PHANA, PHANA_ADPAUS0 | PHANA_AD10FD | PHANA_AD10 | PHANA_AD100FD | PHANA_AD100 | PHANA_ADIEEE0);

    // Enable RX packet reception
    ENC2_BFSReg(ECON1, ECON1_RXEN);

	// KUKU, supaya GPIO aktif untuk deteck packet 
    ENC2_WriteReg(EIE, EIE_INTIE | EIE_LINKIE | EIE_PKTIE | EIE_TXABTIE | EIE_RXABTIE);

    debug_leave;
}

/********************************************************************
 * PACKET TRANSMISSION
 * ******************************************************************/

/**
 * Recieves packet
 * */
u16 ENC2_PacketReceive(u16 len, u08* packet) {
    u16 newRXTail;
    RXSTATUS statusVector;

    if (!(ENC2_ReadReg(EIR) & EIR_PKTIF)) {
        return FALSE;
    }

#ifdef AUTO_ICMP_ECHO  //Check if packet is ICMP echo packet and answer to it automaticaly
kkjk
    //Set buffer for packet data
    u08 packetData[2];
    // Set the RX Read Pointer to the beginning of the next unprocessed packet + statusVektor + nextPacketPointer + position where paket type is saved
    ENC2_WriteReg(ERXRDPT, nextPacketPointer + sizeof (statusVector) + ETH_HEADER);

    //Read type of paket first, if it's IP
    ENC2_ReadMemoryWindow(RX_WINDOW, packetData, sizeof (packetData));
    if (packetData[0] == IP_PROTOCOL1 && packetData[1] == IP_PROTOCOL2) {
        //Ok, it's ip packet, check if it's icmp packet
        ENC2_WriteReg(ERXRDPT, nextPacketPointer + 2 + sizeof (statusVector) + IP_PROTOCOL_POS);
        ENC2_ReadMemoryWindow(RX_WINDOW, packetData, 1);
        if (packetData[0] == ICMP_PROTOCOL) {
            //It's icmp packet, read lenght and do DMA copy operation from recieve buffer to transmit buffer
            ENC2_WriteReg(ERXRDPT, nextPacketPointer + 2);
            ENC2_ReadMemoryWindow(RX_WINDOW, packetData, 2);
            if (*(u16*) packetData < 1522) {
                //Now do DMA copy, first read length from IP packet
                u16 ipPacketLen;
                u08 ipHeaderLen;
                ENC2_WriteReg(ERXRDPT, nextPacketPointer + 2 + sizeof (statusVector) + 14);
                ENC2_ReadMemoryWindow(RX_WINDOW, (u08*) & ipHeaderLen, 1);
                ipHeaderLen = (ipHeaderLen & 15)*4;
                ENC2_WriteReg(ERXRDPT, nextPacketPointer + 2 + sizeof (statusVector) + 16);
                ENC2_ReadMemoryWindow(RX_WINDOW, (u08*) & ipPacketLen, 2);
                ipPacketLen = HTONS(ipPacketLen);
                //Wait until controler is ready
                while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
                }
                //Set DMA copy and no checksum while copying (checksum computing at the end will be faster)
                //Switch MAC addr
                ENC2_BFSReg(ECON1, ECON1_DMACPY);
                ENC2_BFSReg(ECON1, ECON1_DMANOCS);
                ENC2_WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 6); //Switch MAC addr in packet
                ENC2_WriteReg(EDMADST, TXSTART);
                ENC2_WriteReg(EDMALEN, 6);
                ENC2_BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
                }
                ENC2_WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector)); //Switch MAC addr in packet
                ENC2_WriteReg(EDMADST, TXSTART + 6);
                ENC2_WriteReg(EDMALEN, 6);
                ENC2_BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
                }
                ENC2_WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 12); //Copy packet
                ENC2_WriteReg(EDMADST, TXSTART + 12);
                ENC2_WriteReg(EDMALEN, ipPacketLen + 2);
                ENC2_BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
                }
                //Switch IP addr
                ENC2_WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 26); //Switch IP addr in packet
                ENC2_WriteReg(EDMADST, TXSTART + 30);
                ENC2_WriteReg(EDMALEN, 4);
                ENC2_BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
                }
                ENC2_WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 30); //Switch IP addr in packet
                ENC2_WriteReg(EDMADST, TXSTART + 26);
                ENC2_WriteReg(EDMALEN, 4);
                ENC2_BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
                }
                //Change echo request to echo reply
                packetData[0] = 0;
                packetData[1] = 0;
                ENC2_WriteReg(EGPWRPT, 34);
                ENC2_WriteMemoryWindow(GP_WINDOW, packetData, 1);
                ENC2_WriteReg(EGPWRPT, 36);
                ENC2_WriteMemoryWindow(GP_WINDOW, packetData, 2);
                //Compute checksum (use packetData for mem saving)
                *(u16*) packetData = ENC2_ChecksumCalculation(ETH_HEADER + ipHeaderLen, ipPacketLen - ipHeaderLen, 0x0000);
                //Write it to the packet
                ENC2_WriteReg(EGPWRPT, 36);
                ENC2_WriteMemoryWindow(GP_WINDOW, packetData, 2);
                //Flush packet out
                ENC2_WriteReg(ETXLEN, ipPacketLen + ETH_HEADER);
                ENC2_MACFlush();
            }
            ENC2_WriteReg(ERXRDPT, nextPacketPointer);
            ENC2_ReadMemoryWindow(RX_WINDOW, (u08*) & nextPacketPointer, sizeof (nextPacketPointer));
            newRXTail = nextPacketPointer - 2;
            //Special situation if nextPacketPointer is exactly RXSTART
            if (nextPacketPointer == RXSTART)
                newRXTail = RAMSIZE - 2;
            //Packet decrement
            ENC2_BFSReg(ECON1, ECON1_PKTDEC);
            //Write new RX tail
            ENC2_WriteReg(ERXTAIL, newRXTail);
            //
            return 0;
        }
    } 
#endif

    // Set the RX Read Pointer to the beginning of the next unprocessed packet
    ENC2_WriteReg(ERXRDPT, nextPacketPointer);
    //printf("next %X, size %d\n", nextPacketPointer, sizeof (nextPacketPointer));

    ENC2_ReadMemoryWindow(RX_WINDOW, (u08*) & nextPacketPointer, sizeof (nextPacketPointer));

    ENC2_ReadMemoryWindow(RX_WINDOW, (u08*) & statusVector, sizeof (statusVector));
    len = (statusVector.bits.ByteCount <= len+4) ? statusVector.bits.ByteCount-4 : 0;

    ENC2_ReadMemoryWindow(RX_WINDOW, packet, len);

    newRXTail = nextPacketPointer - 2;
    //Special situation if nextPacketPointer is exactly RXSTART
    if (nextPacketPointer == RXSTART)
        newRXTail = RAMSIZE - 2;

    //Packet decrement
    ENC2_BFSReg(ECON1, ECON1_PKTDEC);

    //Write new RX tail
    ENC2_WriteReg(ERXTAIL, newRXTail);
	
    return len;
}

/**
 * Sends packet
 * */
void ENC2_PacketSend(u16 len, u08* packet) {
	debug_printf(" *** SEND %d *** ", len);
    // Set the Window Write Pointer to the beginning of the transmit buffer
    ENC2_WriteReg(EGPWRPT, TXSTART);

#if 0
#ifdef HARDWARE_CHECKSUM_NULL
    // Is it the IP packet? If so, for sure null checksum a let hardware to compute it
    if (packet[12] == IP_PROTOCOL1 && packet[13] == IP_PROTOCOL2) {
        //clear IP checksum
        packet[24] = 0;
        packet[25] = 0;
        //we can also compute icmp/tcp/udp messages
        if (packet[IP_PROTOCOL_POS] == ICMP_PROTOCOL) {
            //clear ICMP checksum
            packet[36] = 0;
            packet[37] = 0;
        } else if (packet[IP_PROTOCOL_POS] == TCP_PROTOCOL) {
            //clear TCP checksum
            packet[50] = 0;
            packet[51] = 0;
        } else if (packet[IP_PROTOCOL_POS] == UDP_PROTOCOL) {
            //clear UDP checksum
            packet[40] = 0;
            packet[41] = 0;
        }
    }
    ff
#endif
#endif
    ENC2_WriteMemoryWindow(GP_WINDOW, packet, len);
#if 0
#ifdef HARDWARE_CHECKSUM
    // Is it the IP packet? Get it computed by hardware
    if (packet[12] == IP_PROTOCOL1 && packet[13] == IP_PROTOCOL2) {
        //Compute header length
        u08 headerLen = (packet[ETH_HEADER] & 15)*4;
        //Compute checksum of IP header
        u16 checksum = ENC2_ChecksumCalculation(ETH_HEADER, headerLen, 0x0000);
        //Write it to correct position
        ENC2_WriteReg(EGPWRPT, 24);
        ENC2_WriteMemoryWindow(GP_WINDOW, ((u08*) & checksum), 2);

        //we can also compute icmp/tcp/udp messages
        if (packet[IP_PROTOCOL_POS] == ICMP_PROTOCOL) { /*ICMP*/
            //Compute header length
            u08 icmpLen = len - headerLen - ETH_HEADER;
            //Compute checksum of ICMP
            checksum = ENC2_ChecksumCalculation(ETH_HEADER + headerLen, icmpLen, 0x0000);
            //Write it to correct position
            ENC2_WriteReg(EGPWRPT, 36);
            ENC2_WriteMemoryWindow(GP_WINDOW, ((u08*) & checksum), 2);
        } else if (packet[IP_PROTOCOL_POS] == TCP_PROTOCOL || packet[IP_PROTOCOL_POS] == UDP_PROTOCOL) { /*TCP or UDP*/
            //Compute header length
            u16 upperLayerLen = len - headerLen - ETH_HEADER;

            //Compute checksum of TCP or UDP
            checksum = ~(HTONS(packet[IP_PROTOCOL_POS] + upperLayerLen)); //HTONS macro is from uIP
            checksum = ENC2_ChecksumCalculation(26, 8, checksum);
            checksum = ENC2_ChecksumCalculation(ETH_HEADER + headerLen, upperLayerLen, checksum);

            //Write it to correct position
            if (packet[IP_PROTOCOL_POS] == TCP_PROTOCOL) {
                ENC2_WriteReg(EGPWRPT, 50);
            } else {
                ENC2_WriteReg(EGPWRPT, 40);
            }
            ENC2_WriteMemoryWindow(GP_WINDOW, ((u08*) & checksum), 2);
        }
    }
#endif
#endif
    ENC2_WriteReg(ETXLEN, len);
    ENC2_MACFlush();
}

/**
 * Reads MAC address of device
 * */
void ENC2_ReadMacAddr(u08 * macAddr) {
    // Get MAC adress
    u16 regValue;
    regValue = ENC2_ReadReg(MAADR1);
    *macAddr++ = ((u08*) & regValue)[0];
    *macAddr++ = ((u08*) & regValue)[1];
    regValue = ENC2_ReadReg(MAADR2);
    *macAddr++ = ((u08*) & regValue)[0];
    *macAddr++ = ((u08*) & regValue)[1];
    regValue = ENC2_ReadReg(MAADR3);
    *macAddr++ = ((u08*) & regValue)[0];
    *macAddr++ = ((u08*) & regValue)[1];

    u16 w = ENC2_ReadReg(MACON2);
}

/**
 * Sets MAC address of device
 * */
void ENC2_SetMacAddr(u08 * macAddr) {
    u16 regValue;
    ((u08*) & regValue)[0] = *macAddr++;
    ((u08*) & regValue)[1] = *macAddr++;
    ENC2_WriteReg(MAADR1, regValue);
    ((u08*) & regValue)[0] = *macAddr++;
    ((u08*) & regValue)[1] = *macAddr++;
    ENC2_WriteReg(MAADR2, regValue);
    ((u08*) & regValue)[0] = *macAddr++;
    ((u08*) & regValue)[1] = *macAddr++;
    ENC2_WriteReg(MAADR3, regValue);
}

/**
 * Enables powersave mode
 * */
void ENC2_PowerSaveEnable(void) {
    //Turn off modular exponentiation and AES engine
    ENC2_BFCReg(EIR, EIR_CRYPTEN);
    //Turn off packet reception
    ENC2_BFCReg(ECON1, ECON1_RXEN);
    //Wait for any in-progress receptions to complete
    while (ENC2_ReadReg(ESTAT) & ESTAT_RXBUSY) {
        _delay_us(100);
    }
    //Wait for any current transmisions to complete
    while (ENC2_ReadReg(ECON1) & ECON1_TXRTS) {
        _delay_us(100);
    }
    //Power-down PHY
    u16 state;
    state = ENC2_ReadPHYReg(PHCON1);
    ENC2_WritePHYReg(PHCON1, state | PHCON1_PSLEEP);
    //Power-down eth interface
    ENC2_BFCReg(ECON2, ECON2_ETHEN);
    ENC2_BFCReg(ECON2, ECON2_STRCH);
}

/**
 * Disables powersave mode
 * */
void ENC2_PowerSaveDisable(void) {
    //Wake-up eth interface
    ENC2_BFSReg(ECON2, ECON2_ETHEN);
    ENC2_BFSReg(ECON2, ECON2_STRCH);
    //Wake-up PHY
    u16 state;
    state = ENC2_ReadPHYReg(PHCON1);
    ENC2_WritePHYReg(PHCON1, state & ~PHCON1_PSLEEP);
    //Turn on packet reception
    ENC2_BFSReg(ECON1, ECON1_RXEN);
}

/**
 * Is link connected?
 * @return <bool>
 */
bool ENC2_IsLinked(void) {
    return (ENC2_ReadReg(ESTAT) & ESTAT_PHYLNK) != 0u;
}

/**
 * Saves data to userspace defined by USSTART & USEND
 * @return bool (true if saved, false if there is no space)
 * */
bool ENC2_SaveToUserSpace(u16 position, u08* data, u16 len) {
    if ((USSTART + position + len) > USEND) return false;
    ENC2_WriteReg(EUDAWRPT, USSTART + position);
    ENC2_WriteMemoryWindow(UDA_WINDOW, data, len);
    return true;
}

/**
 * Loads data from userspace defined by USSTART & USEND
 * @return bool (true if area is in userspace, false if asked area is out of userspace)
 * */
bool ENC2_ReadFromUserSpace(u16 position, u08* data, u16 len) {
    if ((USSTART + position + len) > USEND) return false;
    ENC2_WriteReg(EUDARDPT, USSTART + position);
    ENC2_ReadMemoryWindow(UDA_WINDOW, data, len);
    return true;
}

/********************************************************************
 * UTILS
 * ******************************************************************/

static void ENC2_SendSystemReset(void) 
{
	debug_entry;
	
	int try = 0;
	int try2 = 0;
	unsigned int s;
    // Perform a reset via the SPI/PSP interface
    do {
        // Set and clear a few bits that clears themselves upon reset.
        // If EUDAST cannot be written to and your code gets stuck in this
        // loop, you have a hardware problem of some sort (SPI or PMP not
        // initialized correctly, I/O pins aren't connected or are
        // shorted to something, power isn't available, etc.)
        // sbi(PORTE, PE7);
        do {
            ENC2_WriteReg(EUDAST, 0x1234);
            try++;
            if (try > 15) break;
            
        } while ( (ENC2_ReadReg(EUDAST) != 0x1234));

        s = ENC2_ReadReg(EUDAST);
		printf("Try : %d, EUDAST 0x%X\r\n", try, s);
        try = 0;
        
        // Issue a reset and wait for it to complete
        ENC2_BFSReg(ECON2, ECON2_ETHRST);
        currentBank = 0;
        while ((ENC2_ReadReg(ESTAT) & (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY)) != (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY));
        delay_loop_us(300);
        // Check to see if the reset operation was successful by
        // checking if EUDAST went back to its reset default.  This test
        // should always pass, but certain special conditions might make
        // this test fail, such as a PSP pin shorted to logic high.
		try2 ++;
		if (try2 > 25) break;
        
    } while (ENC2_ReadReg(EUDAST) != 0x0000u);

    s = ENC2_ReadReg(EUDAST);
    printf("Try2 : %d, 0x%X\r\n", try2, s);

    // Really ensure reset is done and give some time for power to be stable
    _delay_us(1000);
    
    debug_leave;
}

void ENC2_MACFlush_ku(void) {
    // Check to see if the duplex status has changed.  This can
    // change if the user unplugs the cable and plugs it into a
    // different node.  Auto-negotiation will automatically set
    // the duplex in the PHY, but we must also update the MAC
    // inter-packet gap timing and duplex state to match.
    if (ENC2_ReadReg(EIR) & EIR_LINKIF) {
        ENC2_BFCReg(EIR, EIR_LINKIF);

        u16 w;

        // Update MAC duplex settings to match PHY duplex setting
        w = ENC2_ReadReg(MACON2);
        if (ENC2_ReadReg(ESTAT) & ESTAT_PHYDPX) {
            // Switching to full duplex
            ENC2_WriteReg(MABBIPG, 0x15);
            w |= MACON2_FULDPX;
        } else {
            // Switching to half duplex
            ENC2_WriteReg(MABBIPG, 0x12);
            w &= ~MACON2_FULDPX;
        }
        ENC2_WriteReg(MACON2, w);
    }
}

/**
 * Is transmission active?
 * @return <bool>
 */
static bool ENC2_MACIsTxReady(void) {
    return !(ENC2_ReadReg(ECON1) & ECON1_TXRTS);
}

//static void ENC2_MACFlush(void) {
void ENC2_MACFlush(void) {
    // Check to see if the duplex status has changed.  This can
    // change if the user unplugs the cable and plugs it into a
    // different node.  Auto-negotiation will automatically set
    // the duplex in the PHY, but we must also update the MAC
    // inter-packet gap timing and duplex state to match.
    if (ENC2_ReadReg(EIR) & EIR_LINKIF) {
        ENC2_BFCReg(EIR, EIR_LINKIF);

        u16 w;

        // Update MAC duplex settings to match PHY duplex setting
        w = ENC2_ReadReg(MACON2);
        if (ENC2_ReadReg(ESTAT) & ESTAT_PHYDPX) {
            // Switching to full duplex
            ENC2_WriteReg(MABBIPG, 0x15);
            w |= MACON2_FULDPX;
        } else {
            // Switching to half duplex
            ENC2_WriteReg(MABBIPG, 0x12);
            w &= ~MACON2_FULDPX;
        }
        ENC2_WriteReg(MACON2, w);
    }


    // Start the transmission, but only if we are linked.
    if (ENC2_ReadReg(ESTAT) & ESTAT_PHYLNK)
        ENC2_BFSReg(ECON1, ECON1_TXRTS);
}

/**
 * Calculates IP checksum value
 *
 * */
static u16 ENC2_ChecksumCalculation(u16 position, u16 length, u16 seed) {
    // Wait until module is idle
    while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
    }
    // Clear DMACPY to prevent a copy operation
    ENC2_BFCReg(ECON1, ECON1_DMACPY);
    // Clear DMANOCS to select a checksum operation
    ENC2_BFCReg(ECON1, ECON1_DMANOCS);
    // Clear DMACSSD to use the default seed of 0000h
    ENC2_BFCReg(ECON1, ECON1_DMACSSD);
    // Set EDMAST to source address
    ENC2_WriteReg(EDMAST, position);
    // Set EDMALEN to length
    ENC2_WriteReg(EDMALEN, length);
    //If we have a seed, now it's time
    if (seed) {
        ENC2_BFSReg(ECON1, ECON1_DMACSSD);
        ENC2_WriteReg(EDMACS, seed);
    }
    // Initiate operation
    ENC2_BFSReg(ECON1, ECON1_DMAST);
    // Wait until done
    while (ENC2_ReadReg(ECON1) & ECON1_DMAST) {
    }
    return ENC2_ReadReg(EDMACS);
}

/********************************************************************
 * READERS AND WRITERS
 * ******************************************************************/

static void ENC2_WriteMemoryWindow(u08 window, u08 *data, u16 length) {
    u08 op = WBMUDA;

    if (window & GP_WINDOW)
        op = WBMGP;
    if (window & RX_WINDOW)
        op = WBMRX;

    ENC2_WriteN(op, data, length);
}

static void ENC2_ReadMemoryWindow(u08 window, u08 *data, u16 length) {
    if (length == 0u)
        return;

    u08 op = RBMUDA;

    if (window & GP_WINDOW)
        op = RBMGP;
    if (window & RX_WINDOW)
        op = RBMRX;

    ENC2_ReadN(op, data, length);
}

/**
 * Reads from address
 * @variable <u16> address - register address
 * @return <u16> data - data in register
 */
static u16 ENC2_ReadReg(u16 address) {
    u16 returnValue;
    u08 bank;

    // See if we need to change register banks
    bank = ((u08) address) & 0xE0;
    //If address is banked, we will use banked access
    if (bank <= (0x3u << 5)) {
        if (bank != currentBank) {
            if (bank == (0x0u << 5))
                ENC2_ExecuteOp0(B0SEL);
            else if (bank == (0x1u << 5))
                ENC2_ExecuteOp0(B1SEL);
            else if (bank == (0x2u << 5))
                ENC2_ExecuteOp0(B2SEL);
            else if (bank == (0x3u << 5))
                ENC2_ExecuteOp0(B3SEL);

            currentBank = bank;
        }
        returnValue = ENC2_ExecuteOp16(RCR | (address & 0x1F), 0x0000);
    } else {
        u32 returnValue32 = ENC2_ExecuteOp32(RCRU, (u32) address);
        ((u08*) & returnValue)[0] = ((u08*) & returnValue32)[1];
        ((u08*) & returnValue)[1] = ((u08*) & returnValue32)[2];
    }

    return returnValue;
}

/**
 * Writes to register
 * @variable <u16> address - register address
 * @variable <u16> data - data to register
 */
static void ENC2_WriteReg(u16 address, u16 data) {
    u08 bank;

    // See if we need to change register banks
    bank = ((u08) address) & 0xE0;
    //If address is banked, we will use banked access
    if (bank <= (0x3u << 5)) {
        if (bank != currentBank) {
            if (bank == (0x0u << 5))
                ENC2_ExecuteOp0(B0SEL);
            else if (bank == (0x1u << 5))
                ENC2_ExecuteOp0(B1SEL);
            else if (bank == (0x2u << 5))
                ENC2_ExecuteOp0(B2SEL);
            else if (bank == (0x3u << 5))
                ENC2_ExecuteOp0(B3SEL);

            currentBank = bank;
        }
        ENC2_ExecuteOp16(WCR | (address & 0x1F), data);
    } else {
        u32 data32;
        ((u08*) & data32)[0] = (u08) address;
        ((u08*) & data32)[1] = ((u08*) & data)[0];
        ((u08*) & data32)[2] = ((u08*) & data)[1];
        ENC2_ExecuteOp32(WCRU, data32);
    }

}

static u16 ENC2_ReadPHYReg(u08 address) {
    u16 returnValue;

    // Set the right address and start the register read operation
    ENC2_WriteReg(MIREGADR, 0x0100 | address);
    ENC2_WriteReg(MICMD, MICMD_MIIRD);

    // Loop to wait until the PHY register has been read through the MII
    // This requires 25.6us
    while (ENC2_ReadReg(MISTAT) & MISTAT_BUSY);

    // Stop reading
    ENC2_WriteReg(MICMD, 0x0000);

    // Obtain results and return
    returnValue = ENC2_ReadReg(MIRD);

    return returnValue;
}

static void ENC2_WritePHYReg(u08 address, u16 Data) {
    // Write the register address
    ENC2_WriteReg(MIREGADR, 0x0100 | address);

    // Write the data
    ENC2_WriteReg(MIWR, Data);

    // Wait until the PHY register has been written
    while (ENC2_ReadReg(MISTAT) & MISTAT_BUSY);
}

static void ENC2_ReadN(u08 op, u08* data, u16 dataLen) {
    // assert CS
	AssertChipSelect_2();
	
    // issue read command
   	spiPut_2( op );

    while (dataLen--) {
        // wait for answer
        *data++ = spiPut_2(0x00);
    }

    // release CS
    DeassertChipSelect_2();
}

static void ENC2_WriteN(u08 op, u08* data, u16 dataLen) {
    // assert CS
    AssertChipSelect_2();

    // issue read command
    spiPut_2( op );

    while (dataLen--) {
        // start sending data to SPI
        spiPut_2( *data++ );
    }

    // release CS
    DeassertChipSelect_2();
}

static void ENC2_BFSReg(u16 address, u16 bitMask) {
    u08 bank;

    // See if we need to change register banks
    bank = ((BYTE) address) & 0xE0;
    if (bank != currentBank) {
        if (bank == (0x0u << 5))
            ENC2_ExecuteOp0(B0SEL);
        else if (bank == (0x1u << 5))
            ENC2_ExecuteOp0(B1SEL);
        else if (bank == (0x2u << 5))
            ENC2_ExecuteOp0(B2SEL);
        else if (bank == (0x3u << 5))
            ENC2_ExecuteOp0(B3SEL);

        currentBank = bank;
    }

    ENC2_ExecuteOp16(BFS | (address & 0x1F), bitMask);
}

static void ENC2_BFCReg(u16 address, u16 bitMask) {
    u08 bank;

    // See if we need to change register banks
    bank = ((u08) address) & 0xE0;
    if (bank != currentBank) {
        if (bank == (0x0u << 5))
            ENC2_ExecuteOp0(B0SEL);
        else if (bank == (0x1u << 5))
            ENC2_ExecuteOp0(B1SEL);
        else if (bank == (0x2u << 5))
            ENC2_ExecuteOp0(B2SEL);
        else if (bank == (0x3u << 5))
            ENC2_ExecuteOp0(B3SEL);

        currentBank = bank;
    }

    ENC2_ExecuteOp16(BFC | (address & 0x1F), bitMask);
}
/********************************************************************
 * EXECUTES
 * ******************************************************************/

/**
 * Execute SPI operation
 * @variable <u08> op - operation
 */
static void ENC2_ExecuteOp0(u08 op) {
    u08 dummy;
    // assert CS
    AssertChipSelect_2();

    // issue read command
    spiPut_2( op );
    
    // release CS
    DeassertChipSelect_2();
}

/**
 * Write data to SPI with operation
 * @variable <u08> op - SPI operation
 * @variable <u08> data - data
 */
static u08 ENC2_ExecuteOp8(u08 op, u08 data) {
    u08 returnValue;
    // assert CS
    AssertChipSelect_2();

#if 0
    // issue write command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // start sending data to SPI
    SPDR = data;
    // wain until all is sent
    while (!(SPSR & (1 << SPIF)));
    // read answer
    returnValue = SPDR;
#endif
	spiPut_2( op );
	returnValue = spiPut_2( data );
    
    // release CS
    DeassertChipSelect_2();

    return returnValue;
}

/**
 * Write data to SPI with operation
 * @variable <u08> op - SPI operation
 * @variable <u16> data - data
 */
static u16 ENC2_ExecuteOp16(u08 op, u16 data) {
    u16 returnValue;
    // assert CS
    // zabereme sbernici
    AssertChipSelect_2();

	#if 0
    // issue write command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // in this cycle, data are sent
    for (int x = 0; x < 2; x++) {
        // start sending data to SPI
        SPDR = ((u08*) & data)[x];
        // wain until all is sent
        while (!(SPSR & (1 << SPIF)));
        // read answer
        ((u08*) & returnValue)[x] = SPDR;
    }
	#endif
	spiPut_2( op );
	((u08 *) &returnValue)[ 0 ] = spiPut_2( ((u08*) & data) [0] );
	((u08 *) &returnValue)[ 1 ] = spiPut_2( ((u08*) & data) [1] );    
	
    // release CS
     DeassertChipSelect_2();

    return returnValue;
}

/**
 * Write data to SPI with operation
 * @variable <u08> op - SPI operation
 * @variable <u32> data - data
 */
static u32 ENC2_ExecuteOp32(u08 op, u32 data) 
{
    u32 returnValue;
    // assert CS
    //ENC2__CONTROL_PORT &= ~(1 << ENC2__CONTROL_CS);
	 AssertChipSelect_2();

	 #if 0
    // issue write command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // in this cycle, data are sent
    for (int x = 0; x < 3; x++) {
        // start sending data to SPI
        SPDR = ((u08*) & data)[x];
        // wain until all is sent
        while (!(SPSR & (1 << SPIF)));
        // read answer
        ((u08*) & returnValue)[x] = SPDR;
    }
    #endif
    spiPut_2( op );
	((u08 *) &returnValue)[ 0 ] = spiPut_2( ((u08*) &data) [ 0 ] );
	((u08 *) &returnValue)[ 1 ] = spiPut_2( ((u08*) &data) [ 1 ] );  
	((u08 *) &returnValue)[ 2 ] = spiPut_2( ((u08*) &data) [ 2 ] );  

    // release CS
    //ENC2__CONTROL_PORT |= (1 << ENC2__CONTROL_CS);
    DeassertChipSelect_2();

    return returnValue;
}
