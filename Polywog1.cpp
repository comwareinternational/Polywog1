/*----------------------------------------------------------------------*
 *	Library to support polywog protocol. Implementation using the	*
 *	nRF24L01+ family of transceivers using the SPI interface.	*
 *									*
 *	Comware International invests time and resources by providing	*
 *	this open source code. Please support Comware by purchasing	*
 *	products or consulting services from us.			*
 *									*
 *	Written by Dave Retz of Comware International.			*
 *	BSD license, all text above must be included in any redist-	*
 *	ribution. 							*
 *::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*
 *	Version Information						*
 *	Ver	By	Date		Description			*
 *	1.0	DLR	20170807	Initial Release			*
 *	1.0a	DLR	20170811	State Standby-I on init		*
 *	1.0b	DLR	20170813	Added setChannel(v)		*
 *									*
 *	This adheres to polywog protocol as defined in specification	*
 *	Version 1, Revision 3. (v1r3).					*
 *----------------------------------------------------------------------*/
 
#define VERSION "1.0b"		// Ver.Rev + (edit sequence code)
#define DEBUG_SERIAL 0		// debug options

#include "Polywog1.h"

// changed for compatibility with w5100 ethernet adapter
// SPISettings spiSet(10000000, MSBFIRST, SPI_MODE0);

  SPISettings spiSet(4000000, MSBFIRST, SPI_MODE0);


#if DEBUG_SERIAL
byte readReg(int reg)
  {
  byte rv;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(reg);
  rv = SPI.transfer(0xFF);
  digitalWrite(CSN, HIGH);
  return (rv);   
  }

// read multiple bytes from a "register"
// c is number of bytes

void readMReg(int reg, byte *buf, int c)
  {
  int j;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(reg | 0x00);
  for (j=0; j<c; j++)
    {
    buf[j] = SPI.transfer(0xFF);  
    }
  digitalWrite(CSN, HIGH);
  }
#endif



void Polywog1::writeReg(int reg, int value)
  {
  digitalWrite(CSN, LOW);
  SPI.transfer(reg | 0x20);
  SPI.transfer(value);
  digitalWrite(CSN, HIGH);
  }

byte Polywog1::readReg(int reg)
  {
  byte rv;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(reg);
  rv = SPI.transfer(0xFF);
  digitalWrite(CSN, HIGH);
  return (rv);   
  }

// read multiple bytes from a "register"
// c is number of bytes

void Polywog1::readMReg(int reg, byte *buf, int c)
  {
  int j;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(reg | 0x00);
  for (j=0; j<c; j++)
    {
    buf[j] = SPI.transfer(0xFF);  
    }
  digitalWrite(CSN, HIGH);
  }


// write multiple bytes to a "register"

void Polywog1::writeMReg(int reg, byte *buf, int c)
  {
  int j;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(reg | 0x20);
  for (j=0; j<c; j++)
    {
    SPI.transfer(buf[j]);  
    }
  digitalWrite(CSN, HIGH);
  }

/*----------------------------------------------------------------------*
 *	readMRXPayload - read c bytes into the specified buffer.	*
 *----------------------------------------------------------------------*/

void Polywog1::readMRXPayload(byte *buf, int c)
  {
  int j;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(0x61);		// command to read RX Payload
  for (j=0; j<c; j++)
    {
    buf[j] = SPI.transfer(0xFF); // NOP output, read data byte  
    }
  digitalWrite(CSN, HIGH);
  }


/*----------------------------------------------------------------------*
 *	flushTX, flushRX - clear TX and RX FIFO contents.		*
 *----------------------------------------------------------------------*/

 void Polywog1::flushTX()   // flush TX FIFO
   {
   digitalWrite(CSN, LOW);
   SPI.transfer(0xE1);
   digitalWrite(CSN, HIGH);
   }
 
 void Polywog1::flushRX()   // flush RX FIFO
   {
   digitalWrite(CSN, LOW);
   SPI.transfer(0xE2);
   digitalWrite(CSN, HIGH);
   }

/*----------------------------------------------------------------------*
 *	readPLWidth - read the number of bytes in the receive FIFO	*
 *	assumes nRF24L01+ radio.					*
 *----------------------------------------------------------------------*/

byte Polywog1::readPLWidth()
  {
  byte rv;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(0x60);     // read payload width via R_RX_PL_WID
  rv = SPI.transfer(0xFF);
  digitalWrite(CSN, HIGH);
  return (rv);   
  }

/*----------------------------------------------------------------------*
 *	writeMTXPayloadNoAck(byte ptr, int)				*
 *									*
 *	This function writes a packet to the nRF24L01+ radio FIFO.	*
 *----------------------------------------------------------------------*/

void Polywog1::writeMTXPayloadNoAck(byte *buf, int c)
  {
  int j;
  
  digitalWrite(CSN, LOW);
  SPI.transfer(0xB0);    // command to write TX Payload NO ACK
  for (j=0; j<c; j++)
    {
    SPI.transfer(buf[j]); // put the output bytes in the TX FIFO 
    }

  digitalWrite(CSN, HIGH);
  }



#if DEBUG_SERIAL

// dump a buffer out, in HEX
// assumes pointer to buffer, count
// requires DEBUG_SERIAL 1

void dumpbuf(byte *cp, int n)
  {
  byte c;

  for (;n;n--)
    {
    c = *cp++;
    Serial.print(c, HEX); Serial.print(", ");
    }
  Serial.println();
  }


void dumpregs()
  {
  int i; byte v;
  byte vbuf[5];         // for variable-sized entries (addresses)

  Serial.println("Register dump:");
  for (i=0; i<0x0A; i++)
    {
    Serial.print("Reg[0x");
    Serial.print(i, HEX);
    Serial.print("] = ");
    v = readReg(i);
    Serial.println(v, HEX);    
    }
  readMReg(0x0A, vbuf, 5);      // Pipe 0 RX address
  Serial.print("RX Pipe 0 Address (5 bytes): ");
  dumpbuf(vbuf, 5);
  
  readMReg(0x0B, vbuf, 5);      // Pipe 1 RX address
  Serial.print("RX Pipe 1 Address (5 bytes): ");
  dumpbuf(vbuf, 5);

  for (i=0x0C; i<0x10; i++) // more addresses, 1 byte each
    {
    Serial.print("Reg[0x");
    Serial.print(i, HEX);
    Serial.print("] = ");
    v = readReg(i);
    Serial.println(v, HEX);    
    }

  readMReg(0x10, vbuf, 5);      // TX address
  Serial.print("TX Address (5 bytes): ");
  dumpbuf(vbuf, 5);
  
 for (i=0x11; i<32; i++)
    {
    Serial.print("Reg[0x");
    Serial.print(i, HEX);
    Serial.print("] = ");
    v = readReg(i);
    Serial.println(v, HEX);    
    }

 
  Serial.println();
  }
#endif






/*----------------------------------------------------------------------*
 *	begin() - initialize radio according to network address in	*
 *	EEPROM setup.							*
 *----------------------------------------------------------------------*/

boolean Polywog1::begin(void)  {
  byte temp;

  polywog_eeprom.net_number = 0;	// default to net 0 (undefined)
// get general Polywog settings (net, node address and name)
  if (! polywog_eeprom_verify(&polywog_eeprom))
        return (0);

// nRF24L Initialization
  
  SPI.begin();
  pinMode(CSN, OUTPUT);         // this signal activates radio SPI interface
  pinMode(CE, OUTPUT);

  digitalWrite(CE, LOW);        // this signal keys the radio (TX or RX)	
  isListening = false;		// Initialized in Standby-I state

  pinMode(XMIT_LED, OUTPUT);
  digitalWrite(XMIT_LED, HIGH); // blink xmit LED 2x to show startup
  delay(100);
  digitalWrite(XMIT_LED, LOW);
  delay(100);
  digitalWrite(XMIT_LED, HIGH);
  delay(100);
  digitalWrite(XMIT_LED, LOW);

/*----------------------------------------------------------------------*
 *	Set initial register configuration, I'm a PRX for now.		*
 *									*
 *	Most of these parameters should match the receive side.		*
 *----------------------------------------------------------------------*/  
  
  SPI.beginTransaction(spiSet);

  writeReg(0, 0b00001101);  // reset by power down
  delay(5);                 // quiesce 5ms during start-up

  writeReg(1, 3);           // enable AA on pipes 0,1
  writeReg(2, 3);           // enable pipes 0, 1
  writeReg(3, 3);           // 5 byte addresses
  writeReg(4, 0);	   // ARQ Disabled
  writeReg(5, radioChannel);   // RF_CH frequency channel (out of the range: 0-127) 

// register 0x06 determines the power and the data rate
// set 0x20 if 250kbps

  writeReg(0x06, 0x26);        // 250kbps, 0dBm
  writeReg(7, 0x60);        // clear the status reg
 
 // setup node and network number for receive and broadcast addresses  

  a1[0] = polywog_eeprom.node_number;
  memcpy(&a1[1], &polywog_eeprom.net_number, 4);
  memcpy(&a2[1], &polywog_eeprom.net_number, 4);
  
  writeMReg(0x0A, a1, 5);   // my receive (RX) address
  writeMReg(0x0B, a2, 5);   // the broadcast address
  writeMReg(0x10, a3, 5);   // my transmit (TX) address (default)

  writeReg(0x11, 32);       // Write Receive payload width P0
  writeReg(0x12, 32);       // Write Receive payload width P1
                            // P2-P5 unused  
  
//  note: the following ONLY works if the auto-ack (AA) feature enabled!
  writeReg(0x1D, 5);        // Enable dynamic payload (EN_DPL)
  writeReg(0x1C, 3);        // Dynamic payload on P0 and P1 receives
//  writeReg(0x1C, 0);        // no dynamic puayload, thank you
//  writeReg(0x1D, 0);        // no special features (requires full 32 PL)
  
// Flush TX and RX FIFOs

  flushTX();                // flush TX FIFO
  flushRX();

// ok, now power up the radio and wait 5ms

  writeReg(0, 0b01111111);  // 0(1 bit), Mask ints 111(3 bits), 8-bit CRC 11(2 bits), PowerUp 1(1 bit), PRX 1(1 bit) 
  delay(5);                 // introduce 5 ms delay for Tpd2stby
//  digitalWrite(CE, HIGH);   // start listening

 #if DEBUG_SERIAL
 //Serial.println("On initialization:");
 //dumpregs();
 #endif


  SPI.endTransaction();  
  return (1);   	// success !
  }


/*----------------------------------------------------------------------*
 *      Polywog Functions                                               *
 *                                                                      *
 *	boolean								*
 *      xmit(PW_PACKET_PTR pp, unsigned int bc, byte nextHop    * 
 *                                                                      *
 *      Assumes a pointer to a polywog packet to be sent, and a byte    *
 *      count (including polywog header and 0-N bytes of data).         *
 *      This function sets the destination address as the TX Address,   *                                                                
 *      places the local radio into PTX mode, Emits a Packet, and       *
 *      waits for the packet to be transmitted as indicated by the      *
 *      TX FIFO being empty (or timeout). Timeout is enough time to     *
 *      transmit a full-size packet at the slowest bitrate (250Kbps).   *
 *      It then falls back to PRX mode, allowing packets to be received.*
 *                                                                      *
 *      nextHop specifies the next node address (may or may not be      *                                                                
 *      the destAddr in the polywog header). It is the next "hop" addr  *
 *      to arrive at the destination address.                           *
 *									*
 *	Returns: 0 (FALSE) if not successful (timeout), or 1 (TRUE)	*
 *	if packet transmitted ok.					*
 *----------------------------------------------------------------------*/

boolean Polywog1::xmit(PW_PACKET_PTR pp, unsigned int bc, byte nextHop)
 {
 uint32_t timer;	/* 32-bit timer for catching timeout */
 boolean retstatus;
 byte txaddr[5];

/* Form TX Address as 5-byte sequence consisting of my 4-byte net number followed by
 *  the node number. Assumes little-endian coding for address.
 */

#if DEBUG_SERIAL
// Serial.println("xmit invoked."); // debug dork
// dumpbuf((byte *) pp, bc);
 #endif
 
 if (isListening)
   {
   digitalWrite(CE, LOW);	// return to Standby-I state
   isListening = false;
   }
 
 digitalWrite(XMIT_LED, HIGH); // blink xmit ON

 SPI.beginTransaction(spiSet);

 // flushTX();

 txaddr[0] = nextHop;                /* destination address of packet */
 memcpy(&txaddr[1], &polywog_eeprom.net_number, 4); /* copy 4-byte net number */
 writeMReg(0x10, txaddr, 5);   // my transmit (TX) address

 writeReg(0x7, 0x20);   // clear TX_DS and MAX_RT status
 writeReg(0x7, 0x10);   // clear TX_DS and MAX_RT status

/* Now place the radio into PTX mode - config reg bit 0 <= 0 */

 writeReg(0, (readReg(0) & 0xFE)); /* reset bit to enter PTX state */
 delay(5);              /* delay for enter PTX */
 
/* Now put stuff into the TX FIFO (no ACK) */

 writeMTXPayloadNoAck( (byte *) pp, (int) bc);

/* Now key the transmitter for 1ms */

 digitalWrite(CE, HIGH);
 delay(1);
 digitalWrite(CE, LOW);

/*----------------------------------------------------------------------*
 *	Transmission is in progress. Now wait for either FIFO to be MT, *
 *	or for the timeout to occur. If the latter, return error (1).	*
 *----------------------------------------------------------------------*/

 timer = millis();	/* Get current milliseconds */
 retstatus = 0;         /* default to NO error condition */
 while ((readReg(0x07) & 0x20) ==0) /* Data not sent yet (DS=0) */
   {
   if ( millis() - timer > TX_TIMEOUT ) /* delay up to TX_TIMEOUT ms. */
     {
     retstatus = 1;     // error: timeout status
     break;
     }  
   }
   
 writeReg(0x07, 0x20);	// force DS bit clear

/* Now, regardless of return status, put this back into PRX mode */

#if DEBUG_SERIAL
//  Serial.print("xmit return status: "); Serial.println(retstatus);
//  dumpregs();
#endif  

// writeReg(0, (readReg(0) | 0x01)); /* set the PRX bit */
 SPI.endTransaction();

// debug dork:
// digitalWrite(CE, HIGH);	// resume listening
 digitalWrite(XMIT_LED, LOW); // blink xmit OFF
 
 return(retstatus);
 }


 /*----------------------------------------------------------------------*
 *      int avail()		                                        * 
 *                                                                      *
 *      This function tests to see if bytes are available in one of the *
 *      receive FIFOs. If so, it returns the number of bytes to be read.*
 *----------------------------------------------------------------------*/

int Polywog1::avail(void)
    {
    byte status, plw;

    SPI.beginTransaction(spiSet);

    if (! isListening)	// if not listening already,
      {
      writeReg(0, (readReg(0) | 0x01)); /* enter PRX State */
      isListening = true;
#if DEBUG_SERIAL
//Serial.println("Entering PRX State.");
//dumpregs();
#endif
      digitalWrite(CE, HIGH);	// resume listening

      }

    status = readReg(0x07);
       
    plw = 0;		/* default 0 bytes received */
    if (status & 0x40)  // data ready ? got something!
      {
      writeReg(7, 0x40);    // clear DR bit
      plw = readPLWidth();

#if DEBUG_SERIAL
Serial.print("PLW="); Serial.println(plw);
#endif
      }

    SPI.endTransaction();
    return (plw);
    }

/*----------------------------------------------------------------------*
 *      int recv(PW_PACKET_PTR pp, unsigned int bc)			* 
 *                                                                      *
 *      This function tests to see if bytes are available in one of the *
 *      receive FIFOs. If so, it returns the number of bytes to be read.*
 *----------------------------------------------------------------------*/

int Polywog1::recv(PW_PACKET_PTR pp, unsigned int bc)
  {
  readMRXPayload((byte *) pp, bc);
  return (0);
  }

/*----------------------------------------------------------------------*
 *      byte my_node(void)						* 
 *                                                                      *
 *      This function obtains the 8-bit node number in the local net	*
 *	of net_number.							*
 *----------------------------------------------------------------------*/

byte Polywog1::my_node(void)
  {
  return(polywog_eeprom.node_number);
  }
  
/*----------------------------------------------------------------------*
 *      unsigned int Polywog1::my_network(void)				* 
 *                                                                      *
 *      This function obtains the network number from eeprom data.	*
 *----------------------------------------------------------------------*/

unsigned long Polywog1::my_network(void)
  {
  return(polywog_eeprom.net_number);
  }

/*----------------------------------------------------------------------*
 *      unsigned int Polywog1::setChannel(void)				*
 *                                                                      *
 *      This function sets the radio channel number, which starts at 0	*
 *	and is the center frequency equal to 2400 Mhz + channel number,	*
 *	in MHz. That is, Channel 4 would be 2404 Mhz (default).		*
 *----------------------------------------------------------------------*/

void Polywog1::setChannel(int channel)
  {
  radioChannel = channel;

// now set the register in the radio

  writeReg(5, channel);		// change the radio frequency

  }
