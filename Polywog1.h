/*----------------------------------------------------------------------*
 *	Polywog1.h							*
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
 *----------------------------------------------------------------------*/

#ifndef _Polywog_h_
#define _Polywog_h_

#include <SPI.h>
#include <EEPROM.h>

#include "polywog_config.h"
#include "polywog.h"
#include "polywog_eeprom.h"

class Polywog1 {
public:
  boolean begin(void);
  boolean xmit(PW_PACKET_PTR pp, unsigned int bc, byte nextHop);
  int recv(PW_PACKET_PTR, unsigned int);
  int avail(void);
  unsigned long my_network(void);
  byte my_node(void);
  void flushTX();
  void flushRX();
  void setChannel(int);		// set transmission channel number
  int radioChannel = 4;	// channel number (chan, Mhz + 2400)
  
 private:
  void writeReg(int reg, int value);
  byte readReg(int reg);
  void readMReg(int reg, byte *buf, int c);
  void writeMReg(int reg, byte *buf, int c);
  void readMRXPayload(byte *buf, int c);
  byte readPLWidth();
  void writeMTXPayloadNoAck(byte *buf, int c);

  boolean isListening;
  
/* Polywog configuration settings */

  POLYWOG_EEPROM polywog_eeprom;

/* My specific receive address */

byte a1[5] = {0x51, 0x01, 0x00, 0x00, 0x00}; // receive address, network number
byte a2[5] = {0xFF, 0x01, 0x00, 0x00, 0x00}; // broadcast address, network number

/* destination ("TO") address if I were to transmit */

byte a3[5] = {0x52, 0x01, 0x00, 0x00, 0x00}; // default destination address, network number

PW_PACKET test_packet = {{ 0x52, 0xA0, 0x00, 0x00, 0x00, 0x00 }}; // ping packet
PW_PACKET packetBuf;

byte receive_buffer[32];   // for received packets

};










#endif
