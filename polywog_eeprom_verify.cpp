// Arduino Network EEPROM Setup utility - polywog_eeprom_net_verify
// D. Retz, August, 2017

#include <Arduino.h>
#include <EEPROM.h>

//typedef unsigned int UINT;

#include "polywog_eeprom.h"    // contains EEPROM structure


// eeprom_crc returns a 32-bit unsigned value

unsigned long eeprom_crc(unsigned char *p, int length) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < length ; ++index) {
    crc = crc_table[(crc ^ p[index]) & 0x0F] ^ (crc >> 4);
    crc = crc_table[(crc ^ (p[index] >> 4)) & 0x0F] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

/*----------------------------------------------------------------------*
 *	This code loads the values into EEPROM structure and returns	*
 *	TRUE if EEPROM valid.						*
 *									* 
 *	Assumes pointer to POLYWOG_EEPROM structure, which gets filled	*
 *	in IF the EEPROM is valid.					*
 *	Returns TRUE if EEPROM is valid, FALSE otherwise.		*
 *----------------------------------------------------------------------*/

boolean polywog_eeprom_verify(POLYWOG_EEPROM *eep)
  {
  unsigned long tcrc; // temp crc for comparison
  int i; 
  unsigned char *xcp;

  for (i=POLYWOG_EEPROM_BASE, xcp=(unsigned char *) eep; i<sizeof(POLYWOG_EEPROM); i++)
     *xcp++ = EEPROM[i];  // copy byte-for-byte into structure    
  tcrc = eeprom_crc((unsigned char *) eep, sizeof(POLYWOG_EEPROM)-sizeof(eep->crc) );  // compute CRC on stored data
  return (tcrc == eep->crc);
  }
