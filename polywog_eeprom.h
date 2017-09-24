/*----------------------------------------------------------------------*
 *	polywog_eeprom definition - for network configuration of nodes	*
 *									*
 *	Use of the EEPROM makes it possible for all nodes to run the	*
 *	same copy of the polywog library, whilst getting their own	*
 *	network number and node number from the EEPROM. The setup	*
 *	program is used to initialize the polywog section of the EEPROM	*
 *									*
 *	See: for code to load EEPROM net_number and node_number info.	*
 *	assumes little-endian machine for storage of net_number.	*
 *----------------------------------------------------------------------*/

#define POLYWOG_EEPROM_BASE 0  // Starting point in EEPROM for POLYWOG network info

typedef struct _polywog_eeprom
  {
  unsigned long net_number;  // 32-bit network number (unused at this point)
  unsigned char node_number;  // 8-bit node number
  char node_name[17];         // 16-characters of name, NUL terminator
  unsigned long crc;          // 32-bit CRC on these values
  } POLYWOG_EEPROM, *POLYWOG_EEPROM_PTR;

/* declarations: */

boolean polywog_eeprom_verify(POLYWOG_EEPROM_PTR);

