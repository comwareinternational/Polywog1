/*----------------------------------------------------------------------*
 *	polywog.h - Protocol header - Version 1, Rev 3 (1.3)		*
 *----------------------------------------------------------------------*/
 
typedef unsigned char BYTE, *BYTE_PTR;
 
 
typedef struct _pw_hdr
   {
 
/*----------------------------------------------------------------------*
 *	Source and Destination addresses are 8-bit fields, where the 	*
 *	value of 0x00 is RESERVED, and the value of 0xFF is to be used	*
 *	for a broadcast address. Hence, there can be 254 addresses	*
 *	in a network.							*
 *----------------------------------------------------------------------*/

   BYTE srcAddr;	/* Source address */
   BYTE dstAddr;	/* Destination address */

/*----------------------------------------------------------------------*
 *	Hop Count is initially set to 0, and is incremented by each	*
 *	router as a packet is forwarded. If it reaches PW_MAX_HOPS the	*
 *	packet is discarded.						*
 *----------------------------------------------------------------------*/

#define PW_MAX_HOPS 13		/* Maximum allowed hopcount value */

   BYTE pTypeHopCount;	/* Packet type (high 4 bits); Hop Count (low 4 bits) */
#define PW_PTYPE_RAWDATA	0x00		/* Raw Data */
#define PW_PTYPE_RAWRSP		0x01		/* Raw Data Response */
#define PW_PTYPE_OPENSEQREQ	0x02		/* Open Seq Packet Req. */
#define PW_PTYPE_OPENSEQRSP	0x03		/* Open Seq Packet Rsp. */
#define PW_PTYPE_CLOSESEQREQ	0x04		/* Close Seq Packet Req. */
#define PW_PTYPE_CLOSESEQRSP	0x05		/* Close Seq Packet Rsp. */
#define PW_PTYPE_SEQDATA	0x06		/* Sequenced Data Packet */
#define PW_PTYPE_SEQDATARSP	0x07		/* Sequenced Data Response */
#define PW_PTYPE_UNUSED		0x08		/* Unused: 0x08-0x0F */

/*----------------------------------------------------------------------*
 *	The "To Be Continued" flag indicates that this packet will be	*
 *	followed by one or more other packets that make up a UDP	*
 *	(Datagram) transaction. The packet re-assembly operation (which *
 *	takes place at a gateway or at a destination node) will not	*
 *	process the packet until it is completely assembled.		*
 *									*
 *	The Sequence number starts at 0 and is incremented by 1 for a	*
 *	full-size packet received. It is assumed that each packet has	*
 *	the maximum payload size except the last one (which does not	*
 *	have the TBC flag set).						*
 *----------------------------------------------------------------------*/

   BYTE tBCSeq;		/* To Be Continued flag (1 bit); sequence number (7 bits) */


   BYTE srcPortDstPort;	/* Source Port (high 4 bits); Dest. Port (low 4 bits) */
   BYTE functionCode;	/* Unused (2 bits); Packet Function Code (6 bits) */
   }
     PW_HDR;
     
typedef struct _pw_packet	/* Payload size of 32 bytes */
   {
   PW_HDR header;		/* Polywog Header */
   BYTE data[22];		/* 0-22 bytes, variable size */
   }
     PW_PACKET;
     
typedef struct _pw_packet * PW_PACKET_PTR;

