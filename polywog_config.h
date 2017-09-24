#define TX_TIMEOUT 20            // 20 ms Transmit on NRF24L01+ at 250Kbit/sec

// Radio Interface Hardware Interface Definitions

#define CSN 8      	// SPI Slave Select pin aka CSN
#define CE 7		// "Chip Enable" - Used to key the transmitter
#define XMIT_LED 6      // XMIT LED to blink when xmitting
