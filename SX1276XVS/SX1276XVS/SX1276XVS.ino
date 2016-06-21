#include <SPI.h>
#include "sx1276-hal.h"
#include "sx1276Regs-Fsk.h"
#include "debug.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   0
#define BUFFER_SIZE                                     32        // Define the payload size here
typedef RadioState States_t;
volatile States_t State = LOWPOWER;
SX1276MB1xAS Radio( OnTxDone, OnTxTimeout, OnRxDone, OnRxTimeout, OnRxError, NULL, NULL );

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;


static const SPISettings settings(10E6, MSBFIRST, SPI_MODE0);

static void lora_pin_nss(uint8_t val)
{
	if (!val)
	{
		SPI.beginTransaction(settings);
	}
	else
	{
		SPI.endTransaction();
	}

	digitalWrite(10, val);
}

uint8_t lora_spi(uint8_t out)
{
	uint8_t res = SPI.transfer(out);
	return res;
}
void debug1(const char *format, ...)
{
	/*
	va_list args;
	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
	*/
}
// the setup function runs once when you press reset or power the board
void setup()
{
	Serial.begin(115200);
	debug("aaa\n");
	pinMode(10, OUTPUT);
	SPI.begin();

	lora_pin_nss(0);
	lora_spi(0x42 & 0x7F);
	uint8_t val = lora_spi(0x00);
	lora_pin_nss(1);
	debug1("a%d", 1);
	delay(1);
	Serial.println(val);

    // verify the connection with the board
    //while( Radio.Read( REG_VERSION ) == 0x00  )
   // {
		//Serial.println("DEBUG_PRINT");
    //}
	//Serial.println("REG_VERSION = ");
	//Serial.println(Radio.Read(REG_VERSION));
}

// the loop function runs over and over again until power down or reset
void loop()
{
  
}

void OnTxDone( void )
{
    State = TX;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
    debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
}

void OnTxTimeout( void )
{
    State = TX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    
    if(BufferSize < BUFFER_SIZE)
        Buffer[ BufferSize ] = 0;
    
    State = RX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    State = RX_ERROR;
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}

