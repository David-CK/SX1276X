#include <SPI.h>
#include "sx1276-hal.h"
#include "sx1276Regs-Fsk.h"

SX1276MB1xAS Radio( OnTxDone, OnTxTimeout, OnRxDone, OnRxTimeout, OnRxError, NULL, NULL );

// the setup function runs once when you press reset or power the board
void setup()
{
	bool b;
    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
    }
}

// the loop function runs over and over again until power down or reset
void loop()
{
  
}
void OnTxDone( void )
{
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
}
void OnTxTimeout( void )
{
}
void OnRxTimeout( void )
{
}
void OnRxError( void )
{
}
