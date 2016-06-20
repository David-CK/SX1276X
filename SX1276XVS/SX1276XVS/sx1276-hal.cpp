/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ( C )2014 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1276-hal.h"
#include <SPI.h>


SX1276MB1xAS::SX1276MB1xAS( void ( *txDone )( ), void ( *txTimeout ) ( ), void ( *rxDone ) ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ), 
                            void ( *rxTimeout ) ( ), void ( *rxError ) ( ), void ( *fhssChangeChannel ) ( uint8_t channelIndex ), void ( *cadDone ) ( bool ChannelActivityDetected ) ) 
                        :   SX1276( txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone )
{
}

void SX1276MB1xAS::IoInit( void )
{
}

void SX1276MB1xAS::RadioRegistersInit( ){
}

void SX1276MB1xAS::SpiInit( void )
{
}


void SX1276MB1xAS::IoDeInit( void )
{
    //nothing
}

uint8_t SX1276MB1xAS::GetPaSelect( uint32_t channel )
{
}

void SX1276MB1xAS::SetAntSwLowPower( bool status )
{
}

void SX1276MB1xAS::AntSwInit( void )
{
}

void SX1276MB1xAS::AntSwDeInit( void )
{
}

void SX1276MB1xAS::SetAntSw( uint8_t rxTx )
{
}

bool SX1276MB1xAS::CheckRfFrequency( uint32_t frequency )
{
    //TODO: Implement check, currently all frequencies are supported
    return true;
}
void SX1276MB1xAS::Reset( void )
{
}
    
void SX1276MB1xAS::Write( uint8_t addr, uint8_t data )
{
}

uint8_t SX1276MB1xAS::Read( uint8_t addr )
{
/*
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
	*/
}

void SX1276MB1xAS::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
}

void SX1276MB1xAS::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
/*
    uint8_t i;

	digitalWrite(nss, LOW);
    SPI.transfer( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer( 0 );
    }
    digitalWrite(nss, HIGH);
*/
}



void SX1276MB1xAS::WriteFifo( uint8_t *buffer, uint8_t size )
{

}

void SX1276MB1xAS::ReadFifo( uint8_t *buffer, uint8_t size )
{

}
