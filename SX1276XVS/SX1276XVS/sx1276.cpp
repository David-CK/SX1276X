/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ( C )2014 Semtech

Description: Actual implementation of a SX1276 radio, inherits Radio

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1276.h"

SX1276::SX1276( void ( *txDone )( ), void ( *txTimeout ) ( ), void ( *rxDone ) ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ), 
                void ( *rxTimeout ) ( ), void ( *rxError ) ( ), void ( *fhssChangeChannel ) ( uint8_t channelIndex ), void ( *cadDone ) ( bool channelActivityDetected ))
            :   Radio( txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone )
{
}
SX1276::~SX1276( )
{
}
RadioState SX1276::GetStatus( void )
{
}
void SX1276::SetChannel( uint32_t freq )
{
}
bool SX1276::IsChannelFree( ModemType modem, uint32_t freq, int8_t rssiThresh )
{
}
uint32_t SX1276::Random( void )
{
}
/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
uint8_t SX1276::GetFskBandwidthRegValue( uint32_t bandwidth )
{
}

void SX1276::SetRxConfig( ModemType modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
}
void SX1276::SetTxConfig( ModemType modem, int8_t power, uint32_t fdev, 
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn, 
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
}
double SX1276::TimeOnAir( ModemType modem, uint8_t pktLen )
{
}

void SX1276::Send( uint8_t *buffer, uint8_t size )
{
}

void SX1276::Sleep( void )
{
}

void SX1276::Standby( void )
{
}

void SX1276::Rx( uint32_t timeout )
{
}

void SX1276::Tx( uint32_t timeout )
{ 
}

void SX1276::StartCad( void )
{
}

int16_t SX1276::GetRssi( ModemType modem )
{
}

void SX1276::SetOpMode( uint8_t opMode )
{
}

void SX1276::SetModem( ModemType modem )
{
}

void SX1276::OnTimeoutIrq( void )
{
}

void SX1276::OnDio0Irq( void )
{
}

void SX1276::OnDio1Irq( void )
{
}

void SX1276::OnDio2Irq( void )
{
}

void SX1276::OnDio3Irq( void )
{
}

void SX1276::OnDio4Irq( void )
{
}

void SX1276::OnDio5Irq( void )
{
}
