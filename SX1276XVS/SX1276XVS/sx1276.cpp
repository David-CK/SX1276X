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

const FskBandwidth_t SX1276::FskBandwidths[] =
{       
    { 2600  , 0x17 },   
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};


SX1276::SX1276( void ( *txDone )( ), void ( *txTimeout ) ( ), void ( *rxDone ) ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ), 
                void ( *rxTimeout ) ( ), void ( *rxError ) ( ), void ( *fhssChangeChannel ) ( uint8_t channelIndex ), void ( *cadDone ) ( bool channelActivityDetected ),
                PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5 )
            :   Radio( txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone ),
                //spi( mosi, miso, sclk ),
                nss( nss ),
                reset( reset ),
                dio0( dio0 ), dio1( dio1 ), dio2( dio2 ), dio3( dio3 ), dio4( dio4 ), dio5( dio5 ),
                isRadioActive( false )
{
    wait_ms( 10 );
    this->rxTx = 0;
    this->rxBuffer = new uint8_t[RX_BUFFER_SIZE];
    previousOpMode = RF_OPMODE_STANDBY;
    
    this->dioIrq = new DioIrqHandler[6];

    this->dioIrq[0] = &SX1276::OnDio0Irq;
    this->dioIrq[1] = &SX1276::OnDio1Irq;
    this->dioIrq[2] = &SX1276::OnDio2Irq;
    this->dioIrq[3] = &SX1276::OnDio3Irq;
    this->dioIrq[4] = &SX1276::OnDio4Irq;
    this->dioIrq[5] = NULL;
    
    this->settings.State = IDLE;
}

SX1276::~SX1276( )
{
    delete this->rxBuffer;
    delete this->dioIrq;
}

void SX1276::RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = this->Read( REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )this->Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )this->Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )this->Read( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    this->Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    Write ( REG_IMAGECAL, ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    settings.Channel=  868000000 ;

    // Launch Rx chain calibration for HF band 
    Write ( REG_IMAGECAL, ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    this->Write( REG_PACONFIG, regPaConfigInitVal );
    SetChannel( initialFreq );
}

RadioState SX1276::GetStatus( void )
{
    return this->settings.State;
}

void SX1276::SetChannel( uint32_t freq )
{
    this->settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

bool SX1276::IsChannelFree( ModemType modem, uint32_t freq, int8_t rssiThresh )
{
    int16_t rssi = 0;
    
    SetModem( modem );

    SetChannel( freq );
    
    SetOpMode( RF_OPMODE_RECEIVER );

    wait_ms( 1 );
    
    rssi = GetRssi( modem );
    
/*************NULL*********/
    
    if( rssi > ( int16_t )rssiThresh )
    {
        return false;
    }
    return true;
}

uint32_t SX1276::Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation 
     */
    // Set LoRa modem ON
    SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        wait_ms( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

/*************NULL*********/

    return rnd;
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
uint8_t SX1276::GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX1276::SetRxConfig( ModemType modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SetModem( modem );

/*************NULL*********/
}

void SX1276::SetTxConfig( ModemType modem, int8_t power, uint32_t fdev, 
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn, 
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    SetModem( modem );
    
    paConfig = Read( REG_PACONFIG );
    paDac = Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | GetPaSelect( this->settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
	/*************NULL*********/

}

double SX1276::TimeOnAir( ModemType modem, uint8_t pktLen )
{
    double airTime = 0.0;
/*************NULL*********/

}

void SX1276::Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    this->settings.State = IDLE;
	/*************NULL*********/

}

void SX1276::Sleep( void )
{
/*************NULL*********/
}

void SX1276::Standby( void )
{
/*************NULL*********/
}

void SX1276::Rx( uint32_t timeout )
{
/*************NULL*********/
}

void SX1276::Tx( uint32_t timeout )
{ 
/*************NULL*********/
}

void SX1276::StartCad( void )
{
/*************NULL*********/
}

int16_t SX1276::GetRssi( ModemType modem )
{
    int16_t rssi = 0;
/*************NULL*********/
    return rssi;
}

void SX1276::SetOpMode( uint8_t opMode )
{
    if( opMode != previousOpMode )
    {
        previousOpMode = opMode;
/*************NULL*********/
    }
}

void SX1276::SetModem( ModemType modem )
{/*************NULL*********/

    if( this->settings.Modem != modem )
    {
        this->settings.Modem = modem;
        switch( this->settings.Modem )
        {
        default:
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            break;
        }
    }
}

void SX1276::OnTimeoutIrq( void )
{
/*************NULL*********/
}

void SX1276::OnDio0Irq( void )
{
/*************NULL*********/
}

void SX1276::OnDio1Irq( void )
{
/*************NULL*********/
}

void SX1276::OnDio2Irq( void )
{
/*************NULL*********/
}

void SX1276::OnDio3Irq( void )
{
/*************NULL*********/
}

void SX1276::OnDio4Irq( void )
{
/*************NULL*********/
}

void SX1276::OnDio5Irq( void )
{
/*************NULL*********/
}
