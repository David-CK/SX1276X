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

const RadioRegisters_t SX1276MB1xAS::RadioRegsInit[] = 
{                                                 
    { MODEM_FSK , REG_LNA                , 0x23 },
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },
    { MODEM_FSK , REG_OSC                , 0x07 },
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },  
};

/*
SX1276MB1xAS::SX1276MB1xAS( void ( *txDone )( ), void ( *txTimeout ) ( ), void ( *rxDone ) ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ), 
                            void ( *rxTimeout ) ( ), void ( *rxError ) ( ), void ( *fhssChangeChannel ) ( uint8_t channelIndex ), void ( *cadDone ) ( bool ChannelActivityDetected ),
                            PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                            PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5,
                            PinName antSwitch )
                            : SX1276( txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone, mosi, miso, sclk, nss, reset, dio0, dio1, dio2, dio3, dio4, dio5),
                            antSwitch( antSwitch ),
                        #if( defined ( TARGET_NUCLEO_L152RE ) )
                            fake( D8 ) 
                        #else
                            fake( A3 )
                        #endif
{
    Reset( );
    
    RxChainCalibration( );
    
    IoInit( );
    
    SetOpMode( RF_OPMODE_SLEEP );
    
    IoIrqInit( dioIrq );
    
    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = IDLE ;
}
*/

SX1276MB1xAS::SX1276MB1xAS( void ( *txDone )( ), void ( *txTimeout ) ( ), void ( *rxDone ) ( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ), 
                            void ( *rxTimeout ) ( ), void ( *rxError ) ( ), void ( *fhssChangeChannel ) ( uint8_t channelIndex ), void ( *cadDone ) ( bool ChannelActivityDetected ) ) 
                        /*
                        #if defined ( TARGET_NUCLEO_L152RE )
                        :   SX1276( txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone, D11, D12, D13, D10, A0, D2, D3, D4, D5, A3, D9 ), // For NUCLEO L152RE dio4 is on port A3
                            antSwitch( A4 ),
                            fake( D8 )
                        #else
                        :   SX1276( txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone, D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9 ),
                            antSwitch( A4 ), 
                            fake( A3 )
                        #endif
						*/
						:   SX1276(txDone, txTimeout, rxDone, rxTimeout, rxError, fhssChangeChannel, cadDone, 11, 12, 13, 10, A5, A4, A4, A4, A4, A4, A4)
{
    Reset( );
    
    boardConnected = UNKNOWN;
    
    DetectBoardType( );
    
    RxChainCalibration( );
    
    IoInit( );
    
    SetOpMode( RF_OPMODE_SLEEP );
    IoIrqInit( dioIrq );
    
    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = IDLE ;
}

//-------------------------------------------------------------------------
//                      Board relative functions
//-------------------------------------------------------------------------
uint8_t SX1276MB1xAS::DetectBoardType( void )
{
	/*
    if( boardConnected == UNKNOWN )
    {
        antSwitch.input( );
        wait_ms( 1 );
        if( antSwitch == 1 )
        {
            boardConnected = SX1276MB1LAS;
        }
        else
        {
            boardConnected = SX1276MB1MAS;
        }
        antSwitch.output( );
        wait_ms( 1 );
    }
	*/
    return ( boardConnected );
}

void SX1276MB1xAS::IoInit( void )
{
    AntSwInit( );
    SpiInit( );
}

void SX1276MB1xAS::RadioRegistersInit( ){
    uint8_t i = 0;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SetModem( RadioRegsInit[i].Modem );
        Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }    
}

void SX1276MB1xAS::SpiInit( void )
{
/*******NULL***********/
}

void SX1276MB1xAS::IoIrqInit( DioIrqHandler *irqHandlers )
{
/*******NULL***********/
}

void SX1276MB1xAS::IoDeInit( void )
{
    //nothing
}

uint8_t SX1276MB1xAS::GetPaSelect( uint32_t channel )
{
    if( channel > RF_MID_BAND_THRESH )
    {
        if( boardConnected == SX1276MB1LAS )
        {
            return RF_PACONFIG_PASELECT_PABOOST;
        }
        else
        {
            return RF_PACONFIG_PASELECT_RFO;
        }
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276MB1xAS::SetAntSwLowPower( bool status )
{
/*******NULL***********/
}

void SX1276MB1xAS::AntSwInit( void )
{
    //antSwitch = 0;
}

void SX1276MB1xAS::AntSwDeInit( void )
{
    //antSwitch = 0;
}

void SX1276MB1xAS::SetAntSw( uint8_t rxTx )
{
	/*
    if( this->rxTx == rxTx )
    {
        //no need to go further
        return;
    }

    this->rxTx = rxTx;

    if( rxTx != 0 )
    {
        antSwitch = 1;
    }
    else
    {
        antSwitch = 0;
    }
	*/
}

bool SX1276MB1xAS::CheckRfFrequency( uint32_t frequency )
{
    //TODO: Implement check, currently all frequencies are supported
    return true;
}


void SX1276MB1xAS::Reset( void )
{
	pinMode(reset, OUTPUT);
	digitalWrite(reset, LOW);
    wait_ms( 1 );
	pinMode(reset, INPUT);
    wait_ms( 6 );
}
    
void SX1276MB1xAS::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}

uint8_t SX1276MB1xAS::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1276MB1xAS::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
/*******NULL***********/
}

void SX1276MB1xAS::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
/*******NULL***********/
	Serial.println("DEBUG_PRINT: Read3");
    uint8_t i;
	//SPI.beginTransaction();
	digitalWrite(10, LOW);
	//digitalWrite(nss, LOW);
    SPI.transfer( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer( 0 );
    }
    //digitalWrite(nss, HIGH);
	//SPI.endTransaction();
	digitalWrite(10, HIGH);

}

void SX1276MB1xAS::WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

void SX1276MB1xAS::ReadFifo( uint8_t *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}
