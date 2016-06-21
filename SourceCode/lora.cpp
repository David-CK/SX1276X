#include <SPI.h>
#include "network.h"
#include "lora.h"

#define EI_NOTPORTB
#define EI_NOTPORTD
#include "EnableInterrupt-master\EnableInterrupt.h"

#define NSEL    10
#define RESET   A5
#define DIO0    A4

extern void nwk_packet_handler(Packet* p);

unsigned char mode;
unsigned char Freq_Sel;
unsigned char Power_Sel;
unsigned char Lora_Rate_Sel;
unsigned char BandWide_Sel;
unsigned char Fsk_Rate_Sel;

static const SPISettings settings(10E6, MSBFIRST, SPI_MODE0);
static void lora_config(void);

static void lora_spi_init()
{
    SPI.begin();
}

static void lora_pin_nss(uint8_t val)
{
    if (!val)
    { SPI.beginTransaction(settings); }
    else
    { SPI.endTransaction(); }

    digitalWrite(NSEL, val);
}

uint8_t lora_spi(uint8_t out)
{
    uint8_t res = SPI.transfer(out);
#ifdef SPI_DEBUG
    Serial.print(">");
    Serial.print(out, HEX);
    Serial.print("<");
    Serial.println(res, HEX);
#endif
    return res;
}

static void spi_write(uint8_t addr, uint8_t data)
{
    lora_pin_nss(0);
    lora_spi(addr | 0x80);
    lora_spi(data);
    lora_pin_nss(1);
}

static uint8_t spi_read(uint8_t addr)
{
    lora_pin_nss(0);
    lora_spi(addr & 0x7F);
    uint8_t val = lora_spi(0x00);
    lora_pin_nss(1);
    return val;
}

static void spi_burst_write(uint8_t addr, unsigned char* buf, uint8_t len)
{
    lora_pin_nss(0);
    lora_spi(addr | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        lora_spi(buf[i]);
    }
    lora_pin_nss(1);
}

static void spi_burst_read(uint8_t addr, unsigned char* buf, uint8_t len)
{
    lora_pin_nss(0);
    lora_spi(addr & 0x7F);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = lora_spi(0x00);
    }
    lora_pin_nss(1);
}

unsigned char sx1278FreqTbl[1][3] = {
    //433Mhz
    { 0x6C, 0x40, 0x00 },
};


unsigned char sx1278PowerTbl[4] = {
    0xFF,
    0xFC,
    0xF9,
    0xF6,
};


unsigned char sx1278SpreadFactorTbl[7] = {
    6,7,8,9,10,11,12
};


unsigned char sx1278LoRaBwTbl[10] = {
    // 7.8Khz, 10.4KHz, 15.6KHz, 20.8KHz, 31.2KHz,
    // 41.7KHz, 62.5KHz, 125KHz, 250KHz, 500KHz
    0,1,2,3,4,5,6,7,8,9
};

unsigned char RxData[64];

void lora_standby(void)
{
    // Standby & Low Frequency mode
    spi_write(LR_RegOpMode, 0x09);
    // standby high frfequency mode
    // SPIWrite(LR_RegOpMode, 0x01);
}

void lora_sleep(void)
{
    // Sleep & Low Frequency mode
    spi_write(LR_RegOpMode, 0x08);
    // Sleep / high frequency mode
    // SPIWrite(LR_RegOpMode, 0x00);
}

void lora_mode(void)
{
    // Low frequency mode
    spi_write(LR_RegOpMode, 0x88);
    // Sigh frequency mode
    // SPIWrite(LR_RegOpMode, 0x80);
}

void lora_clear_irq(void)
{
    spi_write(LR_RegIrqFlags, 0xFF);
}


unsigned char lora_entry_rx(void)
{
    unsigned char addr;
    // setting base parater
    lora_config();

    spi_write(REG_LR_PADAC, 0x84);
    spi_write(LR_RegHopPeriod, 0xFF);
    spi_write(REG_LR_DIOMAPPING1, 0x01);

    spi_write(LR_RegIrqFlagsMask, 0x3f);

    lora_clear_irq();

    spi_write(LR_RegPayloadLength, 21);

    addr = spi_read(LR_RegFifoRxBaseAddr);

    spi_write(LR_RegFifoAddrPtr, addr);
    // Set the Operating Mode to Continuos Rx Mode && Low Frequency Mode
    spi_write(LR_RegOpMode, 0x8d);

    while (1) {
        if ((spi_read(LR_RegModemStat) & 0x04) == 0x04) {
            break;
        }
        return 0;
    }
}


unsigned char lora_read_rssi(void)
{
    unsigned int temp = 10;
    temp = spi_read(LR_RegRssiValue);
    //temp = temp + 127 - 137;
    return (unsigned char)temp;
}

void print_hex(unsigned char* buffer, uint8_t size)
{
    /*for (uint8_t i = 0; i < size; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");*/
}

void lora_receive()
{
    unsigned char i;
    unsigned char addr;
    unsigned char packet_size;

    if (digitalRead(DIO0)) {
        uint8_t flags = spi_read(LORARegIrqFlags);
        if (flags & IRQ_LORA_RXDONE_MASK) {
            lora_clear_irq();
            //Serial.println(F("new lora packet"));

            addr = spi_read(LR_RegFifoRxCurrentAddr);
            spi_write(LR_RegFifoAddrPtr, addr);

            if (sx1278SpreadFactorTbl[Lora_Rate_Sel] == 6) {
                packet_size = 21;
            }
            else {
                packet_size = spi_read(LR_RegRxNbBytes);
            }

            spi_burst_read(0x00, RxData, packet_size);
            if (packet_size > 0) {
                //print_hex(RxData, packet_size);
                //nwk_packet_handler((Packet*)RxData);
				Serial.print("RSSI : ");
				Serial.println(lora_read_rssi());
				Serial.write((char*)RxData, packet_size);
            }
            else {
                //Serial.println(F("packet size is 0"));
            }
            lora_clear_irq();
        }
    }
    return;
}

unsigned char lora_entry_tx(void)
{
    unsigned char addr, temp;

    // setting base parater
    lora_config();

    spi_write(REG_LR_PADAC, 0x87);
    spi_write(LR_RegHopPeriod, 0x00);
    spi_write(REG_LR_DIOMAPPING1, 0x41);

    lora_clear_irq();
    spi_write(LR_RegIrqFlagsMask, 0xF7);
    
    addr = spi_read(LR_RegFifoTxBaseAddr);

    spi_write(LR_RegFifoAddrPtr, addr);

}

void lora_set_payload_len(uint8_t len) {
	unsigned char addr, temp;
	spi_write(LR_RegPayloadLength, len);
	while (1) {
		temp = spi_read(LR_RegPayloadLength);
		if (temp == len) { break; }
	}
}

STATUS lora_tx(uint8_t* data, uint8_t len)
{
    unsigned char TxFlag = 0;
    unsigned char addr;

    lora_entry_tx();
	lora_set_payload_len(len);
    spi_burst_write(0x00, (unsigned char*)data, len);
    noInterrupts();
    spi_write(LR_RegOpMode, 0x8b);
    while (1) {
        if (digitalRead(DIO0)) {
            spi_read(LR_RegIrqFlags);
            lora_clear_irq();
            lora_standby();
            // going to rx
            lora_entry_rx();
            break;
        }
    }
    interrupts();
    //Serial.println(F("Tx Done"));
    return SUCCESS;
}

void print_version()
{
    /*uint8_t version = spi_read(RegVersion);
    Serial.print(F("version is "));
    Serial.print(version);
    Serial.println("");*/
}

void lora_hal_pin_rst(uint8_t val)
{
    if (val == 0 || val == 1) {
        pinMode(RESET, OUTPUT);
        digitalWrite(RESET, val);
    }
    else {
        // keep pin floating
        pinMode(RESET, INPUT);
    }
}

void lora_config(void)
{
    unsigned char i;

    /* configure paramters */
    //lora mode
    mode = 0x01;
    //433Mhz
    Freq_Sel = 0x00;
    Power_Sel = 0x00;
    Lora_Rate_Sel = 0x06;
    BandWide_Sel = 0x07;
    Fsk_Rate_Sel = 0x00;

    // modem must be in sleep mode
    lora_sleep();

    for (i = 250; i != 0; i--);
    delay(10);

    //lora mode
    lora_mode();

    spi_burst_write(LR_RegFrMsb, sx1278FreqTbl[Freq_Sel], 3);	//433M

    spi_write(LR_RegPaConfig, sx1278PowerTbl[Power_Sel]);		//0xff		20dbm

    spi_write(LR_RegOcp, 0x0B);
    spi_write(LR_RegLna, 0x23);				//g1

	spi_write(RegPaDac, 0x87);			

    if (sx1278SpreadFactorTbl[Lora_Rate_Sel] == 6) {
        unsigned char tmp;

        spi_write(LR_RegModemConfig1, \
                  ((sx1278LoRaBwTbl[BandWide_Sel] << 4) + (CR << 1) + 0x01));			//125K
        spi_write(LR_RegModemConfig2, \
                  ((sx1278SpreadFactorTbl[Lora_Rate_Sel] << 4) + (CRC << 2) + 0x03));
        tmp = spi_read(0x31);
        tmp &= 0xF8;
        tmp |= 0x05;
        spi_write(0x31, tmp);
        spi_write(0x37, 0x0C);
    }

    else {
        spi_write(LR_RegModemConfig1, \
                  ((sx1278LoRaBwTbl[BandWide_Sel] << 4) + (CR << 1) + 0x00));			//125K
        spi_write(LR_RegModemConfig2, \
                  ((sx1278SpreadFactorTbl[Lora_Rate_Sel] << 4) + (CRC << 2) + 0x03));	//sf9
    }

    spi_write(LR_RegSymbTimeoutLsb, 0xFF);
    spi_write(LR_RegPreambleMsb, 0x00);
    spi_write(LR_RegPreambleLsb, 12);
    spi_write(REG_LR_DIOMAPPING2, 0x01);

    //print_version();
    lora_standby();
}

void lora_reset()
{
    lora_hal_pin_rst(0);
    delay(1);
    lora_hal_pin_rst(2);
    delay(10);
}

void lora_listening()
{
    pinMode(DIO0, INPUT);
    lora_config();
    lora_entry_rx();
	//attachInterrupt(digitalPinToInterrupt(DIO0), lora_receive, RISING);
	enableInterrupt(DIO0, lora_receive, RISING);
}

void lora_nwk_start()
{
    pinMode(NSEL, OUTPUT);
    lora_spi_init();
    lora_reset();
    lora_listening();
//    hal_set_nwk_tx(lora_tx);
}