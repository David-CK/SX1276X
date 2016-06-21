#pragma once

#include "network.h"
// Define Modes
#define SX1278_MODE_RX_CONTINUOUS     0x00
#define SX1278_MODE_TX                0x00
#define SX1278_MODE_SLEEP             0x00
#define SX1278_MODE_STANDBY           0x00

#define CR_4_5

#ifdef  CR_4_5
#define CR  0x01
#else
#ifdef  CR_4_6
#define CR  0x02
#else
#ifdef  CR_4_7
#define CR  0x03
#else
#ifdef  CR_4_8
#define CR  0x04
#endif
#endif
#endif
#endif

#define   CRC_EN

#ifdef    CRC_EN
#define   CRC    0x01
#else
#define   CRC    0x00
#endif

#define LR_RegFifo                  0x00
#define LR_RegOpMode                0x01
#define LR_RegFrMsb                 0x06
#define LR_RegFrMid                 0x07
#define LR_RegFrLsb                 0x08
#define LR_RegPaConfig              0x09
#define LR_RegPaRamp                0x0A
#define LR_RegOcp                   0x0B
#define LR_RegLna                   0x0C
#define LR_RegFifoAddrPtr           0x0D
#define LR_RegFifoTxBaseAddr        0x0E
#define LR_RegFifoRxBaseAddr        0x0F
#define LR_RegFifoRxCurrentAddr     0x10
#define LR_RegIrqFlagsMask          0x11
#define LR_RegIrqFlags              0x12
#define LR_RegRxNbBytes             0x13
#define LR_RegRxHeaderCntValueMsb   0x14
#define LR_RegRxHeaderCntValueLsb   0x15
#define LR_RegRxPacketCntValueMsb   0x16
#define LR_RegRxPacketCntValueLsb   0x17
#define LR_RegModemStat             0x18
#define LR_RegPktSnrValue           0x19
#define LR_RegPktRssiValue          0x1A
#define LR_RegRssiValue             0x1B
#define LR_RegHopChannel            0x1C
#define LR_RegModemConfig1          0x1D
#define LR_RegModemConfig2          0x1E
#define LR_RegSymbTimeoutLsb        0x1F
#define LR_RegPreambleMsb           0x20
#define LR_RegPreambleLsb           0x21
#define LR_RegPayloadLength         0x22
#define LR_RegMaxPayloadLength      0x23
#define LR_RegHopPeriod             0x24
#define LR_RegFifoRxByteAddr        0x25

#define REG_LR_DIOMAPPING1          0x40
#define REG_LR_DIOMAPPING2          0x41

#define REG_LR_VERSION              0x42

#define REG_LR_PLLHOP               0x44
#define REG_LR_TCX0                 0x4B
#define REG_LR_PADAC                0x4D
#define REG_LR_FORMERTEMP           0x5B

#define REG_LR_AGCREF               0x61
#define REG_LR_AGCTHRESH1           0x62
#define REG_LR_AGCTHRESH1           0x62
#define REG_LR_AGCTHRESH3           0x64


#define RegFIFO                     0x00
#define RegOpMode                   0x01
#define RegBitRateMsb               0x02
#define RegBitRateLsb               0x03
#define RegFdevMsb                  0x04
#define RegFdevLsb                  0x05
#define RegFreqMsb                  0x06
#define ReqFreqMid                  0x07
#define RegFreqLsb                  0x08
#define RegPaConfig                 0x09
#define RegPaRamp                   0x0A
#define RegOcp                      0x0B
#define RegLna                      0x0C
#define RegRxConfig                 0x0D
#define RegRssiConfig               0x0E
#define RegRssiCollision            0x0F
#define RegRssiThresh               0x10
#define RegRssiValue                0x11
#define RegRxBw                     0x12
#define RegAfcBw                    0x13
#define RegOokPeak                  0x14
#define RegOokFix                   0x15
#define RegOokAvg                   0x16

#define RegAfcFei                   0x1A
#define RegAfcMsb                   0x1B
#define RegAfcLsb                   0x1C
#define RegFeiMsb                   0x1D
#define RegFeiLsb                   0x1E
#define RegPreambleDetect           0x1F
#define RegRxTimeout1               0x20
#define RegRxTimeout2               0x21

#define RegRxTimeout3               0x22

#define RegRxDelay                  0x23
#define RegOsc                      0x24

#define RegPreambleMsb              0x25
#define RegPreambleLsb              0x26

#define RegSyncConfig               0x27
#define RegSyncValue1               0x28
#define RegSyncValue2               0x29
#define RegSyncValue3               0x2A
#define RegSyncValue4               0x2B
#define RegSyncValue5               0x2C

#define RegSyncValue6               0x2D
#define RegSyncValue7               0x2E
#define RegSyncValue8               0x2F
#define RegPacketConfig1            0x30
#define RegPacketConfig2            0x31

#define RegPayloadLength            0x32
#define RegNodeAdrs                 0x33
#define RegBroadcastAdrs            0x34
#define RegFifoThresh               0x35

#define RegSeqConfig1               0x36
#define RegSeqConfig2               0x37

#define RegTimerResol               0x38
#define RegTimer1Coef               0x39
#define RegTimer2Coef               0x3A
#define RegImageCal                 0x3B

#define RegTemp                     0x3C
#define RegLowBat                   0x3D
#define RegIrqFlags1                0x3E

#define RegIrqFlags2                0x3F

#define RegDioMapping1              0x40
#define RegDioMapping2              0x41
#define RegVersion                  0x42
#define RegPllHop                   0x44

#define RegPaDac                    0x4D
#define IRQ_LORA_RXDONE_MASK        0x40
#define LORARegIrqFlags             0x12

void lora_nwk_start();
STATUS lora_tx(uint8_t* data, uint8_t len);

void serial_nwk_start();
