#ifndef __NETWORK_H__
#define __NETWORK_H__
#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <WString.h>

#include <lmic.h>
#include <hal/hal.h>

#if PLATFORM_ID == 0xAE
#include <CurieEEPROM.h>
#else
#include <EEPROM.h>
#endif

#define JOIN_REQUEST 0x01
#define JOIN_ACCEPT  0x02
#define PULL_DATA    0x03
#define DATA_PACKET  0x04
#define DATA_ACK     0x05
#define DATA_PULL    0x06

#define TIMEOUT                     -100
#define ACK_TIMEOUT_MS              300
#define JOIN_TIMEOUT_MS             300
#define PULL_TIMEOUT_MS             1000

#define NO_TIMEOUT                  0
#define SUCCESS                     0

#define AP_ADDR                     1
#define BROADCASR_ADDR              0xff
#define NO_ADDR                     0xff

#define OPT_ACK_REQ                 1
#define OPT_NO_ACK                  2


#define CONNECTION_LOSED            -101
#define INVALID_PACKET              -102
#define INVALID_TRACE_ID            -103
#define CRC_ERROR                   -104
#define INVALID_DST_ADDR            -105
#define NO_DATA                     -106

#define DONT_CHECK_TRACE_ID         0

#define MAX_PAYLOAD_SIZE            40

#define ADDR_SIZE                   6

typedef int8_t STATUS;

#ifndef bool
#define bool uint8_t
#endif

#ifndef true
#define true  1
#endif

#ifndef false
#define false 0
#endif

#define MAX_END_DEVICES 256

#define START_BYTE 0x7F

#define START_BYTE_SIZE 1

#define CRC_SIZE 1

enum { max_length = 64 };

#define PRINT_PACKAGE

#pragma pack(push)
#pragma pack(1)

typedef struct {
    uint8_t addr[ADDR_SIZE];
} Addr;

typedef struct {
    uint8_t startByte;
    Addr dstAddr;
    Addr srcAddr;
    uint8_t packetType;
    uint8_t traceId;
    uint8_t packetInfo;
    uint8_t payloadLen;
    uint8_t payload[MAX_PAYLOAD_SIZE];
} Packet;

struct TxHandler {
    //additional attr by xulinzhe
    osjob_t SendJob;
    uint8_t tryTimes;
    bool free;

    //package struct
    Packet txbuffer;

    //init struct
    TxHandler()
    {
        free = true;
        tryTimes = 1;
    }
};

typedef struct {

} JoinRequestPayload;

typedef struct {
    uint8_t status;
} JoinAcceptPayload;

typedef struct {
    uint8_t traceId;
} DataAckPayload;

typedef struct {

} DataPullPayload;

typedef enum __attribute__((__packed__))
{
    RegisterDeviceCommmand = 1,
    ReadAttrCommmand,
    ReadAttrResponseCommmand,
    WriteAttrCommmand,
    PublishAttrCommmand,
    UnRegisterDeviceCommmand,
}
WirelessCommand;

typedef struct {
    WirelessCommand cmd;
    uint8_t payload[30];
} WirelessRawData;

typedef struct {
    WirelessCommand cmd;
    uint64_t sensorUUID;
    uint8_t payload[20];
} WirelessRegisterSensorData;

typedef struct {
    WirelessCommand cmd;
    uint64_t sensorUUID;
    uint16_t attrID;
    uint8_t len;
    uint8_t payload[20];
} WirelessWriteData;

typedef struct {
    WirelessCommand cmd;
    uint64_t sensorUUID;
    uint16_t attrID;
} WirelessReadData;

typedef struct {
    WirelessCommand cmd;
    uint64_t sensorUUID;
    uint16_t attrID;
    uint8_t dataType;
    uint8_t len;
    uint8_t payload[20];
} WirelessReadResponseData;

typedef struct {
    WirelessCommand cmd;
    uint64_t sensorUUID;
    uint16_t attrID;
    uint8_t dataType;
    uint8_t len;
    uint8_t payload[20];
} WirelessPublishData;

typedef enum __attribute__((__packed__))
{
    int8 = 1,
    uint8,
    int16,
    uint16,
    int32,
    uint32,
    int64,
    uint64,
    string,
    bytearray,
    floattype,
    doubletype
}
SensorDataType;
#pragma pack(pop)

typedef STATUS (*hal_nwk_tx_t)(uint8_t* data, uint8_t len);
void hal_set_nwk_tx(hal_nwk_tx_t tx);
void hal_nwk_rx(Packet* packet);

//void nwk_init();
void nwk_start();

void hal_sleep(uint32_t ms);
void hal_log_raw(const __FlashStringHelper* fmt, ...);

#define F1(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

#define hal_log(fmt, ...) hal_log_raw(F1(fmt), ## __VA_ARGS__ )

uint8_t nwk_cal_crc(Packet* p);

void nwk_set_myaddr(Addr addr);
void print_addr(Addr addr, char* addrType);
void print_packet(Packet* p);
bool nwk_is_valid_packet(Packet* p);

bool isBroadCastAddr(Addr addr);
bool isToMe(Addr addr);

void registerSensor(uint64_t sensorUUID, uint8_t len, uint8_t* payload);
void unRegisterSensor(uint64_t sensorUUID);
void publish(uint64_t sensorUUID, uint16_t attrID, uint8_t* data,
             uint8_t len, uint8_t dataType);

void GlobalWriteCbk(uint64_t sensorUUID, uint16_t attrID, uint8_t len,
                    uint8_t* data);
void GlobalReadCbk(uint64_t sensorUUID, uint16_t attrID, uint8_t* len,
                   uint8_t* data, uint8_t* dataType);

#endif
