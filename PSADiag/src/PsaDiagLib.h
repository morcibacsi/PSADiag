#ifndef PsaDiagLib_h
    #define PsaDiagLib_h

#include <inttypes.h>
#include <stdio.h>
#include "../src/Can/ICanMessageSender.h"
#include "../src/SerialPort/AbstractSerial.h"
#include "CAN_TP.hpp"

union uint32_converter {
    struct {
        uint8_t byte4;
        uint8_t byte3;
        uint8_t byte2;
        uint8_t byte1;
    } data;
    uint32_t asUInt32_t;
};


class PsaDiagLib
{
    private:
    ICanMessageSender* _canSender;
    AbsSer* _serial;
    CAN_TP* _canTp;

    uint16_t CAN_EMIT_ID;
    uint16_t CAN_RECV_ID;

    uint8_t LIN;
    bool dump = false;
    bool sendKeepAlives = false;

    uint8_t framesDelayInput;

    uint16_t UnlockKey;
    uint8_t UnlockService;
    uint8_t DiagSess;

    char sendKeepAliveType;
    bool waitForUnlock;
    uint8_t UnlockCMD[2];

    bool waitingReplySerialCMD = false;

    unsigned long lastCMDSent = 0;
    unsigned long lastKeepAliveSent = 0;
    unsigned long lastKeepAliveReceived = 0;

    uint8_t receivedCanTpPacket[4096];
    uint16_t receivedCanTpPacketLength;

    int receiveDiagFrameSize;
    bool customFrameSize = false;

    void PrintOk();
    void PrintError();
    void SendKeepAlive();
    void ChangeId(uint8_t data[], uint8_t length);
    void ChangeFrameDelay(uint8_t data[]);
    void Unlock(uint8_t data[], uint8_t length);
    void ChangeLIN(uint8_t data[]);
    void ChangeFrameSize(uint8_t data[]);
    void ChangeKeepAlive(uint8_t data[]);
    void ResetCAN();
    void PrintCurrentCANId();
    void SendRawFrames(uint8_t data[], uint8_t length);
    void SendFrames(uint8_t data[], uint8_t length);
    void LargeFrameSpliting(uint8_t data[], uint8_t length);
    void PrintArrayToSerial(uint16_t sizeOfByteArray, uint8_t *byteArray, uint8_t startIndex = 0);
    void ProcessUnwrappedMessage(unsigned long currentTime, uint16_t canId, uint8_t length, uint8_t data[]);

    public:
    PsaDiagLib(ICanMessageSender* canSender, AbsSer* serial)
    {
        _canSender = canSender;
        _serial = serial;
        _canTp = new CAN_TP(canSender, 0x772, 0x672);
    };
    void ParseCommand(uint8_t data[], uint8_t length);
    bool Loop(unsigned long currentTime);
    void ProcessIncomingMessage(unsigned long currentTime, uint16_t canId, uint8_t canMessageLength, uint8_t data[]);
    virtual ~PsaDiagLib(){ }
};
#endif
