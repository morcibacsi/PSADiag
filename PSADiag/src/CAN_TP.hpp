#ifndef CAN_TP_H
#define CAN_TP_H

#include "../src/Can/ICanMessageSender.h"

const uint8_t SINGLE_FRAME = 0x0;
const uint8_t FIRST_FRAME  = 0x1;
const uint8_t CONSECUTIVE_FRAME = 0x2;
const uint8_t FLOW_CONTROL_FRAME = 0x3;

union UInt16
{
    struct
    {
        uint8_t rightByte;
        uint8_t leftByte;
    } data;
    uint16_t asUint16;
};

union UInt24
{
    struct
    {
        uint8_t rightByte;
        uint8_t middleByte;
        uint8_t leftByte;
    } data;
    uint32_t asUint24;
};

union PCIDataSingleFrame {
    struct {
        uint16_t length_or_sequence : 4;
        uint8_t frameType           : 4;
    } data;
    uint8_t byte;
};

union PCIDataFirstFrame {
    struct {
        uint16_t length   : 12;
        uint8_t frameType : 4;
    } data;
    UInt16 bytes;
};

union PCIDataFlowControlFrame {
    struct {
        uint8_t separationTime : 8;
        uint8_t blockSize      : 8;
        uint8_t flowStatus     : 4;
        uint8_t frameType      : 4;
    } data;
    UInt24 bytes;
};

class CAN_TP {
    private:
    enum State {
        Idle,
        StartTransmission,
        WaitForFlowControl,
        SendConsecutiveFrames,
        WaitForFirstFrame,
        WaitForConsecutiveFrames,
        EndTransmission,
        EndReception
    };

    ICanMessageSender *_canInterface;
    uint16_t _txId, _rxId;

    State _txState;
    State _rxState;
    uint8_t _consecutiveFrameCounter;
    unsigned long _lastMillis;

    uint16_t _txMessageLength;
    uint8_t txBuffer[4096];

    uint16_t rxMessageLength;
    uint8_t rxBuffer[4096];

    void sendSingleFrame(uint8_t* data, uint8_t size);
    void sendFirstFrame();
    void sendConsecutiveFrame();
    void sendFlowControl();

    public:
    enum ProcessResult {
        InProgress,
        TxSuccess,
        TxFlowControlTimeout,
        RxSuccess,
        RxFirstFrameTimeout,
        RxConsecutiveFrameTimeout,
        RxSequenceNumberMismatch,
        RxCanIdMismatch,
        RxLengthMismatch,
        RxInvalidFrameType
    };

    CAN_TP(ICanMessageSender *canInterface, uint16_t txId, uint16_t rxId);
    void SetIds(uint16_t txId, uint16_t rxId);
    uint8_t Send(uint8_t* byteArray, uint8_t sizeOfByteArray);

    // Incoming CAN message should be passed to this function, it will build the CAN-TP message based on the rxId
    CAN_TP::ProcessResult ProcessIncomingMessage(unsigned long millis, uint16_t canId, uint8_t length, uint8_t incomingBuffer[]);

    // When Process() returns RxSuccess, the received message is stored in receivedMessage and its length in receivedMessageLength
    CAN_TP::ProcessResult Process(unsigned long millis, uint16_t* receivedMessageLength, uint8_t receivedMessage[]);
};

#endif
