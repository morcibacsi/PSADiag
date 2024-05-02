#include "SerialReader.h"

SerialReader::SerialReader(AbsSer* serialPort)
{
    _serialPort = serialPort;
}

void SerialReader::Receive(uint16_t* messageLength, uint8_t message[])
{
    uint16_t receivedMessageLength = 0;
    while (_serialPort->available() > 0)
    {
        uint8_t receivedByte = _serialPort->read();
        if (receivedByte != '\n')
        {
            message[receivedMessageLength] = receivedByte;
            receivedMessageLength++;
        }
        else
        {
            break;
        }
    }
    *messageLength = receivedMessageLength;
}