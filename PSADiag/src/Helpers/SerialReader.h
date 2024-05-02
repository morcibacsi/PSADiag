#pragma once

#ifndef _SerialReader_h
    #define _SerialReader_h

#include "../SerialPort/AbstractSerial.h"

class SerialReader {
    AbsSer* _serialPort;
public:
    SerialReader(AbsSer *serialPort);

    void Receive(uint16_t *messageLength, uint8_t message[]);
};

#endif
