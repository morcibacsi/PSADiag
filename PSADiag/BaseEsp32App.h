#pragma once

#ifndef _BaseEsp32App_h
    #define _BaseEsp32App_h

#include <inttypes.h>
#include "src/SerialPort/AbstractSerial.h"

#ifdef USE_BLUETOOTH_SERIAL
    #include <BluetoothSerial.h>
    #include "src/SerialPort/BluetoothSerialAbs.h"
#else
    #include "src/SerialPort/HardwareSerialAbs.h"
    #include "src/SerialPort/USBSerialAbs.h"
#endif


class BaseEsp32App
{
    #ifdef USE_BLUETOOTH_SERIAL
        BluetoothSerial SerialBT;
    #endif

    public:
        AbsSer* serialPort;
        AbsSer* InitSerialPort();
        uint16_t GetId();
 };

#endif
