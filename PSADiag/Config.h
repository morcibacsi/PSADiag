#pragma once

#ifndef _Config_h
    #define _Config_h

#include <stdint.h>
#include <WString.h>

//#include "Config_Bluetooth.h"

struct Config
{
    #define PRINT_DEBUG

    #ifdef WIFI_ENABLED
        //#define WEBSOCKET_SERIAL
    #endif

    #ifndef WEBSOCKET_SERIAL
        // if defined messages are sent on bluetooth connection, otherwise standard serial is used
        #ifndef WIFI_ENABLED
            //#define USE_BLUETOOTH_SERIAL
        #endif
    #endif
    Config();
};

#endif
