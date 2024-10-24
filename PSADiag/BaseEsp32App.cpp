#include "BaseEsp32App.h"

uint16_t BaseEsp32App::GetId()
{
    uint64_t macAddress = ESP.getEfuseMac();
    return (uint16_t)(macAddress >> 32);
}

AbsSer* BaseEsp32App::InitSerialPort()
{
    uint16_t uniqueIdForBluetooth = 0;
    uniqueIdForBluetooth = GetId();

    char bluetoothDeviceName[10];
    snprintf(bluetoothDeviceName, 10, "ESP32 %04X", uniqueIdForBluetooth);

#ifdef USE_BLUETOOTH_SERIAL
    serialPort = new BluetoothSerAbs(SerialBT, bluetoothDeviceName);
#else
    #if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
        serialPort = new UsbSerAbs(Serial);
    #else
        serialPort = new HwSerAbs(Serial);
    #endif
#endif

    serialPort->begin(115200);
    //serialPort->println(bluetoothDeviceName);

    return serialPort;
}