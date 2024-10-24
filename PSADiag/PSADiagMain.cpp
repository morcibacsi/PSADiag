#include "PSADiagMain.h"
#include "src/SerialPort/AbstractSerial.h"
#include "BaseEsp32App.h"
#include "src/Can/CanMessageSenderEsp32Idf.h"
#include "src/Can/ICanMessageSender.h"
#include "src/Can/ICanMessageSender.h"
#include "src/Helpers/SerialReader.h"
#include "src/PsaDiagLib.h"

const uint8_t CAN_RX_PIN = 18;
const uint8_t CAN_TX_PIN = 15;

BaseEsp32App* baseEsp32App;
TaskHandle_t CANSendDataTask;
TaskHandle_t CANReadTask;

AbsSer* serialPort;
ICanMessageSender* canInterface;
PsaDiagLib* psaDiag;
SerialReader* serialReader;

unsigned long lastBsiEmulated = 0;
bool emulateBsi = true;

void SendIgnitionMessages(unsigned long currentTime)
{
    if (currentTime - lastBsiEmulated >= 100)
    {
        lastBsiEmulated = currentTime;

        uint8_t data_18[] = {0x80, 0x00, 0x02, 0x00, 0x00};
        canInterface->SendMessage(0x18, 0, 5, data_18); // Original delay : 1000ms

        uint8_t data_36[] = {0x0E, 0x00, 0x03, 0x2A, 0x31, 0x00, 0x81, 0xAC};
        canInterface->SendMessage(0x36, 0, 8, data_36); // Original delay : 100ms

        uint8_t data_F6[] = {0x8E, 0x61, 0x00, 0x01, 0xA4, 0x7B, 0x7B, 0x20};
        canInterface->SendMessage(0xF6, 0, 8, data_F6); // Original delay : 50ms
    }
}

void CANSendDataTaskFunction(void * parameter)
{
    unsigned long currentTime = 0;
    uint16_t messageLength = 0;
    uint8_t message[512];

    for (;;)
    {
        currentTime = millis();

        if (emulateBsi)
        {
            SendIgnitionMessages(currentTime);
        }

        messageLength = 0;
        serialReader->Receive(&messageLength, message);
        if (messageLength > 0)
        {
            psaDiag->ParseCommand(message, messageLength);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void CANReadTaskFunction(void * parameter)
{
    unsigned long currentTime = 0;
    uint16_t canId;
    uint8_t canData[8];
    uint8_t canDataLength;

    for (;;)
    {
        currentTime = millis();

        //serialPort->println("Read CAN message");
        canId = 0;
        canInterface->ReadMessage(&canId, &canDataLength, canData);
        if (canId > 0)
        {
            //serialPort->println("Received CAN message");
            psaDiag->ProcessIncomingMessage(currentTime, canId, canDataLength, canData);
        }

        psaDiag->Loop(currentTime);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    baseEsp32App = new BaseEsp32App();
    serialPort = baseEsp32App->InitSerialPort();

    canInterface = new CanMessageSenderEsp32Idf(CAN_RX_PIN, CAN_TX_PIN, false, serialPort);
    canInterface->Init();

    psaDiag = new PsaDiagLib(canInterface, serialPort);
    serialReader = new SerialReader(serialPort);

    xTaskCreatePinnedToCore(
        CANSendDataTaskFunction,        // Function to implement the task
        "CANSendDataTask",              // Name of the task
        15000,                          // Stack size in words
        NULL,                           // Task input parameter
        1,                              // Priority of the task (higher the number, higher the priority)
        &CANSendDataTask,               // Task handle.
        1);                             // Core where the task should run

    xTaskCreatePinnedToCore(
        CANReadTaskFunction,            // Function to implement the task
        "CANReadDataTask",              // Name of the task
        15000,                          // Stack size in words
        NULL,                           // Task input parameter
        2,                              // Priority of the task (higher the number, higher the priority)
        &CANReadTask,                   // Task handle.
        0);                             // Core where the task should run
}

void loop()
{
    vTaskDelay(50 / portTICK_PERIOD_MS);
}