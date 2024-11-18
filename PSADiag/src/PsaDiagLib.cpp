#include "PsaDiagLib.h"
#include <string>

int hexCharToValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    } else if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;  // Invalid character
}

bool hexBytesToByteArray(const uint8_t hexBytes[], uint8_t converted[], uint8_t length, uint8_t startIndex) {
    size_t outputIndex = 0;

    for (size_t inputIndex = startIndex; inputIndex < length; inputIndex += 2) {
        int highNibble = hexCharToValue(hexBytes[inputIndex]);
        int lowNibble = hexCharToValue(hexBytes[inputIndex + 1]);

        if (highNibble == -1 || lowNibble == -1) {
            return false;
        }

        converted[outputIndex] = (highNibble << 4) | lowNibble;
        outputIndex++;
    }
    return true;
}

int16_t transform(uint8_t data_msb, uint8_t data_lsb, uint8_t sec[])
{
	int16_t data = (data_msb << 8) | data_lsb;
	int32_t result = ((data % sec[0]) * sec[2]) - ((data / sec[0]) * sec[1]);
	if (result < 0)
		result += (sec[0] * sec[2]) + sec[1];
	return result;
}

uint32_t compute_response(uint16_t pin, uint32_t chg)
{
    byte sec_1[3] = {0xB2, 0x3F, 0xAA};
    byte sec_2[3] = {0xB1, 0x02, 0xAB};

    long res_msb = transform((pin >> 8), (pin & 0xFF), sec_1) | transform(((chg >> 24) & 0xFF), (chg & 0xFF), sec_2);
    long res_lsb = transform(((chg >> 16) & 0xFF), ((chg >> 8) & 0xFF), sec_1) | transform((res_msb >> 8), (res_msb & 0xFF), sec_2);
    return (res_msb << 16) | res_lsb;
}

void PsaDiagLib::ParseCommand(uint8_t data[], uint8_t length)
{
    uint8_t command = data[0];

    switch (command)
    {
        case '>':
            ChangeId(data, length);
            break;
        case 'T':
            ChangeFrameDelay(data);
            break;
        case ':':
            Unlock(data, length);
            break;
        case 'V':
            _serial->println("1.9");
            break;
        case 'L':
            ChangeLIN(data);
            break;
        case 'W':
            ChangeFrameSize(data);
            break;
        case 'U':
            LIN = 0;
            PrintOk();
            break;
        case 'N':
            dump = false;
            PrintOk();
            break;
        case 'X':
            dump = true;
            PrintOk();
            break;
        case 'K':
            ChangeKeepAlive(data);
            break;
        case 'S':
            sendKeepAlives = false;
            PrintOk();
            break;
        case 'R':
            ResetCAN();
            break;
        case '?':
            PrintCurrentCANId();
            break;
        case '#':
            SendRawFrames(data, length);
            break;
        case '+':
            LargeFrameSpliting(data, length);
            break;
        default:
            if (length % 2 == 0)
            {
                SendFrames(data, length);
            }
            else
            {
                /*
                for (size_t i = 0; i < length; i++)
                {
                    _serial->print(data[i], HEX);
                }
                */
                PrintError();
            }
            break;
    }
}

void PsaDiagLib::PrintOk()
{
    _serial->println("OK");
}

void PsaDiagLib::PrintError()
{
    _serial->println("7F0000");
}

void PsaDiagLib::SendKeepAlive()
{
    if (sendKeepAlives)
    {
        if (sendKeepAliveType == 'K')
        {
            //KWP
            if (LIN > 0)
            {
                uint8_t keepAliveFrame[3] = { LIN, 0x01, 0x3E };
                _canSender->SendMessage(CAN_EMIT_ID, 0, 3, keepAliveFrame);
            }
            else
            {
                uint8_t data[] = { 0x3E };
                _canTp->Send(data, 1);
            }
        }
        else
        {
            //UDS
            if (LIN > 0)
            {
                uint8_t keepAliveFrame[4] = { LIN, 0x02, 0x3E, 0x00 };
                _canSender->SendMessage(CAN_EMIT_ID, 0, 4, keepAliveFrame);
            }
            else
            {
                uint8_t data[] = { 0x3E, 0x00 };
                _canTp->Send(data, 2);
            }
        }
    }
}

void PsaDiagLib::ChangeId(uint8_t data[], uint8_t length)
{
    std::string input(reinterpret_cast<char*>(data), length); // Exclude null terminator
    std::string part1, part2;

    // Find the position of the colon
    size_t pos = input.find(':');
    if (pos != std::string::npos) {
        // Extract parts using substr
        part1 = input.substr(0, pos);
        part2 = input.substr(pos + 1); // Start after the colon

        // Remove the first character from part1
        if (!part1.empty()) {
            part1.erase(0, 1); // Remove the first character
        }

        CAN_EMIT_ID = static_cast<uint16_t>(std::stoi(part1, nullptr, 16));
        CAN_RECV_ID = static_cast<uint16_t>(std::stoi(part2, nullptr, 16));

        //printf("CAN_EMIT_ID: %x\n", CAN_EMIT_ID);
        //printf("CAN_RECV_ID: %x\n", CAN_RECV_ID);

        _canTp->SetIds(CAN_EMIT_ID, CAN_RECV_ID);
        LIN = 0;
        dump = false;
        sendKeepAlives = false;
        PrintOk();
    }
}

void PsaDiagLib::ChangeFrameDelay(uint8_t data[])
{
    // example: T100
    bool parseSuccess = sscanf((char*)data, "T%d", &framesDelayInput) == 1;
    if (parseSuccess)
    {
        PrintOk();
    }
}

void PsaDiagLib::Unlock(uint8_t data[], uint8_t length)
{
    // example: :D91C:03:03

    std::string input(reinterpret_cast<char*>(data), length);
    std::string part1, part2, part3;

    input.erase(0, 1); // Remove the first character
    size_t pos = input.find(':');
    if (pos == std::string::npos)
    {
        return;
    }

    part1 = input.substr(0, pos);
    part2 = input.substr(pos + 1,2);
    part3 = input.substr(pos + 4,2);

    UnlockKey = static_cast<uint16_t>(std::stoi(part1, nullptr, 16));
    UnlockService = static_cast<uint8_t>(std::stoi(part2, nullptr, 16));
    DiagSess = static_cast<uint8_t>(std::stoi(part3, nullptr, 16));

    //printf("UnlockKey: %x\n", UnlockKey);
    //printf("UnlockService: %x\n", UnlockService);
    //printf("DiagSess: %x\n", DiagSess);

    UnlockCMD[0] = 0x27;
    UnlockCMD[1] = UnlockService;

    uint8_t diagCmd[2] = { 0x10, DiagSess };
    _canTp->Send(diagCmd, 2);

    if (DiagSess == 0xC0)
    {
        //KWP
        sendKeepAliveType = 'K';
    }
    else
    {
        //UDS
        sendKeepAliveType = 'U';
    }
    sendKeepAlives = true;
    waitForUnlock = true;
}

void PsaDiagLib::ChangeLIN(uint8_t data[])
{
    //example: L47
    bool parseSuccess = sscanf((char*)data, "L%hhx", &LIN) == 1;
    if (parseSuccess)
    {
        PrintOk();
    }
    else
    {
        PrintError();
    }
}

void PsaDiagLib::ChangeFrameSize(uint8_t data[])
{
    customFrameSize = true;
    bool parseSuccess = sscanf((char*)data, "W%d", &receiveDiagFrameSize) == 1;
    if (parseSuccess)
    {
        receiveDiagFrameSize = receiveDiagFrameSize * 2;
        _serial->println("WOK");
    }
}

void PsaDiagLib::ChangeKeepAlive(uint8_t data[])
{
    //example 1: KK
    //example 2: KU
    sendKeepAlives = true;
    if (data[1] == 'U' || data[1] == 'K')
    {
        sendKeepAliveType = data[1];
    }
    PrintOk();
}

void PsaDiagLib::ResetCAN()
{
    /*
    CAN0.reset();
    CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
    while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
        delay(100);
    }
    */
    PrintOk();
}

void PsaDiagLib::PrintCurrentCANId()
{
    //output example: 764:664
    char tmp[4];

    snprintf(tmp, 4, "%02X", CAN_EMIT_ID);
    _serial->print(tmp);
    _serial->print(":");
    snprintf(tmp, 4, "%02X", CAN_RECV_ID);
    _serial->print(tmp);
}

void PsaDiagLib::SendRawFrames(uint8_t data[], uint8_t length)
{
    //example: #00010203040506070809
    //16+1 because of the #
    //the second condition is to check if we have an even number of characters (2 characters = 1 byte)
    if (length <= (16 + 1) && (length - 1) % 2 == 0){
        uint8_t messageLength = (length - 1) / 2;
        uint8_t converted[messageLength];
        bool success = hexBytesToByteArray(data, converted, length, 1);
        if (!success)
        {
            _serial->println("7F0000");
            return;
        }

        _canSender->SendMessage(CAN_EMIT_ID, 0, messageLength, converted);
        waitingReplySerialCMD = true;
        lastCMDSent = millis();
    }
}

void PsaDiagLib::LargeFrameSpliting(uint8_t data[], uint8_t length)
{
    /*
    sendingAdditionalDiagFramesPos = 1;
    sendingAdditionalDiagFrames = true;

    waitingReplySerialCMD = true;
    lastCMDSent = millis();
    */
}

void PsaDiagLib::SendFrames(uint8_t data[], uint8_t length)
{
    uint8_t messageLength = (length) / 2;
    uint8_t converted[messageLength];
    bool success = hexBytesToByteArray(data, converted, length, 0);
    if (!success)
    {
        _serial->println("7F0000");
        return;
    }

    if (customFrameSize)
    {
        _canSender->SendMessage(CAN_EMIT_ID, 0, receiveDiagFrameSize, converted);
        receiveDiagFrameSize = 0;
        customFrameSize = false;
    }
    else
    {
        //
        /*
        for (size_t i = 0; i < messageLength; i++)
        {
            _serial->print(converted[i], HEX);
        }
        _serial->println();
        //*/
        _canTp->Send(converted, messageLength);
    }

    waitingReplySerialCMD = true;
    lastCMDSent = millis();
}

bool PsaDiagLib::Loop(unsigned long currentTime)
{
    if (currentTime - lastKeepAliveSent >= 1000)
    {
        lastKeepAliveSent = currentTime;
        SendKeepAlive();
    }

    CAN_TP::ProcessResult result = _canTp->Process(currentTime, &receivedCanTpPacketLength, receivedCanTpPacket);
    if (result == CAN_TP::RxSuccess)
    {
        //serialPort->println("Received CAN-TP message:");
        PrintArrayToSerial(receivedCanTpPacketLength, receivedCanTpPacket);

        ProcessUnwrappedMessage(currentTime, CAN_RECV_ID, receivedCanTpPacketLength, receivedCanTpPacket);

        return true;
    }
    return false;
}

void PsaDiagLib::ProcessIncomingMessage(unsigned long currentTime, uint16_t canId, uint8_t canMessageLength, uint8_t data[])
{
    uint16_t len = canMessageLength;
    if (canId >= 0)
    {
        _canTp->ProcessIncomingMessage(currentTime, canId, len, data);

        if (dump)
        {
            char tmp[4];
            snprintf(tmp, 4, "%02X", CAN_EMIT_ID);
            _serial->print(tmp);
            _serial->print(":");

            PrintArrayToSerial(canMessageLength, data);
        }

        if (canId == CAN_RECV_ID && data[0] >= 0x40 && data[0] <= 0x70)
        {
            // UDS or KWP with LIN ECUs, remove encapsulation
            for (int i = 1; i < canMessageLength; i++) {
                data[i - 1] = data[i];
            }

            ProcessUnwrappedMessage(currentTime, canId, canMessageLength-1, data);
        }
        else
        {
            ProcessUnwrappedMessage(currentTime, canId, canMessageLength, data);
        }
    }
}

void PsaDiagLib::ProcessUnwrappedMessage(unsigned long currentTime, uint16_t canId, uint8_t length, uint8_t data[])
{
    if (canId == CAN_EMIT_ID && length == 2 && data[0] == 0x01 && data[1] == 0x3E)
    {
        // Diagbox or external tool sending keep-alives, stop sending ours
        sendKeepAlives = false;
    }

    if (canId == CAN_RECV_ID && length == 1 && data[0] == 0x7E)
    {
        lastKeepAliveReceived = currentTime;
    }

    if (canId == CAN_RECV_ID && waitForUnlock)
    {
        if (length == 2 && data[0] == 0x50 && data[1] == UnlockService)
        {
            // open diag session success, send query seed command
            _canTp->Send(UnlockCMD, 2);
        }
        if (length == 6 && data[0] == 0x67 && data[1] == UnlockService)
        {
            // received seed, compute response and send unlock command
            waitForUnlock = false;

            uint32_converter challenge;
            challenge.data.byte1 = data[2];
            challenge.data.byte2 = data[3];
            challenge.data.byte3 = data[4];
            challenge.data.byte4 = data[5];

            uint32_converter response;
            response.asUInt32_t = compute_response(UnlockKey, challenge.asUInt32_t);

            uint8_t data[] = { 0x27, (UnlockService + 1), response.data.byte1, response.data.byte2, response.data.byte3, response.data.byte4};
            _canTp->Send(data, 6);
        }
    }
}

void PsaDiagLib::PrintArrayToSerial(uint16_t sizeOfByteArray, uint8_t *byteArray, uint8_t startIndex)
{
    char tmp[3];
    for (uint16_t i = startIndex; i < sizeOfByteArray; i++)
    {
        snprintf(tmp, 3, "%02X", byteArray[i]);
        _serial->print(tmp);
    }
    _serial->println();
}
