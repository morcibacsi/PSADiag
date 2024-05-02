#include "PsaDiagLib.h"

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
            ChangeId(data);
            break;
        case 'T':
            ChangeFrameDelay(data);
            break;
        case ':':
            Unlock(data);
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
            Dump = false;
            PrintOk();
            break;
        case 'X':
            Dump = true;
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
                Send(data, 1);
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
                Send(data, 2);
            }
        }
    }
}

void PsaDiagLib::ChangeId(uint8_t data[])
{
    // example: >764:664
    bool parseSuccess = sscanf((char*)data, ">%x:%x", &CAN_EMIT_ID, &CAN_RECV_ID) == 2;
    if (parseSuccess)
    {
        ModifyIds(CAN_EMIT_ID, CAN_RECV_ID);
        LIN = 0;
        Dump = false;
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

void PsaDiagLib::Unlock(uint8_t data[])
{
    // example: :D91C:03:03
    bool parseSuccess = sscanf((char*)data, ":%x:%x:%x", &UnlockKey, &UnlockService, &DiagSess) == 2;
    if (parseSuccess)
    {
        UnlockCMD[0] = 0x27;
        UnlockCMD[1] = UnlockService;

        uint8_t diagCmd[2] = { 0x10, DiagSess };
        Send(diagCmd, 2);

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
        waitingUnlock = true;
    }
}

void PsaDiagLib::ChangeLIN(uint8_t data[])
{
    //example: L47
    bool parseSuccess = sscanf((char*)data, "L%x", &LIN) == 1;
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
        Send(converted, messageLength);
    }

    waitingReplySerialCMD = true;
    lastCMDSent = millis();
}

void PsaDiagLib::InternalProcess()
{
    if (currentTime - lastKeepAliveSent >= 1000)
    {
        lastKeepAliveSent = currentTime;
        SendKeepAlive();
    }
}

void PsaDiagLib::ReceiveFinished()
{
    char tmp[3];
    for (size_t i = 0; i < _rxMsg.len; i++)
    {
        snprintf(tmp, 3, "%02X", _rxMsg.Buffer[i]);
        _serial->print(tmp);
    }
    _serial->println();
}

void PsaDiagLib::ProcessMessage(unsigned long currentTime, uint16_t canId, uint8_t canMessageLength, uint8_t data[])
{
    uint16_t len = canMessageLength;
    if (canId >= 0)
    {
        bool encap = false;
        if (data[0] >= 0x40 && data[0] <= 0x70) { // UDS or KWP with LIN ECUs, remove encapsulation
          for (int i = 1; i < canMessageLength; i++) {
            data[i - 1] = data[i];
          }
          len--;
          encap = true;
        }

        if (data[0] < 0x10 && data[1] == 0x7E)
        {
            lastKeepAliveReceived = currentTime;
        }
        else if (data[0] < 0x10 && data[1] == 0x3E)
        {
            sendKeepAlives = false; // Diagbox or external tool sending keep-alives, stop sending ours
        }
        else
        {
            char tmp[4];
            if (Dump)
            {
                snprintf(tmp, 4, "%02X", canId);
                _serial->print(tmp);
                _serial->print(":");
            }
            for (int i = 1; i < len; i++)
            { // Strip first byte = Data length
                snprintf(tmp, 3, "%02X", data[i]);
                _serial->print(tmp);
            }
            _serial->println();
        }

        if (waitingUnlock && data[0] < 0x10 && data[1] == 0x67 && data[2] == UnlockService)
        {
            uint32_converter challenge;
            challenge.data.byte1 = data[3];
            challenge.data.byte2 = data[4];
            challenge.data.byte3 = data[5];
            challenge.data.byte4 = data[6];

            uint32_converter response;
            response.asUInt32_t = compute_response(UnlockKey, challenge.asUInt32_t);

            uint8_t data[] = { 0x27, (UnlockService + 1), response.data.byte1, response.data.byte2, response.data.byte3, response.data.byte4};
            Send(data, 6);

            char tmp[4];

            if (Dump)
            {
                snprintf(tmp, 4, "%02X", CAN_EMIT_ID);
                _serial->print(tmp);
                _serial->print(":");
                for (int i = 0; i < 6; i++)
                {
                    snprintf(tmp, 3, "%02X", data[i]);
                    _serial->print(tmp);
                }
                _serial->println();
            }

            waitingUnlock = false;
        }
    }
}
