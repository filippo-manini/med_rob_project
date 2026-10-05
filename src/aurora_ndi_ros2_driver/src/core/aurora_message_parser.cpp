#include "aurora_ndi_ros2_driver/core/aurora_message_parser.hpp"
#include "aurora_ndi_ros2_driver/core/ndi_aurora_ros2.hpp"
#include <iostream>

namespace AuroraDriver
{

AuroraMessageParser::AuroraMessageParser()
    : missing_message_("MISSING")
{
}

bool AuroraMessageParser::interpretMessage0001(const char* message, int& startIndex,
                                              ToolPose& toolPose, double& errorRMS,
                                              int& handleNumber, const bool& statusReply,
                                              bool& missingF)
{
    (void)handleNumber; // Suppress unused parameter warning
    bool readF = true;

    if (strncmp(message + startIndex, missing_message_.c_str(), missing_message_.length()) == 0)
    {
        startIndex += missing_message_.length();
        missingF = true;
    }
    else
    {
        double quaternion0, quaternionX, quaternionY, quaternionZ;
        double positionX, positionY, positionZ;

        // Rotation
        readF = readF && readQuaternion(message, startIndex, quaternion0);
        startIndex += CLquaternion;
        readF = readF && readQuaternion(message, startIndex, quaternionX);
        startIndex += CLquaternion;
        readF = readF && readQuaternion(message, startIndex, quaternionY);
        startIndex += CLquaternion;
        readF = readF && readQuaternion(message, startIndex, quaternionZ);
        startIndex += CLquaternion;

        // Translation
        readF = readF && readPosition(message, startIndex, positionX);
        startIndex += CLposition;
        readF = readF && readPosition(message, startIndex, positionY);
        startIndex += CLposition;
        readF = readF && readPosition(message, startIndex, positionZ);
        startIndex += CLposition;

        // ErrorRMS: RMS of the error on the measurement in mm
        readF = readF && readError(message, startIndex, errorRMS);
        startIndex += CLerrorRMS;

        toolPose.qw = quaternion0;
        toolPose.qx = quaternionX;
        toolPose.qy = quaternionY;
        toolPose.qz = quaternionZ;
        toolPose.x = positionX;
        toolPose.y = positionY;
        toolPose.z = positionZ;
    }

    // Port status
    if (statusReply)
    {
        readF = readF && readPortStatus(message, startIndex, handleNumber);
    }
    startIndex += CLportStatus;

    // Frame number
    int frameNumber;
    readF = readF && readFrameNumber(message, startIndex, frameNumber);
    startIndex += CLframeNo;

    return readF;
}

bool AuroraMessageParser::readQuaternion(const char* message, int& startIndex, double& quaternion)
{
    int tempQuaternion;

    // Read the first value
    char2int(message[startIndex + 1], tempQuaternion);
    quaternion = tempQuaternion;
    // Read second value (first value after decimal)
    char2int(message[startIndex + 2], tempQuaternion);
    quaternion += tempQuaternion / 10.0;
    // Read third value (second value after decimal)
    char2int(message[startIndex + 3], tempQuaternion);
    quaternion += tempQuaternion / 100.0;
    // Read fourth value (third value after decimal)
    char2int(message[startIndex + 4], tempQuaternion);
    quaternion += tempQuaternion / 1000.0;
    // Read fifth value (fourth value after decimal)
    char2int(message[startIndex + 5], tempQuaternion);
    quaternion += tempQuaternion / 10000.0;
    // Read the sign
    if (message[startIndex] == '-')
    {
        quaternion = -quaternion;
    }

    return true;
}

bool AuroraMessageParser::readPosition(const char* message, int& startIndex, double& position)
{
    int tempQuaternion;

    // Read the first value
    char2int(message[startIndex + 1], tempQuaternion);
    position = tempQuaternion * 1000.0;
    // Read second value
    char2int(message[startIndex + 2], tempQuaternion);
    position += tempQuaternion * 100.0;
    // Read third value
    char2int(message[startIndex + 3], tempQuaternion);
    position += tempQuaternion * 10.0;
    // Read fourth value
    char2int(message[startIndex + 4], tempQuaternion);
    position += tempQuaternion;
    // Read fifth value (first value after decimal)
    char2int(message[startIndex + 5], tempQuaternion);
    position += tempQuaternion / 10.0;
    // Read sixth value (2nd value after decimal)
    char2int(message[startIndex + 6], tempQuaternion);
    position += tempQuaternion / 100.0;
    // Read the sign
    if (message[startIndex] == '-')
    {
        position = -position;
    }

    return true;
}

bool AuroraMessageParser::readError(const char* message, int& startIndex, double& errorRMS)
{
    int tempQuaternion;

    // Read the first value
    char2int(message[startIndex + 1], tempQuaternion);
    errorRMS = tempQuaternion;
    // Read second value (first value after decimal)
    char2int(message[startIndex + 2], tempQuaternion);
    errorRMS += tempQuaternion / 10.0;
    // Read third value (second value after decimal)
    char2int(message[startIndex + 3], tempQuaternion);
    errorRMS += tempQuaternion / 100.0;
    // Read fourth value (third value after decimal)
    char2int(message[startIndex + 4], tempQuaternion);
    errorRMS += tempQuaternion / 1000.0;
    // Read fifth value (fourth value after decimal)
    char2int(message[startIndex + 5], tempQuaternion);
    errorRMS += tempQuaternion / 10000.0;
    // Read the sign
    if (message[startIndex] == '-')
    {
        errorRMS = -errorRMS;
    }

    return true;
}

bool AuroraMessageParser::readPortStatus(const char* message, int& startIndex, int& handleNumber)
{
    int systemStatus;

    // Port status NOTE: return message is LITTLE ENDIAN => within a byte bit
    // 0 is at the right side
    char2int(message[startIndex + CLportStatus - 1], systemStatus); // => right char of 8 portstatus chars
    if (systemStatus & 0x1) // bit 0
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: occupied" << std::endl;
    }
    if (systemStatus & 0x2) // bit 1
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: switch 1 closed /GPIO line 1 active" << std::endl;
    }
    if (systemStatus & 0x4) // bit 2
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: switch 2 closed/GPIO line 2 active" << std::endl;
    }
    if (systemStatus & 0x8) // bit 3
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: switch 3 closed/GPIO line 3 active" << std::endl;
    }
    char2int(message[startIndex + CLportStatus - 2], systemStatus); // => second from right char of 8 portstatus chars
    if (systemStatus & 0x1) // bit 4
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: initialized" << std::endl;
    }
    if (systemStatus & 0x2) // bit 5
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: enabled" << std::endl;
    }
    if (systemStatus & 0x4) // bit 6
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: out of volume" << std::endl;
    }
    if (systemStatus & 0x8) // bit 7
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: partially out of volume" << std::endl;
    }
    char2int(message[startIndex + CLportStatus - 3], systemStatus); // => 3th from right char of 8 portstatus chars
    if (systemStatus & 0x1) // bit 8
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: a sensor coil is broken" << std::endl;
    }
    if (systemStatus & 0x2) // bit 9 // Only Polaris (Reserved Aurora)
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: Reserved for Polaris" << std::endl;
    }
    if (systemStatus & 0x4) // bit 10
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: reserved" << std::endl;
    }
    if (systemStatus & 0x8) // bit 11
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: reserved" << std::endl;
    }

    char2int(message[startIndex + CLportStatus - 4], systemStatus); // => 4th from right char of 8 portstatus chars
    if (systemStatus & 0x1) // bit 12
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: processing exception " << std::endl;
    }
    if (systemStatus & 0x4) // bit 14 // Only Polaris
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: Reserved for Polaris" << std::endl;
    }
    if (systemStatus & 0x8) // bit 15
    {
        std::cout << "(AuroraMessageParser) port " << handleNumber << " status: Reserved for Polaris" << std::endl;
    }
    // bits 16-31 are all reserved
    char2int(message[startIndex + CLportStatus - 5], systemStatus); // bits 16-19 (All reserved)
    char2int(message[startIndex + CLportStatus - 6], systemStatus); // bits 20-23 (All reserved)
    char2int(message[startIndex + CLportStatus - 7], systemStatus); // bits 24-27 (All reserved)
    char2int(message[startIndex + CLportStatus - 8], systemStatus); // bits 28-31 (All reserved)

    return true;
}

bool AuroraMessageParser::readFrameNumber(const char* message, int& startIndex, int& frameNumber)
{
    int tempFno;
    bool frameNoF = char2int(message[startIndex], tempFno);
    frameNumber = tempFno;
    for (int i = 1; i < CLframeNo; i++)
    {
        frameNoF = frameNoF && char2int(message[startIndex + i], tempFno);
        frameNumber = frameNumber * 16 + tempFno;
    }
    return true;
}

bool AuroraMessageParser::checkNewLine(const char* message, int& startIndex)
{
    std::string newLineCmd = "\n";
    if (strncmp(&message[startIndex], newLineCmd.c_str(), 1) == 0) // returns 0 if equal
    {
        return true;
    }
    std::cout << "(AuroraMessageParser) no newline character at the expected position" << std::endl;
    return false;
}

bool AuroraMessageParser::searchNewLine(const char* message, int& startIndex, const int& messageLength)
{
    char tempChar = message[startIndex];
    startIndex++;
    int checkReference = messageLength - CLcr - CLcrc - CLstatus + 1; // index where the system status begins
    // Check if the next character is a endline and if we didn't search too
    // far in the message (=an error occurred)
    while (tempChar != '\n' && startIndex < checkReference)
    {
        tempChar = message[startIndex];
        startIndex++;
    }
    if (startIndex < checkReference)
    {
        return true;
    }
    std::cout << "(AuroraMessageParser) ERROR: trying to search for a newline character after system status on the last line" << std::endl;
    return false;
}

bool AuroraMessageParser::int2string(const int& int2convert, std::string& stringFromInt)
{
    temp_int2string_.str("");
    temp_int2string_.clear();
    temp_int2string_ << int2convert;
    stringFromInt = temp_int2string_.str();
    return true;
}

bool AuroraMessageParser::char2int(const char& char2convert, int& intFromChar)
{
    temp_char2int_.str("");
    temp_char2int_.clear();
    temp_char2int_ << std::hex << char2convert;
    temp_char2int_ >> intFromChar;
    return true;
}

bool AuroraMessageParser::char2hex(const char& char2convert, int& intFromChar)
{
    temp_char2int_.str("");
    temp_char2int_.clear();
    temp_char2int_ << char2convert;
    temp_char2int_ >> std::hex >> intFromChar;
    return true;
}

bool AuroraMessageParser::string2int(const std::string& string2convert, unsigned int& intFromChar)
{
    temp_char2int_.str("");
    temp_char2int_.clear();
    temp_char2int_ << std::hex << string2convert;
    temp_char2int_ >> intFromChar;
    return true;
}

bool AuroraMessageParser::string2hex(const std::string& string2convert, unsigned int& intFromChar)
{
    temp_char2int_.str("");
    temp_char2int_.clear();
    temp_char2int_ << string2convert;
    temp_char2int_ >> std::hex >> intFromChar;
    return true;
}

} // namespace AuroraDriver
