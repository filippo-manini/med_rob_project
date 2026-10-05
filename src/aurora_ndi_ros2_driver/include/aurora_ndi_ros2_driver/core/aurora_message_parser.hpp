#pragma once

#include <string>
#include <sstream>
#include <cstring>

namespace AuroraDriver
{

struct ToolPose;

/**
 * @brief Parses and interprets messages from Aurora device
 *
 * This class handles all message interpretation including:
 * - Quaternion reading
 * - Position reading
 * - Error RMS reading
 * - Port status interpretation
 * - Frame number extraction
 * - String/hex conversions
 */
class AuroraMessageParser
{
public:
    AuroraMessageParser();

    /**
     * @brief Parse transformation data from Aurora message (reply option 0001)
     * @param message Raw message buffer from Aurora
     * @param startIndex Current parsing position (will be updated)
     * @param toolPose Output structure for pose data
     * @param errorRMS Output for error RMS value
     * @param handleNumber Port handle number
     * @param statusReply Whether to print status information
     * @param missingF Output flag indicating if tool is missing
     * @return true if parsing successful
     */
    bool interpretMessage0001(const char* message, int& startIndex, ToolPose& toolPose,
                            double& errorRMS, int& handleNumber, const bool& statusReply,
                            bool& missingF);

    /**
     * @brief Read quaternion component from message
     * @param message Message buffer
     * @param startIndex Starting position
     * @param quaternion Output quaternion value
     * @return true if successful
     */
    bool readQuaternion(const char* message, int& startIndex, double& quaternion);

    /**
     * @brief Read position component from message
     * @param message Message buffer
     * @param startIndex Starting position
     * @param position Output position value in mm
     * @return true if successful
     */
    bool readPosition(const char* message, int& startIndex, double& position);

    /**
     * @brief Read error RMS from message
     * @param message Message buffer
     * @param startIndex Starting position
     * @param errorRMS Output error value
     * @return true if successful
     */
    bool readError(const char* message, int& startIndex, double& errorRMS);

    /**
     * @brief Read and interpret port status
     * @param message Message buffer
     * @param startIndex Starting position
     * @param handleNumber Handle number for logging
     * @return true if successful
     */
    bool readPortStatus(const char* message, int& startIndex, int& handleNumber);

    /**
     * @brief Read frame number from message
     * @param message Message buffer
     * @param startIndex Starting position
     * @param frameNumber Output frame number
     * @return true if successful
     */
    bool readFrameNumber(const char* message, int& startIndex, int& frameNumber);

    /**
     * @brief Check for newline character at expected position
     * @param message Message buffer
     * @param startIndex Position to check
     * @return true if newline found
     */
    bool checkNewLine(const char* message, int& startIndex);

    /**
     * @brief Search for newline character in message
     * @param message Message buffer
     * @param startIndex Starting position (will be updated)
     * @param messageLength Total message length
     * @return true if newline found
     */
    bool searchNewLine(const char* message, int& startIndex, const int& messageLength);

    // Conversion utilities
    bool int2string(const int& int2convert, std::string& stringFromInt);
    bool char2int(const char& char2convert, int& intFromChar);
    bool char2hex(const char& char2convert, int& intFromChar);
    bool string2int(const std::string& string2convert, unsigned int& intFromChar);
    bool string2hex(const std::string& string2convert, unsigned int& intFromChar);

private:
    // Message field lengths (constants from NDI protocol)
    static constexpr int CLcr = 1;
    static constexpr int CLcrc = 4;
    static constexpr int CLnoPH = 2;
    static constexpr int CLPHno = 2;
    static constexpr int CLPHstat = 3;
    static constexpr int CLstatus = 4;
    static constexpr int CLquaternion = 6;
    static constexpr int CLposition = 7;
    static constexpr int CLerrorRMS = 6;
    static constexpr int CLlf = 1;
    static constexpr int CLportStatus = 8;
    static constexpr int CLframeNo = 8;
    static constexpr int CLnoSM = 2;

    // Temporary conversion variables
    std::ostringstream temp_int2string_;
    std::stringstream temp_char2int_;

    const std::string missing_message_;
};

} // namespace AuroraDriver
