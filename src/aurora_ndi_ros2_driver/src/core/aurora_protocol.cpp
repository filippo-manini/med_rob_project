#include "aurora_ndi_ros2_driver/core/aurora_protocol.hpp"
#include "aurora_ndi_ros2_driver/core/aurora_serial_interface.hpp"
#include "aurora_ndi_ros2_driver/core/aurora_message_parser.hpp"
#include "aurora_ndi_ros2_driver/core/ndi_aurora_ros2.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <algorithm>

namespace AuroraDriver
{

AuroraProtocol::AuroraProtocol(AuroraSerialInterface* serial, AuroraMessageParser* parser)
    : serial_(serial)
    , parser_(parser)
    , error_message_("ERROR")
    , missing_message_("MISSING")
    , warning_message_("WARNING")
    , no_ports_enabled_(0)
    , rom_file_(nullptr)
{
    memset(return_message_, 0, sizeof(return_message_));
    memset(return_message_copy_, 0, sizeof(return_message_copy_));
    memset(buffer_, 0, sizeof(buffer_));
}

AuroraProtocol::~AuroraProtocol()
{
    if (rom_file_) {
        fclose(rom_file_);
        rom_file_ = nullptr;
    }
}

bool AuroraProtocol::initAurora()
{
    std::string message = "INIT \r";
    if (!serial_->write(message.c_str(), message.length())) {
        return false;
    }

    usleep(10000);

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: initialization Aurora failed " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) initialization of Aurora: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::stopAurora()
{
    no_ports_enabled_ = 0;
    active_port_handles_.clear();
    all_tool_pose_.clear();
    visible_tools_.clear();
    return true;
}

bool AuroraProtocol::sendBeep()
{
    std::string message = "BEEP 1\r";
    if (!serial_->write(message.c_str(), message.length())) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: Aurora BEEP failed: " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) returned message for BEEP: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::portHandleStatus(const int& replyOption, char(&returnMessage)[255])
{
    std::string replyOptionStr;
    if (!parser_->int2string(replyOption, replyOptionStr)) {
        return false;
    }

    std::string message = "PHSR 0" + replyOptionStr + "\r";

    if (!serial_->write(message.c_str(), message.length())) {
        return false;
    }

    if (!serial_->read(returnMessage)) {
        return false;
    }

    if (strncmp(returnMessage, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: port handle status request failed " << returnMessage << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) port handle status request: message returned: " << returnMessage << std::endl;

    return true;
}

bool AuroraProtocol::portHandleFree(char(&portHandle)[2])
{
    sprintf(buffer_, "PHF %c%c\r", portHandle[0], portHandle[1]);

    if (!serial_->write(buffer_, strlen(buffer_))) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: free port handle failed " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) free port handle request: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::loadToolDefFile(char(&portHandle)[2], char* toolDefinition)
{
    // Open tool definition file (.rom)
    if (!(rom_file_ = fopen(toolDefinition, "rb"))) // rb=read binary
    {
        std::cout << "(AuroraProtocol) ERROR: failed to open tool definition file (.rom)" << std::endl;
        if (rom_file_) { fclose(rom_file_); }
        return false;
    }

    // Read the tool definition file
    unsigned char toolDefData[1024];
    int nBytes = fread(toolDefData, 1, sizeof(toolDefData), rom_file_);
    if (nBytes < 1)
    {
        std::cout << "(AuroraProtocol) ERROR: failed to read tool definition file (.rom)" << std::endl;
        if (rom_file_) { fclose(rom_file_); }
        return false;
    }

    // Construct the command (port per port)
    char tooldefCommand[256];
    for (int j = 0; j < nBytes;)
    {
        sprintf(tooldefCommand, "PVWR %c%c%04X", portHandle[0], portHandle[1], j);
        for (int i = 0; i < 64; i++, j++) // tooldefinition data = 128 hex characters = 64*2
        {
            sprintf(tooldefCommand + 11 + 2 * i, "%02X", toolDefData[j]); // 11 = start message, 2 ascii characters/char
        }

        sprintf(tooldefCommand + 139, "\r"); // don't forget the carriage return!

        if (!serial_->write(tooldefCommand, strlen(tooldefCommand))) {
            fclose(rom_file_);
            rom_file_ = nullptr;
            return false;
        }

        if (!serial_->read(return_message_)) {
            fclose(rom_file_);
            rom_file_ = nullptr;
            return false;
        }

        // Check returnMessage
        if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
        {
            std::cout << "(AuroraProtocol) ERROR: failed to load tool definition file" << return_message_ << std::endl;
            fclose(rom_file_);
            rom_file_ = nullptr;
            return false;
        }
        std::cout << "(AuroraProtocol) load tool definition file: message returned: " << return_message_ << std::endl;
    }

    // Close the tool definition file
    if (rom_file_) {
        fclose(rom_file_);
        rom_file_ = nullptr;
    }

    return true;
}

bool AuroraProtocol::portHandleInit(char(&portHandle)[2])
{
    sprintf(buffer_, "PINIT %c%c\r", portHandle[0], portHandle[1]);

    if (!serial_->write(buffer_, strlen(buffer_))) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    // Check returnMessage
    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: init port handle failed " << return_message_ << std::endl;
        return false;
    }
    else if (strncmp(return_message_, warning_message_.c_str(), warning_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) WARNING: init port handle warning: " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) init port handle request: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::defaultToolConfig(char(&portHandle)[2])
{
    sprintf(buffer_, "TTCFG %c%c\r", portHandle[0], portHandle[1]);

    if (!serial_->write(buffer_, strlen(buffer_))) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    // Check returnMessage
    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: init port handle failed " << return_message_ << std::endl;
        return false;
    }
    else if (strncmp(return_message_, warning_message_.c_str(), warning_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) WARNING: init port handle warning: " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) ttcfg port handle request: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::portHandleEnable(char(&portHandle)[2], char priority)
{
    sprintf(buffer_, "PENA %c%c%c\r", portHandle[0], portHandle[1], priority);

    if (!serial_->write(buffer_, strlen(buffer_))) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    // Check returnMessage
    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: enable port handle failed " << return_message_ << std::endl;
        return false;
    }
    else if (strncmp(return_message_, warning_message_.c_str(), warning_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) WARNING: enable port handle warning: " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) enable port handle request: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                                     const std::vector<std::string>& toolDefPortHandles,
                                     const std::vector<std::string>& portHandlesAutoconfig,
                                     const std::string& refPortHandle)
{
    // Validate port handles
    for (size_t idx = 0; idx < toolDefPortHandles.size(); idx++)
        if (2 != toolDefPortHandles.at(idx).length()) {
            std::cout << "(AuroraProtocol) Error: Length of the port handles must be 2" << std::endl;
            return false;
        }
    if (2 != refPortHandle.length()) {
        std::cout << "(AuroraProtocol) Error: Length of the port handles must be 2" << std::endl;
        return false;
    }

    char sign_refPortHandle[2];
    sign_refPortHandle[0] = refPortHandle.c_str()[0];
    sign_refPortHandle[1] = refPortHandle.c_str()[1];

    // Calculate how many ports need ROM files vs autoconfigured
    int num_autoconfig = portHandlesAutoconfig.size();
    int expected_rom_files = toolDefPortHandles.size() - num_autoconfig;

    if (toolDefinitions.size() != static_cast<size_t>(expected_rom_files)) {
        std::cout << "(AuroraProtocol) ERROR: Number of tool definition files (" << toolDefinitions.size()
                  << ") doesn't match expected (" << expected_rom_files
                  << " = total ports " << toolDefPortHandles.size()
                  << " - autoconfig " << num_autoconfig << ")" << std::endl;
        return false;
    }

    std::cout << "(AuroraProtocol) Configuration: " << toolDefPortHandles.size() << " total ports, "
              << toolDefinitions.size() << " with ROM files, "
              << num_autoconfig << " autoconfigured" << std::endl;

    usleep(500000);

    if (!portHandleStatus(0, return_message_copy_))
    {
        std::cout << "(AuroraProtocol) port handle status request for PH to free failed." << std::endl;
        return false;
    }

    // Get port handles that need to be freed
    if (!portHandleStatus(1, return_message_copy_))
    {
        std::cout << "(AuroraProtocol) port handle status request for PH to free failed." << std::endl;
        return false;
    }

    // Free port handles if needed
    int PHint = 0;
    int PHint2 = 0;
    parser_->char2int(return_message_copy_[0], PHint);
    parser_->char2int(return_message_copy_[1], PHint2);
    for (int i = 0; i < (PHint * 16 + PHint2); i++)
    {
        char portHandle[2];
        portHandle[0] = return_message_copy_[2 + i * (CLPHno + CLPHstat)];
        portHandle[1] = return_message_copy_[3 + i * (CLPHno + CLPHstat)];
        if (!portHandleFree(portHandle))
        {
            std::cout << "(AuroraProtocol) free port handle " << i << " failed." << std::endl;
            return false;
        }
    }

    // Get port handles that need to be initialized
    if (!portHandleStatus(2, return_message_copy_))
    {
        std::cout << "(AuroraProtocol) port handle status request for PH to initialize failed." << std::endl;
        return false;
    }

    PHint = 0;
    PHint2 = 0;
    parser_->char2int(return_message_copy_[0], PHint);
    parser_->char2int(return_message_copy_[1], PHint2);
    int noTools = 16 * PHint + PHint2;

    std::cout << "(AuroraProtocol) Number of ports located: " << noTools << std::endl << "Status of the ports:" << std::endl;

    // Helper function to check if port is autoconfigured
    auto isAutoconfig = [&portHandlesAutoconfig](const std::string& port) -> bool {
        for (const auto& ac_port : portHandlesAutoconfig) {
            if (port == ac_port) return true;
        }
        return false;
    };

    // Helper function to check if port is in configured list
    auto isConfigured = [&toolDefPortHandles](const std::string& port) -> bool {
        for (const auto& handle : toolDefPortHandles) {
            if (port == handle) return true;
        }
        return false;
    };

    // FIRST LOOP: Load ROM files for configured ports (excluding autoconfig)
    int rom_file_index = 0;
    for (int k = 0; k < noTools; k++)
    {
        std::string sPH = std::string(return_message_copy_ + (2 + k * 5), 2);
        std::string sPHStatus = std::string(return_message_copy_ + (4 + k * 5), 3);
        std::cout << "Port: " << k << " with handle: " << sPH <<
                                "; has status of: " << sPHStatus << std::endl;

        // Check if this port is configured
        if (!isConfigured(sPH)) {
            std::cout << "\tSkipping unconfigured port: " << sPH << std::endl;
            continue;
        }

        // Check if this port needs autoconfig
        if (isAutoconfig(sPH)) {
            std::cout << "\tPort " << sPH << " will be AUTOCONFIGURED (using internal ROM)" << std::endl;
            continue;
        }

        // Regular port with external ROM file
        char portHandle[2];
        portHandle[0] = sPH.c_str()[0];
        portHandle[1] = sPH.c_str()[1];

        std::cout << "\tThis port will load Tool Definition File: " << toolDefinitions.at(rom_file_index) << std::endl;

        // Load the tool definition file
        char* romFileName = const_cast<char*>(toolDefinitions[rom_file_index].c_str());
        if (!loadToolDefFile(portHandle, romFileName))
        {
            std::cout << "(AuroraProtocol) loading tool definition file failed." << std::endl;
            return false;
        }
        rom_file_index++;
    }

    bool bDuplicate = false;
    int noPorts2Init = 0;
    do {
        // Check for port Handles that have to be initialized
        if (!portHandleStatus(2, return_message_copy_))
        {
            std::cout << "(AuroraProtocol) port handle status request for PH to initialize failed." << std::endl;
            return false;
        }

        PHint = 0;
        PHint2 = 0;
        parser_->char2int(return_message_copy_[0], PHint);
        parser_->char2int(return_message_copy_[1], PHint2);
        noPorts2Init = 16 * PHint + PHint2;

        for (int k = 0; k < noPorts2Init; k++) {
            std::string sPH = std::string(return_message_copy_ + (2 + k * 5), 2);

            // Initialize ALL configured ports (both ROM and autoconfig)
            if (isConfigured(sPH)) {
                char portHandle[2];
                portHandle[0] = sPH.c_str()[0];
                portHandle[1] = sPH.c_str()[1];
                if (!portHandleInit(portHandle))
                {
                    std::cout << "(AuroraProtocol) port handle initialization of PH " << sPH << " failed." << std::endl;
                    return false;
                }

                if (isAutoconfig(sPH)) {
                    std::cout << "(AuroraProtocol) Successfully initialized AUTOCONFIGURED port: " << sPH << std::endl;
                } else {
                    std::cout << "(AuroraProtocol) Successfully initialized configured port: " << sPH << std::endl;
                }
            } else {
                std::cout << "(AuroraProtocol) Skipping initialization of unconfigured port: " << sPH << std::endl;
            }
        }
    } while ((noPorts2Init != 0) && bDuplicate);

    int noPorts2Enable = 0;
    do {
        // Check all the ports to be enabled
        if (!portHandleStatus(3, return_message_copy_))
        {
            std::cout << "(AuroraProtocol) port handle status request for PH to enable failed." << std::endl;
            return false;
        }

        PHint = 0;
        PHint2 = 0;
        parser_->char2int(return_message_copy_[0], PHint);
        parser_->char2int(return_message_copy_[1], PHint2);
        noPorts2Enable = 16 * PHint + PHint2;

        for (int k = 0; k < noPorts2Enable; k++) {
            std::string sPH = std::string(return_message_copy_ + (2 + k * 5), 2);

            // Enable ALL configured ports
            if (isConfigured(sPH)) {
                char portHandle[2];
                portHandle[0] = sPH.c_str()[0];
                portHandle[1] = sPH.c_str()[1];

                char currentPrio;
                if (0 == sPH.compare(0, 2, sign_refPortHandle)) {
                    currentPrio = 'S';
                    std::cout << "(AuroraProtocol) Selected port " << sPH << " as Static (ref Tool)" << std::endl;
                }
                else {
                    currentPrio = 'D';
                    std::cout << "(AuroraProtocol) Selected port " << sPH << " as Dynamic" << std::endl;
                }

                if (!portHandleEnable(portHandle, currentPrio))
                {
                    std::cout << "(AuroraProtocol) port handle enable of PH " << sPH << " failed." << std::endl;
                    return false;
                }

                if (isAutoconfig(sPH)) {
                    std::cout << "(AuroraProtocol) Successfully enabled AUTOCONFIGURED port: " << sPH << std::endl;
                } else {
                    std::cout << "(AuroraProtocol) Successfully enabled configured port: " << sPH << std::endl;
                }
            } else {
                std::cout << "(AuroraProtocol) Skipping enable of unconfigured port: " << sPH << std::endl;
            }
        }
    } while (noPorts2Enable != 0);

    // Get port handles that are initialized and enabled
    if (!portHandleStatus(4, return_message_copy_))
    {
        std::cout << "(AuroraProtocol) port handle status request for enabled PH failed." << std::endl;
        return false;
    }
    PHint = 0;
    PHint2 = 0;
    no_ports_enabled_ = 0;
    parser_->char2int(return_message_copy_[0], PHint);
    parser_->char2int(return_message_copy_[1], PHint2);
    no_ports_enabled_ = 16 * PHint + PHint2;
    active_port_handles_.clear();
    for (int k = 0; k < no_ports_enabled_; k++)
        active_port_handles_.push_back(std::string(return_message_copy_ + (2 + k * 5), 2));

    std::cout << "(AuroraProtocol) Successfully configured " << no_ports_enabled_ << " port handles" << std::endl;
    return true;
}

bool AuroraProtocol::startDiagnosticMode()
{
    std::string message = "DSTART \r";
    if (!serial_->write(message.c_str(), message.length())) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: starting diagnostic mode failed " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) starting diagnostic mode: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::stopDiagnosticMode()
{
    std::string message = "DSTOP \r";
    if (!serial_->write(message.c_str(), message.length())) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: stopping diagnostic mode failed " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) stopping diagnostic mode: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::startTrackingMode(const int& trackReplyOption)
{
    switch (trackReplyOption)
    {
        case 0:
            sprintf(buffer_, "TSTART \r");
            break;

        case 80:
            sprintf(buffer_, "TSTART %i\r", trackReplyOption);
            break;

        default:
            std::cout << "(AuroraProtocol) ERROR: " << trackReplyOption << " is not a supported tracking reply option!" << std::endl;
            return false;
    }

    if (!serial_->write(buffer_, strlen(buffer_))) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: starting tracking mode failed " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) starting tracking mode: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::stopTrackingMode()
{
    std::string message = "TSTOP \r";
    if (!serial_->write(message.c_str(), message.length())) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: stopping tracking mode failed " << return_message_ << std::endl;
        return false;
    }
    std::cout << "(AuroraProtocol) stopping tracking mode: message returned: " << return_message_ << std::endl;

    return true;
}

bool AuroraProtocol::measureTool(const std::string& measureReplyOption,
                                std::vector<ToolPose>& allToolPose,
                                const bool& statusReply,
                                std::vector<int>& portIndexes,
                                std::vector<int>& visibleTools)
{
    // Initializations
    bool interpretOK = true;
    memset(return_message_, '0', 255);
    memset(buffer_, '0', 255);
    int PHcounter = 0;
    allToolPose.clear();
    visibleTools.clear();
    portIndexes.clear();

    // Measure tool command
    sprintf(buffer_, "TX %c%c%c%c\r", *measureReplyOption.c_str(), *(measureReplyOption.c_str() + 1),
            *(measureReplyOption.c_str() + 2), *(measureReplyOption.c_str() + 3));

    if (!serial_->write(buffer_, strlen(buffer_))) {
        return false;
    }

    if (!serial_->read(return_message_)) {
        return false;
    }

    // Check returnMessage - error?
    if (strncmp(return_message_, error_message_.c_str(), error_message_.length()) == 0)
    {
        std::cout << "(AuroraProtocol) ERROR: measure tool failed " << return_message_ << std::endl;
        return false;
    }

    // Length of returnMessage
    std::string tempString = return_message_;
    int messageLength = tempString.length();

    // System status check
    if (statusReply)
    {
        int systemStatus;
        parser_->char2int(return_message_[messageLength - CLcr - CLcrc - 1], systemStatus);
        if (systemStatus & 0x1) {
            std::cout << "(AuroraProtocol) system status ERROR: system communication error" << std::endl;
            return false;
        }
        if (systemStatus & 0x2) {
            std::cout << "(AuroraProtocol) system status ERROR: reserved (bit 1)" << std::endl;
            return false;
        }
        if (systemStatus & 0x4) {
            std::cout << "(AuroraProtocol) system status ERROR: system CRC error" << std::endl;
            return false;
        }
        if (systemStatus & 0x8) {
            std::cout << "(AuroraProtocol) system status ERROR: recoverable system processing exception" << std::endl;
            return false;
        }
        parser_->char2int(return_message_[messageLength - CLcr - CLcrc - 2], systemStatus);
        if (systemStatus & 0x1) {
            std::cout << "(AuroraProtocol) system status ERROR: hardware failure" << std::endl;
            return false;
        }
        if (systemStatus & 0x2) {
            std::cout << "(AuroraProtocol) system status ERROR: hardware change, Field Generator is disconnected from SCU" << std::endl;
            return false;
        }
        if (systemStatus & 0x4) {
            std::cout << "(AuroraProtocol) system status ERROR: some port handle has become occupied" << std::endl;
            return false;
        }
        if (systemStatus & 0x8) {
            std::cout << "(AuroraProtocol) system status ERROR: some port handle has become unoccupied" << std::endl;
            return false;
        }
    }

    // Check number of tools
    int PHint = 0;
    int PHint2 = 0;
    parser_->char2int(return_message_[0], PHint);
    parser_->char2int(return_message_[1], PHint2);
    int noTxPorts = 16 * PHint + PHint2;

    // Define the startindex of the reply data
    int startIndex = CLnoPH;
    unsigned int replyOptionInt;
    parser_->string2int(measureReplyOption, replyOptionInt);

    // Check all options for every handle
    bool extraTransfo = false;
    if (replyOptionInt & 0x800)
    {
        if (noTxPorts != no_ports_enabled_) {
            std::cout << "(AuroraProtocol) Error on the number of ports expected to receive: " << no_ports_enabled_ << ", Received: " << noTxPorts << std::endl;
            return false;
        }
        extraTransfo = true;
    }

    if (noTxPorts < no_ports_enabled_) {
        std::cout << "(AuroraProtocol) Warning: Received fewer ports than enabled: " << noTxPorts << " < " << no_ports_enabled_ << std::endl;
    }
    std::cout << "(AuroraProtocol) Processing " << noTxPorts << " ports from Aurora response" << std::endl;

    // Check all options for every handle
    for (int handleNumber = 1; handleNumber <= noTxPorts; handleNumber++)
    {
        // Calculate handle number that is examined
        int realHandleNumber;
        parser_->char2int(return_message_[startIndex], PHint);
        realHandleNumber = PHint * 16;
        parser_->char2int(return_message_[startIndex + 1], PHint);
        realHandleNumber = realHandleNumber + PHint;

        startIndex += CLPHno;

        ToolPose toolPose;
        double errorRMS = 0.0;
        bool missingF = false;

        if (replyOptionInt & 0x1) // check if replyoption 0001: transformation data
        {
            interpretOK = interpretOK && parser_->interpretMessage0001(return_message_, startIndex, toolPose,
                                                                       errorRMS, handleNumber, statusReply, missingF);
        }

        // Check if there is a newline character
        if (parser_->checkNewLine(return_message_, startIndex))
        {
            PHcounter++;
        }
        else
        {
            interpretOK = false;
        }
        startIndex += CLlf;

        // Put the measured toolPoses in the vector with the frames
        allToolPose.push_back(toolPose);
        portIndexes.push_back(realHandleNumber);

        // Book keeping: which tool is visible?
        if (!missingF)
        {
            visibleTools.push_back(realHandleNumber);
        }
    }

    return interpretOK;
}

bool AuroraProtocol::initToolVector()
{
    return initToolVector(no_ports_enabled_);
}

bool AuroraProtocol::initToolVector(const int& noTools)
{
    all_tool_pose_.reserve(noTools);
    visible_tools_.reserve(noTools);
    port_indexes_.reserve(noTools);
    return true;
}

bool AuroraProtocol::getNoToolsReady(int& noToolsReady)
{
    noToolsReady = no_ports_enabled_;
    return true;
}

} // namespace AuroraDriver
