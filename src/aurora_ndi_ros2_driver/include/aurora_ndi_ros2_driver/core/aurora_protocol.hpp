#pragma once

#include <string>
#include <vector>
#include <cstdio>

namespace AuroraDriver
{

class AuroraSerialInterface;
class AuroraMessageParser;
struct ToolPose;

/**
 * @brief Implements NDI Aurora protocol commands
 *
 * This class handles all Aurora-specific protocol operations including:
 * - System initialization and configuration
 * - Port handle management
 * - Tool definition loading
 * - Tracking mode control
 * - Measurement operations
 */
class AuroraProtocol
{
public:
    AuroraProtocol(AuroraSerialInterface* serial, AuroraMessageParser* parser);
    ~AuroraProtocol();

    /**
     * @brief Initialize Aurora system
     * @return true if successful
     */
    bool initAurora();

    /**
     * @brief Stop Aurora tracking and clear data
     * @return true if successful
     */
    bool stopAurora();

    /**
     * @brief Send beep command
     * @return true if successful
     */
    bool sendBeep();

    /**
     * @brief Get port handle status
     * @param replyOption 0=all, 1=to free, 2=to init, 3=to enable, 4=enabled
     * @param returnMessage Buffer to store response
     * @return true if successful
     */
    bool portHandleStatus(const int& replyOption, char(&returnMessage)[255]);

    /**
     * @brief Free a port handle
     * @param portHandle Port handle to free (2 char array)
     * @return true if successful
     */
    bool portHandleFree(char(&portHandle)[2]);

    /**
     * @brief Load tool definition file (.rom) to port handle
     * @param portHandle Target port handle
     * @param toolDefinition Path to .rom file
     * @return true if successful
     */
    bool loadToolDefFile(char(&portHandle)[2], char* toolDefinition);

    /**
     * @brief Initialize a port handle
     * @param portHandle Port handle to initialize
     * @return true if successful
     */
    bool portHandleInit(char(&portHandle)[2]);

    /**
     * @brief Load default tool configuration (for autoconfig ports)
     * @param portHandle Port handle to configure
     * @return true if successful
     */
    bool defaultToolConfig(char(&portHandle)[2]);

    /**
     * @brief Enable a port handle for tracking
     * @param portHandle Port handle to enable
     * @param priority 'S' for static, 'D' for dynamic
     * @return true if successful
     */
    bool portHandleEnable(char(&portHandle)[2], char priority);

    /**
     * @brief Initialize and enable port handles for wireless tools
     * @param toolDefinitions Vector of .rom file paths
     * @param toolDefPortHandles Vector of port handles to configure
     * @param portHandlesAutoconfig Vector of port handles to autoconfigure
     * @param refPortHandle Reference port handle for static tool
     * @return true if successful
     */
    bool initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                         const std::vector<std::string>& toolDefPortHandles,
                         const std::vector<std::string>& portHandlesAutoconfig,
                         const std::string& refPortHandle);

    /**
     * @brief Start diagnostic mode
     * @return true if successful
     */
    bool startDiagnosticMode();

    /**
     * @brief Stop diagnostic mode
     * @return true if successful
     */
    bool stopDiagnosticMode();

    /**
     * @brief Start tracking mode
     * @param trackReplyOption Reply option (0 or 80)
     * @return true if successful
     */
    bool startTrackingMode(const int& trackReplyOption);

    /**
     * @brief Stop tracking mode
     * @return true if successful
     */
    bool stopTrackingMode();

    /**
     * @brief Measure tool positions (main measurement function)
     * @param measureReplyOption Reply option string (e.g., "0001")
     * @param allToolPose Output vector of tool poses
     * @param statusReply Whether to print status information
     * @param portIndexes Output vector of port indexes
     * @param visibleTools Output vector of visible tool handles
     * @return true if successful
     */
    bool measureTool(const std::string& measureReplyOption,
                    std::vector<ToolPose>& allToolPose,
                    const bool& statusReply,
                    std::vector<int>& portIndexes,
                    std::vector<int>& visibleTools);

    /**
     * @brief Initialize tool vector with current number of enabled tools
     * @return true if successful
     */
    bool initToolVector();

    /**
     * @brief Initialize tool vector with specific size
     * @param noTools Number of tools
     * @return true if successful
     */
    bool initToolVector(const int& noTools);

    /**
     * @brief Get number of enabled tools
     * @param noToolsReady Output number of tools ready
     * @return true if successful
     */
    bool getNoToolsReady(int& noToolsReady);

private:
    AuroraSerialInterface* serial_;
    AuroraMessageParser* parser_;

    // Response message buffers
    char return_message_[255];
    char return_message_copy_[255];
    char buffer_[255];

    // Error/warning messages
    const std::string error_message_;
    const std::string missing_message_;
    const std::string warning_message_;

    // Port handle management
    std::vector<std::string> active_port_handles_;
    int no_ports_enabled_;

    // Tool data storage
    std::vector<ToolPose> all_tool_pose_;
    std::vector<int> port_indexes_;
    std::vector<int> visible_tools_;

    // ROM file handling
    FILE* rom_file_;

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
};

} // namespace AuroraDriver
