#pragma once

#include <vector>
#include <string>
#include <memory>

namespace AuroraDriver
{
    // Forward declarations
    class AuroraSerialInterface;
    class AuroraProtocol;
    class AuroraMessageParser;

    /**
     * @brief Data structure for tool pose (position + orientation)
     */
    struct ToolPose
    {
        double qx, qy, qz, qw;  // Quaternion orientation
        double x, y, z;          // Position in mm

        ToolPose() : qx(0), qy(0), qz(0), qw(1), x(0), y(0), z(0) {}

        ToolPose(const double& _qx, const double& _qy, const double& _qz, const double& _qw,
                const double& _x, const double& _y, const double& _z)
            : qx(_qx), qy(_qy), qz(_qz), qw(_qw), x(_x), y(_y), z(_z) {}
    };

    /**
     * @brief Facade class for NDI Aurora electromagnetic tracking system
     *
     * This class provides a simplified interface to the Aurora tracking system
     * by delegating to modular components (Serial, Protocol, Parser).
     * It maintains backward compatibility with the original monolithic interface.
     */
    class ndi_aurora
    {
    private:
        const char* _port;

        // Modular components (composition over inheritance)
        std::unique_ptr<AuroraSerialInterface> serial_;
        std::unique_ptr<AuroraProtocol> protocol_;
        std::unique_ptr<AuroraMessageParser> parser_;

    public:
        /**
         * @brief Constructor
         * @param port Serial port path (e.g., "/dev/ttyUSB0")
         */
        ndi_aurora(const char* port);

        /**
         * @brief Destructor - cleans up modular components
         */
        ~ndi_aurora();

        // Serial communication
        bool startSerial();
        bool stopSerial();
        bool changeBaudRate(const int& BaudRate);
        bool getFd(int& fd);

        // System initialization and control
        bool initAurora();
        bool stopAurora(const int& fd);
        bool sendBeep();

        // Port handle management
        bool initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                             const std::vector<std::string>& toolDefPortHandles,
                             const std::vector<std::string>& defaultToolsConfiguration,
                             const std::string& refPortHandle);

        bool initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                             const std::vector<std::string>& toolDefPortHandles);

        // Tool vector management
        bool initToolVector(const int& noTools);
        bool initToolVector();
        bool getNoToolsReady(int& noToolsReady);

        // Diagnostic mode
        bool startDiagnosticMode();
        bool stopDiagnosticMode();

        // Tracking mode
        bool startTrackingMode(const int& trackReplyOption);
        bool stopTrackingMode();

        /**
         * @brief Measure tool positions (main measurement function)
         * @param measureReplyOption Reply option string (e.g., "0001")
         * @param allToolPose Output vector of tool poses
         * @param statusReply Whether to print status information
         * @param portIndexes Output vector of port indexes
         * @param visibleTools Output vector of visible tool handles
         * @return true if measurement successful
         */
        bool measureTool(const std::string& measureReplyOption,
                        std::vector<ToolPose>& allToolPose,
                        const bool& statusReply,
                        std::vector<int>& portIndexes,
                        std::vector<int>& visibleTools);
    };

} // namespace AuroraDriver
