#include "aurora_ndi_ros2_driver/core/ndi_aurora_ros2.hpp"
#include "aurora_ndi_ros2_driver/core/aurora_serial_interface.hpp"
#include "aurora_ndi_ros2_driver/core/aurora_protocol.hpp"
#include "aurora_ndi_ros2_driver/core/aurora_message_parser.hpp"

namespace AuroraDriver
{

ndi_aurora::ndi_aurora(const char* port)
    : _port(port)
{
    // Create modular components using smart pointers
    serial_ = std::make_unique<AuroraSerialInterface>();
    parser_ = std::make_unique<AuroraMessageParser>();
    protocol_ = std::make_unique<AuroraProtocol>(serial_.get(), parser_.get());
}

ndi_aurora::~ndi_aurora()
{
    // Smart pointers automatically clean up resources
}

bool ndi_aurora::startSerial()
{
    return serial_->open(_port);
}

bool ndi_aurora::stopSerial()
{
    protocol_->stopAurora();
    return serial_->close();
}

bool ndi_aurora::sendBeep()
{
    return protocol_->sendBeep();
}

bool ndi_aurora::changeBaudRate(const int& BaudRate)
{
    return serial_->changeBaudRate(BaudRate);
}

bool ndi_aurora::initAurora()
{
    return protocol_->initAurora();
}

bool ndi_aurora::stopAurora(const int& fd)
{
    (void)fd; // Unused in new implementation
    return protocol_->stopAurora();
}

bool ndi_aurora::getFd(int& fd)
{
    return serial_->getFileDescriptor(fd);
}

bool ndi_aurora::initToolVector(const int& noTools)
{
    return protocol_->initToolVector(noTools);
}

bool ndi_aurora::initToolVector()
{
    return protocol_->initToolVector();
}

bool ndi_aurora::getNoToolsReady(int& noToolsReady)
{
    return protocol_->getNoToolsReady(noToolsReady);
}

bool ndi_aurora::initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                                  const std::vector<std::string>& toolDefPortHandles,
                                  const std::vector<std::string>& defaultToolsConfiguration,
                                  const std::string& refPortHandle)
{
    return protocol_->initPortHandleWT(toolDefinitions, toolDefPortHandles,
                                      defaultToolsConfiguration, refPortHandle);
}

bool ndi_aurora::initPortHandleWT(const std::vector<std::string>& toolDefinitions,
                                  const std::vector<std::string>& toolDefPortHandles)
{
    return protocol_->initPortHandleWT(toolDefinitions, toolDefPortHandles,
                                      std::vector<std::string>(), "00");
}

bool ndi_aurora::startDiagnosticMode()
{
    return protocol_->startDiagnosticMode();
}

bool ndi_aurora::stopDiagnosticMode()
{
    return protocol_->stopDiagnosticMode();
}

bool ndi_aurora::startTrackingMode(const int& trackReplyOption)
{
    return protocol_->startTrackingMode(trackReplyOption);
}

bool ndi_aurora::stopTrackingMode()
{
    return protocol_->stopTrackingMode();
}

bool ndi_aurora::measureTool(const std::string& measureReplyOption,
                            std::vector<ToolPose>& allToolPose,
                            const bool& statusReply,
                            std::vector<int>& portIndexes,
                            std::vector<int>& visibleTools)
{
    return protocol_->measureTool(measureReplyOption, allToolPose, statusReply,
                                 portIndexes, visibleTools);
}

} // namespace AuroraDriver
