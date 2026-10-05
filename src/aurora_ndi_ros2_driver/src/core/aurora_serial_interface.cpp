#include "aurora_ndi_ros2_driver/core/aurora_serial_interface.hpp"
#include <iostream>

namespace AuroraDriver
{

AuroraSerialInterface::AuroraSerialInterface()
    : port_(nullptr)
    , fd_(0)
    , fd_global_(0)
    , current_baudrate_(B9600)
    , newtio_(nullptr)
    , bufptr_(nullptr)
    , bytes_read_(0)
{
    memset(&oldtio_, 0, sizeof(oldtio_));
    memset(&newtio_struct_, 0, sizeof(newtio_struct_));
    memset(read_buffer_, 0, sizeof(read_buffer_));
}

AuroraSerialInterface::~AuroraSerialInterface()
{
    close();
}

bool AuroraSerialInterface::initSerial(const char* serialdev, struct termios* oldtio, int& fd)
{
    newtio_ = &newtio_struct_;

    fd = ::open(serialdev, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        std::cout << "(AuroraSerialInterface) IOException. Not able to open: " << serialdev << std::endl;
        return false;
    }

    tcgetattr(fd, oldtio);

    // After power up, the camera will reset to 9600bps
    memset(newtio_, 0, sizeof(struct termios));
    newtio_->c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    newtio_->c_iflag = IGNPAR;
    newtio_->c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio_->c_lflag = 0;
    newtio_->c_cc[VTIME] = 10;   // Inter-character timer unused
    newtio_->c_cc[VMIN] = 255;   // Blocking read until 255 chars received
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, newtio_);

    // Set the PC side as well
    ::close(fd);
    fd = ::open(serialdev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cout << "(AuroraSerialInterface) IOException PC side." << std::endl;
        return false;
    }
    newtio_->c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, newtio_);
    fd_global_ = fd;

    return true;
}

bool AuroraSerialInterface::open(const char* port)
{
    port_ = port;
    if (!initSerial(port_, &oldtio_, fd_)) {
        return false;
    }

    tcsendbreak(fd_, 0);

    char response[255];
    if (!read(response)) {
        return false;
    }

    std::cout << "(AuroraSerialInterface) Returned message for serial break: " << response << std::endl;
    return true;
}

bool AuroraSerialInterface::close()
{
    if (fd_ > 0) {
        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &oldtio_);
        ::close(fd_);
        fd_ = 0;
        fd_global_ = 0;
    }
    return true;
}

bool AuroraSerialInterface::write(const char* data, int length)
{
    if (fd_ <= 0) {
        std::cout << "(AuroraSerialInterface) Serial port not open" << std::endl;
        return false;
    }
    ::write(fd_, (const void*)data, length);
    return true;
}

bool AuroraSerialInterface::read(char (&buffer)[255])
{
    if (fd_ <= 0) {
        std::cout << "(AuroraSerialInterface) Serial port not open" << std::endl;
        return false;
    }

    bufptr_ = buffer;

    while ((bytes_read_ = ::read(fd_, bufptr_, 1)) > 0)
    {
        bufptr_ += bytes_read_;
        if (bufptr_[-1] == '\r')
        {
            break;
        }
    }

    *bufptr_ = '\0';
    return true;
}

bool AuroraSerialInterface::sendBreak()
{
    if (fd_ <= 0) {
        std::cout << "(AuroraSerialInterface) Serial port not open" << std::endl;
        return false;
    }
    tcsendbreak(fd_, 0);
    return true;
}

bool AuroraSerialInterface::changeBaudRate(int baudRate)
{
    std::string command;
    speed_t new_baudrate;

    switch(baudRate)
    {
        case 9600:
            command = "COMM 00000\r";
            new_baudrate = B9600;
            break;

        case 19200:
            command = "COMM 20000\r";
            new_baudrate = B19200;
            break;

        case 38400:
            command = "COMM 30000\r";
            new_baudrate = B38400;
            break;

        case 57600:
            command = "COMM 40000\r";
            new_baudrate = B57600;
            break;

        case 115200:
            command = "COMM 50001\r";
            new_baudrate = B115200;
            break;

        case 230400:
            command = "COMM A0000\r";
            new_baudrate = B230400;
            break;

        default:
            std::cout << "(AuroraSerialInterface) ERROR: " << baudRate
                      << " is not a supported baudrate!" << std::endl;
            return false;
    }

    // Send command to Aurora device
    if (!write(command.c_str(), command.length())) {
        return false;
    }

    char response[255];
    if (!read(response)) {
        return false;
    }

    if (strncmp(response, "ERROR", 5) == 0)
    {
        std::cout << "(AuroraSerialInterface) ERROR: could not change baudrate: "
                  << response << std::endl;
        return false;
    }

    std::cout << "(AuroraSerialInterface) Baudrate change request sent to camera: "
              << response << std::endl;

    usleep(500000);

    // Change PC side baudrate
    current_baudrate_ = new_baudrate;
    if (!changePCBaudRate(port_, current_baudrate_)) {
        return false;
    }

    std::cout << "(AuroraSerialInterface) Baudrate change completed successfully" << std::endl;
    return true;
}

bool AuroraSerialInterface::changePCBaudRate(const char* serialdev, const speed_t& pcBaudRate)
{
    ::close(fd_);
    fd_ = ::open(serialdev, O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cout << "(AuroraSerialInterface) IOException during baud rate change." << std::endl;
        return false;
    }
    newtio_->c_cflag = pcBaudRate | CS8 | CLOCAL | CREAD;
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, newtio_);
    fd_global_ = fd_;
    return true;
}

bool AuroraSerialInterface::getFileDescriptor(int& fd) const
{
    fd = fd_global_;
    return true;
}

bool AuroraSerialInterface::isOpen() const
{
    return fd_ > 0;
}

} // namespace AuroraDriver
