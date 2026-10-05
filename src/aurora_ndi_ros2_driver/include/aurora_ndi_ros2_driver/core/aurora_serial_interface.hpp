#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>

namespace AuroraDriver
{

/**
 * @brief Handles low-level serial communication with Aurora device
 *
 * This class encapsulates all serial port operations including:
 * - Opening/closing serial connection
 * - Reading/writing raw data
 * - Baud rate configuration
 */
class AuroraSerialInterface
{
public:
    AuroraSerialInterface();
    ~AuroraSerialInterface();

    /**
     * @brief Initialize serial connection
     * @param port Serial port device path (e.g., "/dev/ttyUSB0")
     * @return true if successful
     */
    bool open(const char* port);

    /**
     * @brief Close serial connection and restore terminal settings
     * @return true if successful
     */
    bool close();

    /**
     * @brief Send raw data through serial port
     * @param data Data buffer to send
     * @param length Number of bytes to send
     * @return true if successful
     */
    bool write(const char* data, int length);

    /**
     * @brief Read data from serial port until carriage return
     * @param buffer Buffer to store received data (must be at least 255 bytes)
     * @return true if successful
     */
    bool read(char (&buffer)[255]);

    /**
     * @brief Send break signal to serial port
     * @return true if successful
     */
    bool sendBreak();

    /**
     * @brief Change baud rate of Aurora device and PC serial port
     * @param baudRate Target baud rate (9600, 19200, 38400, 57600, 115200, 230400)
     * @return true if successful
     */
    bool changeBaudRate(int baudRate);

    /**
     * @brief Get file descriptor for the serial port
     * @param fd Reference to store file descriptor
     * @return true if successful
     */
    bool getFileDescriptor(int& fd) const;

    /**
     * @brief Check if serial port is open
     * @return true if open
     */
    bool isOpen() const;

private:
    bool initSerial(const char* serialdev, struct termios* oldtio, int& fd);
    bool changePCBaudRate(const char* serialdev, const speed_t& pcBaudRate);

    const char* port_;
    int fd_;
    int fd_global_;
    speed_t current_baudrate_;

    termios oldtio_;
    termios newtio_struct_;
    struct termios* newtio_;

    char read_buffer_[255];
    char* bufptr_;
    size_t bytes_read_;
};

} // namespace AuroraDriver
