//==================================================================
// FILE: server/src/serial/SerialPort.cpp
// FINAL VERSION - Complete implementation
//==================================================================

#include "../serial/SerialPort.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <sys/ioctl.h>
#include <unistd.h>

SerialPort::SerialPort(boost::asio::io_context& io_ctx)
    : io_context(io_ctx),
      serial(io_ctx),
      is_open(false) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, unsigned int baud_rate) {
    try {
        serial.open(port);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        
        is_open = true;
        
        std::cout << "[SerialPort] Opened " << port 
                  << " at " << baud_rate << " baud" << std::endl;
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Failed to open: " << e.what() << std::endl;
        is_open = false;
        return false;
    }
}

void SerialPort::close() {
    if (is_open && serial.is_open()) {
        try {
            serial.close();
            is_open = false;
            std::cout << "[SerialPort] Closed" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "[SerialPort] Error closing: " << e.what() << std::endl;
        }
    }
}

bool SerialPort::isOpen() const {
    return is_open && serial.is_open();
}

// ========== WRITE FUNCTIONS ==========

bool SerialPort::sendCommand(const std::string& cmd) {
    if (!isOpen()) {
        std::cerr << "[SerialPort] Cannot send - port closed" << std::endl;
        return false;
    }
    
    try {
        std::string full_cmd = cmd + "\n";
        boost::asio::write(serial, boost::asio::buffer(full_cmd));
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Write error: " << e.what() << std::endl;
        return false;
    }
}

bool SerialPort::sendControlData(float pos1, float pos2, float pos3,
                                 float vel1, float vel2, float vel3,
                                 float fc1, float fc2, float fc3) {
    if (!isOpen()) return false;
    
    try {
        // Format sama dengan program asli: "S<pos1>,<pos2>,<pos3>,<vel1>,<vel2>,<vel3>,<fc1>,<fc2>,<fc3>"
        std::ostringstream oss;
        oss << "S" 
            << pos1 << "," << pos2 << "," << pos3 << ","
            << vel1 << "," << vel2 << "," << vel3 << ","
            << fc1 << "," << fc2 << "," << fc3;
        
        std::string data = oss.str();
        return sendCommand(data);
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Control write error: " << e.what() << std::endl;
        return false;
    }
}

bool SerialPort::sendManualCommand(char direction) {
    if (!isOpen()) return false;
    
    std::string cmd;
    switch (direction) {
        case '0': cmd = "0"; break;  // Stop
        case '1': cmd = "1"; break;  // Forward
        case '2': cmd = "2"; break;  // Backward
        default:
            std::cerr << "[SerialPort] Invalid manual command: " << direction << std::endl;
            return false;
    }
    
    return sendCommand(cmd);
}

bool SerialPort::sendCalibrate() {
    // Program asli pakai "X" untuk calibrate
    return sendCommand("X");
}

bool SerialPort::sendEmergencyStop() {
    // Program asli pakai "E" untuk emergency
    return sendCommand("E");
}

// ========== READ FUNCTIONS ==========

std::string SerialPort::readLine() {
    if (!isOpen()) {
        return "";
    }
    
    static std::string buffer;
    
    try {
        // Read one byte at a time (inefficient but compatible)
        char c;
        boost::system::error_code ec;
        
        while (true) {
            size_t n = boost::asio::read(serial, boost::asio::buffer(&c, 1), ec);
            
            if (ec) {
                // No more data available
                return "";
            }
            
            if (n == 1) {
                if (c == '\n') {
                    // Found newline, return accumulated buffer
                    std::string result = buffer;
                    buffer.clear();
                    
                    // Remove trailing \r if present
                    if (!result.empty() && result.back() == '\r') {
                        result.pop_back();
                    }
                    
                    return result;
                } else {
                    buffer += c;
                    
                    // Prevent buffer overflow
                    if (buffer.size() > 1024) {
                        buffer.clear();
                    }
                }
            }
        }
    }
    catch (const std::exception& e) {
        // Silently ignore - no data
    }
    
    return "";
}

std::string SerialPort::readAvailable() {
    if (!isOpen()) {
        return "";
    }
    
    try {
        // Pakai ioctl untuk check bytes available (compatible dengan semua versi Boost)
        int bytes_available = 0;
        ::ioctl(serial.native_handle(), FIONREAD, &bytes_available);
        
        if (bytes_available > 0) {
            char buffer[256];
            size_t to_read = std::min(bytes_available, 255);
            
            boost::system::error_code ec;
            size_t n = serial.read_some(boost::asio::buffer(buffer, to_read), ec);
            
            if (!ec && n > 0) {
                return std::string(buffer, n);
            }
        }
    }
    catch (const std::exception& e) {
        // Silently ignore
    }
    
    return "";
}