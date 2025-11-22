//==================================================================
// FILE: server/src/serial/SerialPort.cpp
// UPDATED: Added non-blocking read functionality
//==================================================================

#include "SerialPort.h"
#include <iostream>
#include <sstream>

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
        case '0': cmd = "0\n"; break;  // Stop
        case '1': cmd = "1\n"; break;  // Forward
        case '2': cmd = "2\n"; break;  // Backward
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

// ========== READ FUNCTIONS (NEW!) ==========

std::string SerialPort::readLine() {
    if (!isOpen()) {
        return "";
    }
    
    try {
        // Use async read with timeout instead of non_blocking
        boost::asio::streambuf buffer;
        boost::system::error_code ec;
        
        // Try to read with immediate timeout (non-blocking behavior)
        boost::asio::deadline_timer timer(io_context);
        timer.expires_from_now(boost::posix_time::milliseconds(1));
        
        // Async read
        bool data_available = false;
        std::string result;
        
        boost::asio::async_read_until(serial, buffer, '\n',
            [&](const boost::system::error_code& error, std::size_t bytes_transferred) {
                ec = error;
                if (!error && bytes_transferred > 0) {
                    std::istream is(&buffer);
                    std::getline(is, result);
                    
                    // Remove carriage return if present
                    if (!result.empty() && result.back() == '\r') {
                        result.pop_back();
                    }
                    data_available = true;
                }
            });
        
        // Run for a very short time (non-blocking)
        io_context.reset();
        io_context.poll();  // Non-blocking run
        
        if (data_available) {
            return result;
        }
    }
    catch (const std::exception& e) {
        // Silently ignore - no data available
    }
    
    return "";
}

std::string SerialPort::readAvailable() {
    if (!isOpen()) {
        return "";
    }
    
    try {
        // Set non-blocking mode
        serial.non_blocking(true);
        
        // Try to read some data
        char buffer[256];
        boost::system::error_code ec;
        
        size_t n = serial.read_some(boost::asio::buffer(buffer, sizeof(buffer)), ec);
        
        if (ec) {
            if (ec == boost::asio::error::would_block || 
                ec == boost::asio::error::try_again) {
                return "";  // No data available
            }
            std::cerr << "[SerialPort] Read error: " << ec.message() << std::endl;
            return "";
        }
        
        if (n > 0) {
            return std::string(buffer, n);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[SerialPort] Read error: " << e.what() << std::endl;
    }
    
    return "";
}