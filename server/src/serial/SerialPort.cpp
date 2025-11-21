//==================================================================
// FILE : server/src/serial/SerialPort.cpp
//==================================================================

#include "SerialPort.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <sys/ioctl.h>      // <-- penting untuk FIONREAD
#include <unistd.h>

SerialPort::SerialPort(io_context& io_ctx)
    : serial(io_ctx), port_name("") {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, unsigned int baud_rate) {
    port_name = port;

    try {
        serial.open(port);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));

        std::cout << "[SerialPort] Opened " << port << " @ " << baud_rate << " baud\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[SerialPort] Error opening port: " << e.what() << std::endl;
        return false;
    }
}

void SerialPort::close() {
    if (serial.is_open()) {
        try {
            serial.close();
            std::cout << "[SerialPort] Port closed\n";
        } catch (const std::exception& e) {
            std::cerr << "[SerialPort] Error closing port: " << e.what() << std::endl;
        }
    }
}

bool SerialPort::isOpen() {
    return serial.is_open();
}

void SerialPort::sendCommand(const std::string& cmd) {
    if (!serial.is_open()) return;

    try {
        std::string message = cmd + "\n";
        boost::asio::write(serial, boost::asio::buffer(message));
    } catch (const std::exception& e) {
        std::cerr << "[SerialPort] Send error: " << e.what() << std::endl;
    }
}

void SerialPort::sendControlData(float pos1, float pos2, float pos3,
                                 float velo1, float velo2, float velo3,
                                 float fc1, float fc2, float fc3) {
    std::stringstream ss;
    ss << "S"
       << pos1 << "," << pos2 << "," << pos3 << ","
       << velo1 << "," << velo2 << "," << velo3 << ","
       << fc1 << "," << fc2 << "," << fc3;

    sendCommand(ss.str());
}

void SerialPort::sendThresholds(int threshold1, int threshold2) {
    std::stringstream ss;
    ss << "T" << threshold1 << "," << threshold2;
    sendCommand(ss.str());
}

void SerialPort::sendCalibrate() {
    sendCommand("X");
}

void SerialPort::sendEmergencyStop() {
    sendCommand("E");
}

void SerialPort::sendManualCommand(char direction) {
    std::string cmd = "";
    cmd += direction;
    sendCommand(cmd);
}


//======================================================
// FIX: Replace serial.available() with ioctl(FIONREAD)
//======================================================
static int getBytesAvailable(boost::asio::serial_port &serial)
{
    int bytes = 0;
    int fd = serial.native_handle();
    if (ioctl(fd, FIONREAD, &bytes) < 0) {
        return 0;
    }
    return bytes;
}

bool SerialPort::hasData() {
    if (!serial.is_open()) return false;
    return getBytesAvailable(serial) > 0;
}


//======================================================
// readLine() — NONBLOCKING, using FIONREAD
//======================================================
std::string SerialPort::readLine() {
    if (!serial.is_open()) return "";

    std::string line = "";
    char c;
    boost::system::error_code ec;

    while (getBytesAvailable(serial) > 0) {
        boost::asio::read(serial, boost::asio::buffer(&c, 1), ec);

        if (ec) {
            std::cerr << "[SerialPort] Read error: " << ec.message() << std::endl;
            return "";
        }

        if (c == '\n' || c == '\r') {
            if (!line.empty())
                return line;
        } else if (c >= 32 && c <= 126) {
            line += c;
        }
    }
    return "";
}


//======================================================
// readUntilChar(delimiter)
//======================================================
std::string SerialPort::readUntilChar(char delimiter) {
    if (!serial.is_open()) return "";

    std::string data = "";
    char c;
    boost::system::error_code ec;

    while (getBytesAvailable(serial) > 0) {
        boost::asio::read(serial, boost::asio::buffer(&c, 1), ec);

        if (ec) return data;
        if (c == delimiter) return data;

        data += c;
    }

    return data;
}


//======================================================
// STATUS PARSER
//======================================================
SerialPort::ArduinoStatus SerialPort::parseStatusMessage(const std::string& msg) {
    ArduinoStatus status;
    status.status = "unknown";
    status.mode = "unknown";
    status.load = 0.0;
    status.scale = 1.0;
    status.pos1 = status.pos2 = status.pos3 = 0.0;

    size_t pos = msg.find("status:");
    if (pos != std::string::npos) {
        size_t end = msg.find(',', pos);
        status.status = msg.substr(pos + 7, end - pos - 7);
    }

    pos = msg.find("mode:");
    if (pos != std::string::npos) {
        size_t end = msg.find(',', pos);
        status.mode = msg.substr(pos + 5, end - pos - 5);
    }

    pos = msg.find("load:");
    if (pos != std::string::npos) {
        status.load = std::stof(msg.substr(pos + 5));
    }

    pos = msg.find("scale:");
    if (pos != std::string::npos) {
        status.scale = std::stof(msg.substr(pos + 6));
    }

    pos = msg.find("pos:");
    if (pos != std::string::npos) {
        std::string pos_str = msg.substr(pos + 4);
        size_t c1 = pos_str.find(',');
        size_t c2 = pos_str.find(',', c1 + 1);

        status.pos1 = std::stof(pos_str.substr(0, c1));
        status.pos2 = std::stof(pos_str.substr(c1 + 1, c2 - c1 - 1));
        status.pos3 = std::stof(pos_str.substr(c2 + 1));
    }

    return status;
}

void SerialPort::printPort() {
    std::cout << "[SerialPort] Port: " << port_name
              << " | Status: " << (isOpen() ? "OPEN" : "CLOSED") << std::endl;
}
