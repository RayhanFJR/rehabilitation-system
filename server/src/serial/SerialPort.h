//==================================================================
// FILE : server/src/serial/SerialPort.h
//==================================================================

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <vector>
#include <boost/asio.hpp>

using boost::asio::io_context;

class SerialPort {

public:
    SerialPort(io_context& io_ctx);
    ~SerialPort();
    
    // ========== CONNECTION ==========
    bool open(const std::string& port, unsigned int baud_rate = 115200);
    void close();
    bool isOpen();
    
    // ========== SEND DATA ==========
    void sendCommand(const std::string& cmd);
    void sendControlData(float pos1, float pos2, float pos3,
                        float velo1, float velo2, float velo3,
                        float fc1, float fc2, float fc3);
    void sendThresholds(int threshold1, int threshold2);
    void sendCalibrate();
    void sendEmergencyStop();
    void sendManualCommand(char direction);  // '1'=fwd, '2'=back, '0'=stop
    
    // ========== RECEIVE DATA ==========
    bool hasData();
    std::string readLine();
    std::string readUntilChar(char delimiter);
    
    // ========== DATA PARSING ==========
    struct ArduinoStatus {
        std::string status;          // "running", "paused", etc
        std::string mode;            // "forward", "retreat"
        float load;                  // Load value
        float scale;                 // Load scale factor
        float pos1, pos2, pos3;      // Motor positions
    };
    
    ArduinoStatus parseStatusMessage(const std::string& msg);
    
    // ========== DEBUG ==========
    void printPort();
    
private:
    boost::asio::serial_port serial;
    std::string port_name;
    
    // Buffer management
    std::string read_buffer;
    const int BUFFER_MAX = 1024;
};

#endif  // SERIAL_PORT_H
