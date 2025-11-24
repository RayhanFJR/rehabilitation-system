//==================================================================
// FILE: server/src/serial/SerialPort.cpp
// Modernized non-blocking async implementation
//==================================================================

#include "SerialPort.h"
#include <iostream>
#include <sstream>
#include <array>
#include <sys/ioctl.h>

using namespace std;
using namespace boost::asio;

SerialCommunication::SerialCommunication(io_context& io) 
    : serial_port_(io) {
}

SerialCommunication::~SerialCommunication() {
    if (serial_port_.is_open()) {
        serial_port_.close();
    }
}

bool SerialCommunication::initialize(const char* device_path, int baud_rate) {
    try {
        serial_port_.open(device_path);
        serial_port_.set_option(serial_port_base::baud_rate(baud_rate));
        cout << "Serial port opened: " << device_path 
             << " @ " << baud_rate << " baud" << endl;
        return true;
    } catch (const boost::system::system_error& e) {
        cerr << "Error membuka port serial: " << e.what() << endl;
        return false;
    }
}

void SerialCommunication::sendCommand(const string& cmd) {
    string message = cmd + "\n";
    serial_port_.write_some(buffer(message));
}

void SerialCommunication::sendControllerData(int index, 
                                            double* pos1, double* pos2, double* pos3,
                                            double* velo1, double* velo2, double* velo3,
                                            double* fc1, double* fc2, double* fc3) {
    stringstream ss;
    ss << "S" 
       << pos1[index] << "," << pos2[index] << "," << pos3[index] << ","
       << velo1[index] << "," << velo2[index] << "," << velo3[index] << ","
       << fc1[index] << "," << fc2[index] << "," << fc3[index];
    sendCommand(ss.str());
}

void SerialCommunication::sendRetreatData(int index,
                                         double* pos1, double* pos2, double* pos3,
                                         double* velo1, double* velo2, double* velo3,
                                         double* fc1, double* fc2, double* fc3) {
    stringstream ss;
    ss << "R"
       << pos1[index] << "," << pos2[index] << "," << pos3[index] << ","
       << velo1[index] << "," << velo2[index] << "," << velo3[index] << ","
       << fc1[index] << "," << fc2[index] << "," << fc3[index];
    sendCommand(ss.str());
}

bool SerialCommunication::dataAvailable() {
    int bytes_available = 0;
    ::ioctl(serial_port_.native_handle(), FIONREAD, &bytes_available);
    return bytes_available > 0;
}

string SerialCommunication::readData() {
    array<char, 256> buf;
    boost::system::error_code error;
    size_t len = serial_port_.read_some(buffer(buf), error);
    
    if (len > 0) {
        return string(buf.data(), len);
    }
    return "";
}

void SerialCommunication::close() {
    if (serial_port_.is_open()) {
        serial_port_.close();
        cout << "Serial port closed" << endl;
    }
}