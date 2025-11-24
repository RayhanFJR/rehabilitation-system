//==================================================================
// FILE: server/src/serial/SerialPort.h
// UPDATED: Added read functionality
//==================================================================

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <boost/asio.hpp>
#include <string>

//==================================================================
// CLASS SERIAL COMMUNICATION
//==================================================================

class SerialCommunication {
public:
    SerialCommunication(boost::asio::io_context& io);
    ~SerialCommunication();
    
    // Inisialisasi koneksi serial
    bool initialize(const char* device_path, int baud_rate);
    
    // Kirim command ke Arduino
    void sendCommand(const std::string& cmd);
    
    // Kirim data kontroler
    void sendControllerData(int index, double* pos1, double* pos2, double* pos3,
                           double* velo1, double* velo2, double* velo3,
                           double* fc1, double* fc2, double* fc3);
    
    // Kirim data retreat
    void sendRetreatData(int index, double* pos1, double* pos2, double* pos3,
                        double* velo1, double* velo2, double* velo3,
                        double* fc1, double* fc2, double* fc3);
    
    // Baca data dari Arduino
    std::string readData();
    
    // Cek apakah ada data tersedia
    bool dataAvailable();
    
    // Close koneksi
    void close();
    
    // Get serial port reference
    boost::asio::serial_port& getPort() { return serial_port_; }
    
private:
    boost::asio::serial_port serial_port_;
};

#endif // SERIAL_PORT_H