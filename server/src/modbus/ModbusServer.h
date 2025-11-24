//==================================================================
// FILE: server/include/modbus/ModbusServer.h
// FIXED: Match dengan program asli
//==================================================================

#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

#include <modbus/modbus.h>
#include "config.h"

//==================================================================
// CLASS MODBUS HANDLER
//==================================================================

class ModbusHandler {
public:
    ModbusHandler();
    ~ModbusHandler();
    
    // Inisialisasi Modbus server
    bool initialize(const char* ip, int port, int slave_id);
    
    // Accept koneksi dari HMI
    bool acceptConnection();
    
    // Receive data dari HMI
    int receive(uint8_t* query);
    
    // Reply ke HMI
    void reply(const uint8_t* query, int query_length);
    
    // Get mapping
    modbus_mapping_t* getMapping() { return mb_mapping_; }
    
    // Graph management
    void resetGraphData();
    void loadTrajectoryData(float (*data_grafik)[2], int start_idx, int end_idx);
    void clearChannel1Data();
    void updateGraphAnimation(int t_grafik, float (*data_grafik)[2], 
                             int start_idx, int end_idx, int& counter);
    
    // Update real-time load cell value
    void updateLoadCell(float value);
    
    // Close koneksi
    void close();
    
private:
    modbus_t* ctx_;
    modbus_mapping_t* mb_mapping_;
    int server_socket_;
};

#endif // MODBUS_SERVER_H