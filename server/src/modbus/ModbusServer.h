//==================================================================
// FILE: server/include/modbus/ModbusServer.h
// FIXED: Match dengan program asli
//==================================================================

#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

#include <modbus/modbus.h>
#include <string>
#include <cstdint>

// ========== MODBUS REGISTER MAP (SAMA PERSIS DENGAN PROGRAM ASLI) ==========
namespace ModbusAddr {
    // Control Registers
    const int MANUAL_MAJU = 99;
    const int MANUAL_STOP = 100;
    const int MANUAL_MUNDUR = 101;
    const int CALIBRATE = 102;
    const int START = 103;
    const int EMERGENCY = 104;
    const int RESET = 105;
    
    // Tombol Trajektori
    const int TRAJEKTORI_1 = 106;
    const int TRAJEKTORI_2 = 107;
    const int TRAJEKTORI_3 = 108;
    
    // Threshold Registers
    const int THRESHOLD_1 = 130;
    const int THRESHOLD_2 = 131;
    
    // Cycle Counter
    const int JUMLAH_CYCLE = 132;
    
    // Graph Registers
    const int COMMAND_REG = 120;
    const int NUM_OF_DATA_CH0 = 121;
    const int NUM_OF_DATA_CH1 = 122;
    const int REALTIME_LOAD_CELL = 126;
    const int X_DATA_CH0_START = 200;
    const int Y_DATA_CH0_START = 2000;
    const int X_DATA_CH1_START = 4000;
    const int Y_DATA_CH1_START = 6000;
}

class ModbusServer {
public:
    ModbusServer();
    ~ModbusServer();
    
    // Connection management
    bool initialize(const std::string& ip, int port);
    bool acceptConnection();
    void closeConnection();
    bool isConnected() const;
    
    // Modbus operations
    int receiveQuery(uint8_t* query, int max_length);
    void sendReply(const uint8_t* query, int query_length);
    
    // Register access
    uint16_t readRegister(int address);
    void writeRegister(int address, uint16_t value);
    bool readCoil(int address);
    void writeCoil(int address, bool value);
    
    // Float helpers (menggunakan modbus_set_float_dcba format)
    void writeFloat(int address, float value);
    float readFloat(int address);
    
    // Direct access to mapping (for batch operations)
    modbus_mapping_t* getMapping() { return mb_mapping; }

private:
    modbus_t* ctx;
    modbus_mapping_t* mb_mapping;
    int server_socket;
    bool connected;
};

#endif // MODBUS_SERVER_H