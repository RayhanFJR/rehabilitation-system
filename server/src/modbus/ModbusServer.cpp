//==================================================================
// FILE: server/src/modbus/ModbusServer.cpp
// FIXED: Match with updated header file
//==================================================================

#include "modbus/ModbusServer.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <errno.h>

// Constants
constexpr int NUM_COILS = 100;
constexpr int NUM_DISCRETE_INPUTS = 100;
constexpr int NUM_HOLDING_REGISTERS = 10000;
constexpr int NUM_INPUT_REGISTERS = 10000;

ModbusServer::ModbusServer() 
    : ctx(nullptr),
      mb_mapping(nullptr),
      server_socket(-1),
      connected(false) {
}

ModbusServer::~ModbusServer() {
    closeConnection();
    
    if (mb_mapping) {
        modbus_mapping_free(mb_mapping);
        mb_mapping = nullptr;
    }
    
    if (ctx) {
        modbus_free(ctx);
        ctx = nullptr;
    }
}

bool ModbusServer::initialize(const std::string& ip, int port) {
    std::cout << "[ModbusServer] Initializing on " << ip << ":" << port << std::endl;
    
    // Create TCP context
    ctx = modbus_new_tcp(ip.c_str(), port);
    if (!ctx) {
        std::cerr << "[ModbusServer] Failed to create Modbus context" << std::endl;
        return false;
    }
    
    // Allocate memory for registers
    mb_mapping = modbus_mapping_new(
        NUM_COILS,              // Coils
        NUM_DISCRETE_INPUTS,    // Discrete inputs
        NUM_HOLDING_REGISTERS,  // Holding registers
        NUM_INPUT_REGISTERS     // Input registers
    );
    
    if (!mb_mapping) {
        std::cerr << "[ModbusServer] Failed to allocate Modbus mapping" << std::endl;
        modbus_free(ctx);
        ctx = nullptr;
        return false;
    }
    
    // Initialize all registers to 0
    memset(mb_mapping->tab_bits, 0, NUM_COILS);
    memset(mb_mapping->tab_input_bits, 0, NUM_DISCRETE_INPUTS);
    memset(mb_mapping->tab_registers, 0, NUM_HOLDING_REGISTERS * sizeof(uint16_t));
    memset(mb_mapping->tab_input_registers, 0, NUM_INPUT_REGISTERS * sizeof(uint16_t));
    
    // Create TCP socket
    server_socket = modbus_tcp_listen(ctx, 1);
    if (server_socket == -1) {
        std::cerr << "[ModbusServer] Failed to create TCP socket: " 
                  << modbus_strerror(errno) << std::endl;
        modbus_mapping_free(mb_mapping);
        modbus_free(ctx);
        mb_mapping = nullptr;
        ctx = nullptr;
        return false;
    }
    
    std::cout << "[ModbusServer] Server initialized successfully" << std::endl;
    return true;
}

bool ModbusServer::acceptConnection() {
    if (!ctx || server_socket == -1) {
        std::cerr << "[ModbusServer] Cannot accept - server not initialized" << std::endl;
        return false;
    }
    
    std::cout << "[ModbusServer] Waiting for client connection..." << std::endl;
    
    int client_socket = modbus_tcp_accept(ctx, &server_socket);
    if (client_socket == -1) {
        std::cerr << "[ModbusServer] Failed to accept connection: " 
                  << modbus_strerror(errno) << std::endl;
        return false;
    }
    
    connected = true;
    std::cout << "[ModbusServer] Client connected!" << std::endl;
    return true;
}

void ModbusServer::closeConnection() {
    if (ctx) {
        modbus_close(ctx);
        connected = false;
        std::cout << "[ModbusServer] Connection closed" << std::endl;
    }
    
    if (server_socket != -1) {
        close(server_socket);
        server_socket = -1;
    }
}

bool ModbusServer::isConnected() const {
    return connected;
}

int ModbusServer::receiveQuery(uint8_t* query, int max_length) {
    if (!ctx) {
        return -1;
    }
    
    int rc = modbus_receive(ctx, query);
    if (rc == -1) {
        // Connection lost
        if (errno == ECONNRESET || errno == EPIPE || errno == EBADF) {
            connected = false;
        }
    }
    
    return rc;
}

void ModbusServer::sendReply(const uint8_t* query, int query_length) {
    if (!ctx || !mb_mapping) {
        return;
    }
    
    modbus_reply(ctx, query, query_length, mb_mapping);
}

uint16_t ModbusServer::readRegister(int address) {
    if (!mb_mapping || address < 0 || address >= NUM_HOLDING_REGISTERS) {
        std::cerr << "[ModbusServer] Invalid register address: " << address << std::endl;
        return 0;
    }
    
    return mb_mapping->tab_registers[address];
}

void ModbusServer::writeRegister(int address, uint16_t value) {
    if (!mb_mapping || address < 0 || address >= NUM_HOLDING_REGISTERS) {
        std::cerr << "[ModbusServer] Invalid register address: " << address << std::endl;
        return;
    }
    
    mb_mapping->tab_registers[address] = value;
}

bool ModbusServer::readCoil(int address) {
    if (!mb_mapping || address < 0 || address >= NUM_COILS) {
        std::cerr << "[ModbusServer] Invalid coil address: " << address << std::endl;
        return false;
    }
    
    return mb_mapping->tab_bits[address] != 0;
}

void ModbusServer::writeCoil(int address, bool value) {
    if (!mb_mapping || address < 0 || address >= NUM_COILS) {
        std::cerr << "[ModbusServer] Invalid coil address: " << address << std::endl;
        return;
    }
    
    mb_mapping->tab_bits[address] = value ? 1 : 0;
}

// ========== FLOAT HELPERS ==========
// Menggunakan format modbus_set_float_dcba (sama dengan program asli)

void ModbusServer::writeFloat(int address, float value) {
    if (!mb_mapping || address < 0 || address + 1 >= NUM_HOLDING_REGISTERS) {
        std::cerr << "[ModbusServer] Invalid float address: " << address << std::endl;
        return;
    }
    
    // Gunakan modbus_set_float_dcba untuk kompatibilitas dengan HMI
    modbus_set_float_dcba(value, mb_mapping->tab_registers + address);
}

float ModbusServer::readFloat(int address) {
    if (!mb_mapping || address < 0 || address + 1 >= NUM_HOLDING_REGISTERS) {
        std::cerr << "[ModbusServer] Invalid float address: " << address << std::endl;
        return 0.0f;
    }
    
    // Gunakan modbus_get_float_dcba untuk membaca
    return modbus_get_float_dcba(mb_mapping->tab_registers + address);
}