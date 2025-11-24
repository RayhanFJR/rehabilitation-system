//==================================================================
// FILE: server/src/modbus/ModbusServer.cpp
// FIXED: Match with updated header file
//==================================================================

#include "ModbusServer.h"
#include <iostream>
#include <cerrno>
#include <cstring>

using namespace std;

ModbusHandler::ModbusHandler() 
    : ctx_(nullptr), mb_mapping_(nullptr), server_socket_(-1) {
}

ModbusHandler::~ModbusHandler() {
    close();
}

bool ModbusHandler::initialize(const char* ip, int port, int slave_id) {
    ctx_ = modbus_new_tcp(ip, port);
    if (ctx_ == nullptr) {
        cerr << "Error: Tidak dapat membuat context Modbus" << endl;
        return false;
    }
    
    modbus_set_slave(ctx_, slave_id);
    server_socket_ = modbus_tcp_listen(ctx_, 1);
    
    if (server_socket_ == -1) {
        cerr << "Error: Tidak dapat listen Modbus" << endl;
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }
    
    // Alokasi mapping
    mb_mapping_ = modbus_mapping_new(0, 0, 8000, 0);
    if (mb_mapping_ == nullptr) {
        cerr << "Error: Tidak dapat membuat mapping Modbus" << endl;
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }
    
    cout << "Modbus server initialized on " << ip << ":" << port << endl;
    return true;
}

bool ModbusHandler::acceptConnection() {
    cout << "Menunggu koneksi HMI..." << endl;
    if (modbus_tcp_accept(ctx_, &server_socket_) == -1) {
        cerr << "Error: Tidak dapat accept koneksi" << endl;
        return false;
    }
    cout << "Koneksi HMI diterima." << endl;
    return true;
}

int ModbusHandler::receive(uint8_t* query) {
    modbus_set_response_timeout(ctx_, 0, 10000);
    return modbus_receive(ctx_, query);
}

void ModbusHandler::reply(const uint8_t* query, int query_length) {
    modbus_reply(ctx_, query, query_length, mb_mapping_);
}

void ModbusHandler::resetGraphData() {
    mb_mapping_->tab_registers[ModbusAddr::NUM_OF_DATA_CH0] = 0;
    mb_mapping_->tab_registers[ModbusAddr::NUM_OF_DATA_CH1] = 0;
    modbus_set_float_dcba(0.0f, mb_mapping_->tab_registers + ModbusAddr::REALTIME_LOAD_CELL);
    mb_mapping_->tab_registers[ModbusAddr::COMMAND_REG] = 2;
}

void ModbusHandler::loadTrajectoryData(float (*data_grafik)[2], int start_idx, int end_idx) {
    int hmi_index = 0;
    for (int i = start_idx; i < end_idx; ++i) {
        modbus_set_float_dcba(data_grafik[i][0], 
            mb_mapping_->tab_registers + ModbusAddr::X_DATA_CH0_START + (hmi_index * 2));
        modbus_set_float_dcba(data_grafik[i][1], 
            mb_mapping_->tab_registers + ModbusAddr::Y_DATA_CH0_START + (hmi_index * 2));
        hmi_index++;
    }
    mb_mapping_->tab_registers[ModbusAddr::NUM_OF_DATA_CH0] = hmi_index;
    cout << "Trajectory loaded to HMI: " << hmi_index << " points" << endl;
}

void ModbusHandler::clearChannel1Data() {
    for (int i = 0; i < (TrajectoryConfig::MAX_TITIK * 2); ++i) {
        mb_mapping_->tab_registers[ModbusAddr::X_DATA_CH1_START + i] = 0;
        mb_mapping_->tab_registers[ModbusAddr::Y_DATA_CH1_START + i] = 0;
    }
    mb_mapping_->tab_registers[ModbusAddr::NUM_OF_DATA_CH1] = 0;
}

void ModbusHandler::updateGraphAnimation(int t_grafik, float (*data_grafik)[2], 
                                        int start_idx, int end_idx, int& counter) {
    if (t_grafik >= start_idx && t_grafik < end_idx) {
        int hmi_index = t_grafik - start_idx;
        
        modbus_set_float_dcba(data_grafik[t_grafik][0], 
            mb_mapping_->tab_registers + ModbusAddr::X_DATA_CH1_START + (hmi_index * 2));
        modbus_set_float_dcba(data_grafik[t_grafik][1], 
            mb_mapping_->tab_registers + ModbusAddr::Y_DATA_CH1_START + (hmi_index * 2));
        
        counter++;
        mb_mapping_->tab_registers[ModbusAddr::NUM_OF_DATA_CH1] = counter;
        mb_mapping_->tab_registers[ModbusAddr::COMMAND_REG] = 3;
    }
}

void ModbusHandler::updateLoadCell(float value) {
    modbus_set_float_dcba(value, mb_mapping_->tab_registers + ModbusAddr::REALTIME_LOAD_CELL);
}

void ModbusHandler::close() {
    if (mb_mapping_) {
        modbus_mapping_free(mb_mapping_);
        mb_mapping_ = nullptr;
    }
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    cout << "Modbus handler closed" << endl;
}