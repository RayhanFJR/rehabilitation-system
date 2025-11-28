#include "GraphManager.h"
#include "Config.h"
#include <iostream>

GraphManager::GraphManager(ModbusHandler& modbus, TrajectoryManager& trajectory)
    : modbusHandler(modbus), trajectoryManager(trajectory), hmi_animation_counter(0) {
}

void GraphManager::resetGraphData() {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    mb_mapping->tab_registers[ModbusAddr::NUM_OF_DATA_CH0] = 0;
    mb_mapping->tab_registers[ModbusAddr::NUM_OF_DATA_CH1] = 0;
    modbusHandler.writeFloat(ModbusAddr::REALTIME_LOAD_CELL, 0.0f);
    mb_mapping->tab_registers[ModbusAddr::COMMAND_REG] = 2;
}

void GraphManager::loadTrajectoryData() {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    float (*data_grafik_aktif)[2] = trajectoryManager.getActiveGraphData();
    int grafik_start = trajectoryManager.getGraphStartIndex();
    int grafik_end = trajectoryManager.getGraphEndIndex();
    int jumlah_titik_hmi = trajectoryManager.getGraphPointCount();
    
    int hmi_index = 0;
    for (int i = grafik_start; i < grafik_end; ++i) {
        modbus_set_float_dcba(data_grafik_aktif[i][0], 
            mb_mapping->tab_registers + ModbusAddr::X_DATA_CH0_START + (hmi_index * 2));
        modbus_set_float_dcba(data_grafik_aktif[i][1], 
            mb_mapping->tab_registers + ModbusAddr::Y_DATA_CH0_START + (hmi_index * 2));
        hmi_index++;
    }
    mb_mapping->tab_registers[ModbusAddr::NUM_OF_DATA_CH0] = jumlah_titik_hmi;
    std::cout << "Trajectory " << trajectoryManager.getActiveTrajectory() 
              << " loaded to HMI: " << jumlah_titik_hmi << " points" << std::endl;
}

void GraphManager::clearChannel1Data() {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    for (int i = 0; i < (MAX_TITIK * 2); ++i) {
        mb_mapping->tab_registers[ModbusAddr::X_DATA_CH1_START + i] = 0;
        mb_mapping->tab_registers[ModbusAddr::Y_DATA_CH1_START + i] = 0;
    }
    mb_mapping->tab_registers[ModbusAddr::NUM_OF_DATA_CH1] = 0;
    hmi_animation_counter = 0;
}

void GraphManager::updateGraphAnimation(int t_grafik) {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    int grafik_start = trajectoryManager.getGraphStartIndex();
    int grafik_end = trajectoryManager.getGraphEndIndex();
    
    if (t_grafik >= grafik_start && t_grafik < grafik_end) {
        float (*data_grafik_aktif)[2] = trajectoryManager.getActiveGraphData();
        int hmi_index = t_grafik - grafik_start;
        
        modbus_set_float_dcba(data_grafik_aktif[t_grafik][0], 
            mb_mapping->tab_registers + ModbusAddr::X_DATA_CH1_START + (hmi_index * 2));
        modbus_set_float_dcba(data_grafik_aktif[t_grafik][1], 
            mb_mapping->tab_registers + ModbusAddr::Y_DATA_CH1_START + (hmi_index * 2));
        
        // Set counter ke posisi yang sesuai dengan t_grafik untuk sinkronisasi yang tepat
        hmi_animation_counter = hmi_index + 1;
        mb_mapping->tab_registers[ModbusAddr::NUM_OF_DATA_CH1] = hmi_animation_counter;
        mb_mapping->tab_registers[ModbusAddr::COMMAND_REG] = 3;
    }
}

