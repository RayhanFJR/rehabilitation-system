//==================================================================
// FILE : server/src/config/config.h
//==================================================================

#ifndef CONFIG_H
#define CONFIG_H

//==================================================================
// KONFIGURASI SISTEM REHABILITASI
//==================================================================

// Konfigurasi Trajektori
namespace TrajectoryConfig {
    constexpr int JUMLAH_TITIK_T1 = 816;
    constexpr int JUMLAH_TITIK_T2 = 1370;
    constexpr int JUMLAH_TITIK_T3 = 1370;
    constexpr int MAX_TITIK = 1370;
    
    // Filter Grafik HMI - Trajektori 1
    constexpr int GRAFIK_START_INDEX_T1 = 101;
    constexpr int GRAFIK_END_INDEX_T1 = 715;
    constexpr int GAIT_START_INDEX_T1 = 101;
    constexpr int GAIT_END_INDEX_T1 = 715;
    
    // Filter Grafik HMI - Trajektori 2
    constexpr int GRAFIK_START_INDEX_T2 = 1;
    constexpr int GRAFIK_END_INDEX_T2 = 1370;
    constexpr int GAIT_START_INDEX_T2 = 165;
    constexpr int GAIT_END_INDEX_T2 = 1177;
    
    // Filter Grafik HMI - Trajektori 3
    constexpr int GRAFIK_START_INDEX_T3 = 1;
    constexpr int GRAFIK_END_INDEX_T3 = 1370;
    constexpr int GAIT_START_INDEX_T3 = 165;
    constexpr int GAIT_END_INDEX_T3 = 1177;
}

// Konfigurasi Timing
namespace TimingConfig {
    constexpr int JEDA_KONTROLER_MS = 100;
    constexpr int JEDA_GRAFIK_MS = 100;
    constexpr int POST_REHAB_DELAY_SEC = 5;
}

// Konfigurasi Serial
namespace SerialConfig {
    // Gunakan macro untuk string literals
    #define SERIAL_DEVICE_PATH "/dev/ttyACM0"
    constexpr int BAUD_RATE = 115200;
}

// Konfigurasi Modbus
namespace ModbusConfig {
    // Gunakan macro untuk string literals
    #define MODBUS_IP_ADDRESS "0.0.0.0"
    constexpr int PORT = 5020;
    constexpr int SLAVE_ID = 1;
    constexpr int MAX_CONNECTIONS = 1;
}

// Alamat Register Modbus
namespace ModbusAddr {
    // Control Registers
    constexpr int MANUAL_MAJU = 99;
    constexpr int MANUAL_STOP = 100;
    constexpr int MANUAL_MUNDUR = 101;
    constexpr int CALIBRATE = 102;
    constexpr int START = 103;
    constexpr int EMERGENCY = 104;
    constexpr int RESET = 105;
    
    // Trajectory Selection
    constexpr int TRAJEKTORI_1 = 106;
    constexpr int TRAJEKTORI_2 = 107;
    constexpr int TRAJEKTORI_3 = 108;
    
    // Threshold Registers
    constexpr int THRESHOLD_1 = 130;
    constexpr int THRESHOLD_2 = 131;
    
    // Cycle Counter
    constexpr int JUMLAH_CYCLE = 132;
    
    // Graph Registers
    constexpr int COMMAND_REG = 120;
    constexpr int NUM_OF_DATA_CH0 = 121;
    constexpr int NUM_OF_DATA_CH1 = 122;
    constexpr int REALTIME_LOAD_CELL = 126;
    constexpr int X_DATA_CH0_START = 200;
    constexpr int Y_DATA_CH0_START = 2000;
    constexpr int X_DATA_CH1_START = 4000;
    constexpr int Y_DATA_CH1_START = 6000;
}

#endif // CONFIG_H