#ifndef CONFIG_H
#define CONFIG_H

//==================================================================
// KONFIGURASI SISTEM REHABILITASI
//==================================================================

// Konfigurasi Trajektori
namespace TrajectoryConfig {
    const int JUMLAH_TITIK_T1 = 816;
    const int JUMLAH_TITIK_T2 = 1370;
    const int JUMLAH_TITIK_T3 = 1370;
    const int MAX_TITIK = 1370;
    
    // Filter Grafik HMI - Trajektori 1
    const int GRAFIK_START_INDEX_T1 = 101;
    const int GRAFIK_END_INDEX_T1 = 715;
    const int GAIT_START_INDEX_T1 = 101;
    const int GAIT_END_INDEX_T1 = 715;
    
    // Filter Grafik HMI - Trajektori 2
    const int GRAFIK_START_INDEX_T2 = 1;
    const int GRAFIK_END_INDEX_T2 = 1370;
    const int GAIT_START_INDEX_T2 = 165;
    const int GAIT_END_INDEX_T2 = 1177;
    
    // Filter Grafik HMI - Trajektori 3
    const int GRAFIK_START_INDEX_T3 = 1;
    const int GRAFIK_END_INDEX_T3 = 1370;
    const int GAIT_START_INDEX_T3 = 165;
    const int GAIT_END_INDEX_T3 = 1177;
}

// Konfigurasi Timing
namespace TimingConfig {
    const int JEDA_KONTROLER_MS = 100;
    const int JEDA_GRAFIK_MS = 100;
    const int POST_REHAB_DELAY_SEC = 5;
}

// Konfigurasi Serial
namespace SerialConfig {
    const char* DEVICE_PATH = "/dev/ttyACM0";
    const int BAUD_RATE = 115200;
}

// Konfigurasi Modbus
namespace ModbusConfig {
    const char* IP_ADDRESS = "0.0.0.0";
    const int PORT = 5020;
    const int SLAVE_ID = 1;
    const int MAX_CONNECTIONS = 1;
}

// Alamat Register Modbus
namespace ModbusAddr {
    // Control Registers
    const int MANUAL_MAJU = 99;
    const int MANUAL_STOP = 100;
    const int MANUAL_MUNDUR = 101;
    const int CALIBRATE = 102;
    const int START = 103;
    const int EMERGENCY = 104;
    const int RESET = 105;
    
    // Trajectory Selection
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

#endif // CONFIG_H