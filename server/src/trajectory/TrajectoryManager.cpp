//==================================================================
// FILE: server/src/trajectory/TrajectoryManager.cpp - UPDATED
// Updated dengan path management yang benar
//==================================================================
#include "TrajectoryManager.h"
#include "../config/PathConfig.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

using namespace std;

TrajectoryData::TrajectoryData() {
    // Inisialisasi default ke trajektori 1
    trajektori_aktif = 1;
    data_grafik_aktif = data_grafik_1;
    referencePos1_aktif = referencePos1_T1;
    referencePos2_aktif = referencePos2_T1;
    referencePos3_aktif = referencePos3_T1;
    referenceVelo1_aktif = referenceVelo1_T1;
    referenceVelo2_aktif = referenceVelo2_T1;
    referenceVelo3_aktif = referenceVelo3_T1;
    referenceFc1_aktif = referenceFc1_T1;
    referenceFc2_aktif = referenceFc2_T1;
    referenceFc3_aktif = referenceFc3_T1;
    
    JUMLAH_TITIK_AKTIF = TrajectoryConfig::JUMLAH_TITIK_T1;
    GRAFIK_START_INDEX = TrajectoryConfig::GRAFIK_START_INDEX_T1;
    GRAFIK_END_INDEX = TrajectoryConfig::GRAFIK_END_INDEX_T1;
    JUMLAH_TITIK_HMI = GRAFIK_END_INDEX - GRAFIK_START_INDEX;
    GAIT_START_INDEX = TrajectoryConfig::GAIT_START_INDEX_T1;
    GAIT_END_INDEX = TrajectoryConfig::GAIT_END_INDEX_T1;
    JUMLAH_TITIK_GAIT = GAIT_END_INDEX - GAIT_START_INDEX;
}

bool TrajectoryData::loadDataGrafik(const string& filename, float array[][2], int jumlah_titik) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Tidak dapat membuka file " << filename << endl;
        return false;
    }
    
    int count = 0;
    string line;
    while (getline(file, line) && count < jumlah_titik) {
        stringstream ss(line);
        string x_str, y_str;
        
        if (getline(ss, x_str, ',') && getline(ss, y_str, ',')) {
            array[count][0] = stof(x_str);
            array[count][1] = stof(y_str);
            count++;
        }
    }
    
    file.close();
    cout << "Loaded " << count << " data points from " << filename << endl;
    return count == jumlah_titik;
}

bool TrajectoryData::loadDoubleArray(const string& filename, double* array, int size) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Tidak dapat membuka file " << filename << endl;
        return false;
    }
    
    int count = 0;
    string value;
    while (file >> value && count < size) {
        array[count++] = stod(value);
    }
    
    file.close();
    cout << "Loaded " << count << " values from " << filename << endl;
    return count == size;
}

bool TrajectoryData::loadAllTrajectories() {
    cout << "\n=== Loading All Trajectory Data ===" << endl;
    PathConfig::printPaths();
    
    bool success = true;
    namespace fs = std::filesystem;
    
    // Load Trajektori 1
    cout << "\n[Trajektori 1 - " << TrajectoryConfig::JUMLAH_TITIK_T1 << " titik]" << endl;
    const fs::path traj1Dir = PathConfig::getTrajectoryDir(1);
    success &= loadDataGrafik((traj1Dir / "grafik.txt").string(), data_grafik_1, TrajectoryConfig::JUMLAH_TITIK_T1);
    success &= loadDoubleArray((traj1Dir / "pos1.txt").string(), referencePos1_T1, 818);
    success &= loadDoubleArray((traj1Dir / "pos2.txt").string(), referencePos2_T1, 818);
    success &= loadDoubleArray((traj1Dir / "pos3.txt").string(), referencePos3_T1, 818);
    success &= loadDoubleArray((traj1Dir / "velo1.txt").string(), referenceVelo1_T1, TrajectoryConfig::JUMLAH_TITIK_T1);
    success &= loadDoubleArray((traj1Dir / "velo2.txt").string(), referenceVelo2_T1, TrajectoryConfig::JUMLAH_TITIK_T1);
    success &= loadDoubleArray((traj1Dir / "velo3.txt").string(), referenceVelo3_T1, TrajectoryConfig::JUMLAH_TITIK_T1);
    success &= loadDoubleArray((traj1Dir / "fc1.txt").string(), referenceFc1_T1, TrajectoryConfig::JUMLAH_TITIK_T1);
    success &= loadDoubleArray((traj1Dir / "fc2.txt").string(), referenceFc2_T1, TrajectoryConfig::JUMLAH_TITIK_T1);
    success &= loadDoubleArray((traj1Dir / "fc3.txt").string(), referenceFc3_T1, TrajectoryConfig::JUMLAH_TITIK_T1);
    
    // Load Trajektori 2
    cout << "\n[Trajektori 2 - " << TrajectoryConfig::JUMLAH_TITIK_T2 << " titik]" << endl;
    const fs::path traj2Dir = PathConfig::getTrajectoryDir(2);
    success &= loadDataGrafik((traj2Dir / "grafik.txt").string(), data_grafik_2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "pos1.txt").string(), referencePos1_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "pos2.txt").string(), referencePos2_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "pos3.txt").string(), referencePos3_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "velo1.txt").string(), referenceVelo1_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "velo2.txt").string(), referenceVelo2_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "velo3.txt").string(), referenceVelo3_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "fc1.txt").string(), referenceFc1_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "fc2.txt").string(), referenceFc2_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    success &= loadDoubleArray((traj2Dir / "fc3.txt").string(), referenceFc3_T2, TrajectoryConfig::JUMLAH_TITIK_T2);
    
    // Load Trajektori 3
    cout << "\n[Trajektori 3 - " << TrajectoryConfig::JUMLAH_TITIK_T3 << " titik]" << endl;
    const fs::path traj3Dir = PathConfig::getTrajectoryDir(3);
    success &= loadDataGrafik((traj3Dir / "grafik.txt").string(), data_grafik_3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "pos1.txt").string(), referencePos1_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "pos2.txt").string(), referencePos2_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "pos3.txt").string(), referencePos3_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "velo1.txt").string(), referenceVelo1_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "velo2.txt").string(), referenceVelo2_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "velo3.txt").string(), referenceVelo3_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "fc1.txt").string(), referenceFc1_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "fc2.txt").string(), referenceFc2_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    success &= loadDoubleArray((traj3Dir / "fc3.txt").string(), referenceFc3_T3, TrajectoryConfig::JUMLAH_TITIK_T3);
    
    if (success) {
        cout << "\n=== All trajectory data loaded successfully ===" << endl;
    } else {
        cerr << "\n=== Error: Failed to load some trajectory data ===" << endl;
    }
    
    return success;
}

void TrajectoryData::switchTrajectory(int traj_num) {
    trajektori_aktif = traj_num;
    
    if (traj_num == 1) {
        data_grafik_aktif = data_grafik_1;
        referencePos1_aktif = referencePos1_T1;
        referencePos2_aktif = referencePos2_T1;
        referencePos3_aktif = referencePos3_T1;
        referenceVelo1_aktif = referenceVelo1_T1;
        referenceVelo2_aktif = referenceVelo2_T1;
        referenceVelo3_aktif = referenceVelo3_T1;
        referenceFc1_aktif = referenceFc1_T1;
        referenceFc2_aktif = referenceFc2_T1;
        referenceFc3_aktif = referenceFc3_T1;
        
        JUMLAH_TITIK_AKTIF = TrajectoryConfig::JUMLAH_TITIK_T1;
        GRAFIK_START_INDEX = TrajectoryConfig::GRAFIK_START_INDEX_T1;
        GRAFIK_END_INDEX = TrajectoryConfig::GRAFIK_END_INDEX_T1;
        JUMLAH_TITIK_HMI = GRAFIK_END_INDEX - GRAFIK_START_INDEX;
        GAIT_START_INDEX = TrajectoryConfig::GAIT_START_INDEX_T1;
        GAIT_END_INDEX = TrajectoryConfig::GAIT_END_INDEX_T1;
        JUMLAH_TITIK_GAIT = GAIT_END_INDEX - GAIT_START_INDEX;
    } 
    else if (traj_num == 2) {
        data_grafik_aktif = data_grafik_2;
        referencePos1_aktif = referencePos1_T2;
        referencePos2_aktif = referencePos2_T2;
        referencePos3_aktif = referencePos3_T2;
        referenceVelo1_aktif = referenceVelo1_T2;
        referenceVelo2_aktif = referenceVelo2_T2;
        referenceVelo3_aktif = referenceVelo3_T2;
        referenceFc1_aktif = referenceFc1_T2;
        referenceFc2_aktif = referenceFc2_T2;
        referenceFc3_aktif = referenceFc3_T2;
        
        JUMLAH_TITIK_AKTIF = TrajectoryConfig::JUMLAH_TITIK_T2;
        GRAFIK_START_INDEX = TrajectoryConfig::GRAFIK_START_INDEX_T2;
        GRAFIK_END_INDEX = TrajectoryConfig::GRAFIK_END_INDEX_T2;
        JUMLAH_TITIK_HMI = GRAFIK_END_INDEX - GRAFIK_START_INDEX;
        GAIT_START_INDEX = TrajectoryConfig::GAIT_START_INDEX_T2;
        GAIT_END_INDEX = TrajectoryConfig::GAIT_END_INDEX_T2;
        JUMLAH_TITIK_GAIT = GAIT_END_INDEX - GAIT_START_INDEX;
    }
    else if (traj_num == 3) {
        data_grafik_aktif = data_grafik_3;
        referencePos1_aktif = referencePos1_T3;
        referencePos2_aktif = referencePos2_T3;
        referencePos3_aktif = referencePos3_T3;
        referenceVelo1_aktif = referenceVelo1_T3;
        referenceVelo2_aktif = referenceVelo2_T3;
        referenceVelo3_aktif = referenceVelo3_T3;
        referenceFc1_aktif = referenceFc1_T3;
        referenceFc2_aktif = referenceFc2_T3;
        referenceFc3_aktif = referenceFc3_T3;
        
        JUMLAH_TITIK_AKTIF = TrajectoryConfig::JUMLAH_TITIK_T3;
        GRAFIK_START_INDEX = TrajectoryConfig::GRAFIK_START_INDEX_T3;
        GRAFIK_END_INDEX = TrajectoryConfig::GRAFIK_END_INDEX_T3;
        JUMLAH_TITIK_HMI = GRAFIK_END_INDEX - GRAFIK_START_INDEX;
        GAIT_START_INDEX = TrajectoryConfig::GAIT_START_INDEX_T3;
        GAIT_END_INDEX = TrajectoryConfig::GAIT_END_INDEX_T3;
        JUMLAH_TITIK_GAIT = GAIT_END_INDEX - GAIT_START_INDEX;
    }
    
    cout << "\n*** TRAJEKTORI " << traj_num << " AKTIF ***" << endl;
    cout << "Jumlah titik total: " << JUMLAH_TITIK_AKTIF << endl;
    cout << "HMI display range: " << GRAFIK_START_INDEX << " to " << GRAFIK_END_INDEX 
         << " (" << JUMLAH_TITIK_HMI << " points)" << endl;
    cout << "Main gait cycle range: " << GAIT_START_INDEX << " to " << GAIT_END_INDEX 
         << " (" << JUMLAH_TITIK_GAIT << " points)" << endl;
}

// Getter implementations
float (*TrajectoryData::getActiveGraphData())[2] { return data_grafik_aktif; }
double* TrajectoryData::getActivePos1() { return referencePos1_aktif; }
double* TrajectoryData::getActivePos2() { return referencePos2_aktif; }
double* TrajectoryData::getActivePos3() { return referencePos3_aktif; }
double* TrajectoryData::getActiveVelo1() { return referenceVelo1_aktif; }
double* TrajectoryData::getActiveVelo2() { return referenceVelo2_aktif; }
double* TrajectoryData::getActiveVelo3() { return referenceVelo3_aktif; }
double* TrajectoryData::getActiveFc1() { return referenceFc1_aktif; }
double* TrajectoryData::getActiveFc2() { return referenceFc2_aktif; }
double* TrajectoryData::getActiveFc3() { return referenceFc3_aktif; }