#include "TrajectoryManager.h"
#include "Config.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// Internal data structure
struct TrajectoryData {
    float data_grafik[MAX_TITIK][2];
    double referencePos1[MAX_TITIK + 2];
    double referencePos2[MAX_TITIK + 2];
    double referencePos3[MAX_TITIK + 2];
    double referenceVelo1[MAX_TITIK];
    double referenceVelo2[MAX_TITIK];
    double referenceVelo3[MAX_TITIK];
    double referenceFc1[MAX_TITIK];
    double referenceFc2[MAX_TITIK];
    double referenceFc3[MAX_TITIK];
    
    int jumlah_titik;
    int grafik_start_index;
    int grafik_end_index;
    int gait_start_index;
    int gait_end_index;
    
    TrajectoryData() : jumlah_titik(0), grafik_start_index(0), 
                      grafik_end_index(0), gait_start_index(0), gait_end_index(0) {
        // Initialize arrays to zero
        for (int i = 0; i < MAX_TITIK; i++) {
            data_grafik[i][0] = 0.0f;
            data_grafik[i][1] = 0.0f;
            referenceVelo1[i] = 0.0;
            referenceVelo2[i] = 0.0;
            referenceVelo3[i] = 0.0;
            referenceFc1[i] = 0.0;
            referenceFc2[i] = 0.0;
            referenceFc3[i] = 0.0;
        }
        for (int i = 0; i < MAX_TITIK + 2; i++) {
            referencePos1[i] = 0.0;
            referencePos2[i] = 0.0;
            referencePos3[i] = 0.0;
        }
    }
};

TrajectoryManager::TrajectoryManager() {
    trajectory1 = new TrajectoryData();
    trajectory2 = new TrajectoryData();
    trajectory3 = new TrajectoryData();
    activeTrajectoryData = trajectory1;
    activeTrajectory = 1;
}

TrajectoryManager::~TrajectoryManager() {
    delete trajectory1;
    delete trajectory2;
    delete trajectory3;
}

bool TrajectoryManager::loadDataGrafik(const std::string& filename, float array[][2], int expectedCount) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Tidak dapat membuka file " << filename << std::endl;
        return false;
    }
    
    int count = 0;
    std::string line;
    while (std::getline(file, line) && count < expectedCount) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            array[count][0] = std::stof(x_str);
            array[count][1] = std::stof(y_str);
            count++;
        }
    }
    
    file.close();
    std::cout << "Loaded " << count << " data points from " << filename << std::endl;
    return count == expectedCount;
}

bool TrajectoryManager::loadDoubleArray(const std::string& filename, double* array, int size) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Tidak dapat membuka file " << filename << std::endl;
        return false;
    }
    
    int count = 0;
    std::string value;
    while (file >> value && count < size) {
        array[count++] = std::stod(value);
    }
    
    file.close();
    std::cout << "Loaded " << count << " values from " << filename << std::endl;
    return count == size;
}

bool TrajectoryManager::loadTrajectory(int trajNum, const std::string& basePath) {
    TrajectoryData* traj = nullptr;
    std::string folder;
    int expectedPoints;
    
    if (trajNum == 1) {
        traj = trajectory1;
        folder = basePath + "/data/data_1/";
        expectedPoints = JUMLAH_TITIK_T1;
        traj->jumlah_titik = JUMLAH_TITIK_T1;
        traj->grafik_start_index = GRAFIK_START_INDEX_T1;
        traj->grafik_end_index = GRAFIK_END_INDEX_T1;
        traj->gait_start_index = GAIT_START_INDEX_T1;
        traj->gait_end_index = GAIT_END_INDEX_T1;
    } else if (trajNum == 2) {
        traj = trajectory2;
        folder = basePath + "/data/data_2/";
        expectedPoints = JUMLAH_TITIK_T2;
        traj->jumlah_titik = JUMLAH_TITIK_T2;
        traj->grafik_start_index = GRAFIK_START_INDEX_T2;
        traj->grafik_end_index = GRAFIK_END_INDEX_T2;
        traj->gait_start_index = GAIT_START_INDEX_T2;
        traj->gait_end_index = GAIT_END_INDEX_T2;
    } else if (trajNum == 3) {
        traj = trajectory3;
        folder = basePath + "/data/data_3/";
        expectedPoints = JUMLAH_TITIK_T3;
        traj->jumlah_titik = JUMLAH_TITIK_T3;
        traj->grafik_start_index = GRAFIK_START_INDEX_T3;
        traj->grafik_end_index = GRAFIK_END_INDEX_T3;
        traj->gait_start_index = GAIT_START_INDEX_T3;
        traj->gait_end_index = GAIT_END_INDEX_T3;
    } else {
        return false;
    }
    
    bool success = true;
    success &= loadDataGrafik(folder + "grafik.txt", traj->data_grafik, expectedPoints);
    success &= loadDoubleArray(folder + "pos1.txt", traj->referencePos1, expectedPoints + 2);
    success &= loadDoubleArray(folder + "pos2.txt", traj->referencePos2, expectedPoints + 2);
    success &= loadDoubleArray(folder + "pos3.txt", traj->referencePos3, expectedPoints + 2);
    success &= loadDoubleArray(folder + "velo1.txt", traj->referenceVelo1, expectedPoints);
    success &= loadDoubleArray(folder + "velo2.txt", traj->referenceVelo2, expectedPoints);
    success &= loadDoubleArray(folder + "velo3.txt", traj->referenceVelo3, expectedPoints);
    success &= loadDoubleArray(folder + "fc1.txt", traj->referenceFc1, expectedPoints);
    success &= loadDoubleArray(folder + "fc2.txt", traj->referenceFc2, expectedPoints);
    success &= loadDoubleArray(folder + "fc3.txt", traj->referenceFc3, expectedPoints);
    
    return success;
}

bool TrajectoryManager::loadAllTrajectoryData(const std::string& basePath) {
    std::cout << "\n=== Loading All Trajectory Data ===" << std::endl;
    
    bool success = true;
    
    std::cout << "\n[Trajectory 1 - " << JUMLAH_TITIK_T1 << " points]" << std::endl;
    success &= loadTrajectory(1, basePath);
    
    std::cout << "\n[Trajectory 2 - " << JUMLAH_TITIK_T2 << " points]" << std::endl;
    success &= loadTrajectory(2, basePath);
    
    std::cout << "\n[Trajectory 3 - " << JUMLAH_TITIK_T3 << " points]" << std::endl;
    success &= loadTrajectory(3, basePath);
    
    if (success) {
        std::cout << "\n=== All trajectory data loaded successfully ===" << std::endl;
    } else {
        std::cerr << "\n=== Error: Failed to load some trajectory data ===" << std::endl;
    }
    
    return success;
}

void TrajectoryManager::switchTrajectory(int trajNum) {
    activeTrajectory = trajNum;
    
    if (trajNum == 1) {
        activeTrajectoryData = trajectory1;
    } else if (trajNum == 2) {
        activeTrajectoryData = trajectory2;
    } else if (trajNum == 3) {
        activeTrajectoryData = trajectory3;
    }
    
    std::cout << "\n*** TRAJECTORY " << trajNum << " ACTIVE ***" << std::endl;
    std::cout << "Total points: " << activeTrajectoryData->jumlah_titik << std::endl;
    std::cout << "HMI display range: " << activeTrajectoryData->grafik_start_index 
              << " to " << activeTrajectoryData->grafik_end_index 
              << " (" << (activeTrajectoryData->grafik_end_index - activeTrajectoryData->grafik_start_index) 
              << " points)" << std::endl;
    std::cout << "Main gait cycle range: " << activeTrajectoryData->gait_start_index 
              << " to " << activeTrajectoryData->gait_end_index 
              << " (" << (activeTrajectoryData->gait_end_index - activeTrajectoryData->gait_start_index) 
              << " points)" << std::endl;
}

float (*TrajectoryManager::getActiveGraphData())[2] {
    return activeTrajectoryData->data_grafik;
}

double* TrajectoryManager::getActivePos1() const { return activeTrajectoryData->referencePos1; }
double* TrajectoryManager::getActivePos2() const { return activeTrajectoryData->referencePos2; }
double* TrajectoryManager::getActivePos3() const { return activeTrajectoryData->referencePos3; }
double* TrajectoryManager::getActiveVelo1() const { return activeTrajectoryData->referenceVelo1; }
double* TrajectoryManager::getActiveVelo2() const { return activeTrajectoryData->referenceVelo2; }
double* TrajectoryManager::getActiveVelo3() const { return activeTrajectoryData->referenceVelo3; }
double* TrajectoryManager::getActiveFc1() const { return activeTrajectoryData->referenceFc1; }
double* TrajectoryManager::getActiveFc2() const { return activeTrajectoryData->referenceFc2; }
double* TrajectoryManager::getActiveFc3() const { return activeTrajectoryData->referenceFc3; }

int TrajectoryManager::getActivePointCount() const { return activeTrajectoryData->jumlah_titik; }
int TrajectoryManager::getGraphStartIndex() const { return activeTrajectoryData->grafik_start_index; }
int TrajectoryManager::getGraphEndIndex() const { return activeTrajectoryData->grafik_end_index; }
int TrajectoryManager::getGraphPointCount() const { 
    return activeTrajectoryData->grafik_end_index - activeTrajectoryData->grafik_start_index; 
}
int TrajectoryManager::getGaitStartIndex() const { return activeTrajectoryData->gait_start_index; }
int TrajectoryManager::getGaitEndIndex() const { return activeTrajectoryData->gait_end_index; }
int TrajectoryManager::getGaitPointCount() const { 
    return activeTrajectoryData->gait_end_index - activeTrajectoryData->gait_start_index; 
}

