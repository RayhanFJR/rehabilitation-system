//==================================================================
// FILE : server/src/trajectory/TrajectoryManager.h
//==================================================================
#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include "config.h"
#include <string>

//==================================================================
// CLASS TRAJECTORY DATA
//==================================================================

class TrajectoryData {
public:
    // Constructor
    TrajectoryData();
    
    // Load semua data trajektori
    bool loadAllTrajectories();
    
    // Switch trajektori aktif
    void switchTrajectory(int trajectory_num);
    
    // Getter untuk data aktif
    float (*getActiveGraphData())[2];
    double* getActivePos1();
    double* getActivePos2();
    double* getActivePos3();
    double* getActiveVelo1();
    double* getActiveVelo2();
    double* getActiveVelo3();
    double* getActiveFc1();
    double* getActiveFc2();
    double* getActiveFc3();
    
    // Getter untuk konfigurasi aktif
    int getActiveTrajectory() const { return trajektori_aktif; }
    int getJumlahTitikAktif() const { return JUMLAH_TITIK_AKTIF; }
    int getGrafikStartIndex() const { return GRAFIK_START_INDEX; }
    int getGrafikEndIndex() const { return GRAFIK_END_INDEX; }
    int getJumlahTitikHMI() const { return JUMLAH_TITIK_HMI; }
    int getGaitStartIndex() const { return GAIT_START_INDEX; }
    int getGaitEndIndex() const { return GAIT_END_INDEX; }
    int getJumlahTitikGait() const { return JUMLAH_TITIK_GAIT; }
    
private:
    // Data Trajektori 1
    float data_grafik_1[TrajectoryConfig::JUMLAH_TITIK_T1][2];
    double referencePos1_T1[818];
    double referencePos2_T1[818];
    double referencePos3_T1[818];
    double referenceVelo1_T1[TrajectoryConfig::JUMLAH_TITIK_T1];
    double referenceVelo2_T1[TrajectoryConfig::JUMLAH_TITIK_T1];
    double referenceVelo3_T1[TrajectoryConfig::JUMLAH_TITIK_T1];
    double referenceFc1_T1[TrajectoryConfig::JUMLAH_TITIK_T1];
    double referenceFc2_T1[TrajectoryConfig::JUMLAH_TITIK_T1];
    double referenceFc3_T1[TrajectoryConfig::JUMLAH_TITIK_T1];
    
    // Data Trajektori 2
    float data_grafik_2[TrajectoryConfig::JUMLAH_TITIK_T2][2];
    double referencePos1_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referencePos2_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referencePos3_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referenceVelo1_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referenceVelo2_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referenceVelo3_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referenceFc1_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referenceFc2_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    double referenceFc3_T2[TrajectoryConfig::JUMLAH_TITIK_T2];
    
    // Data Trajektori 3
    float data_grafik_3[TrajectoryConfig::JUMLAH_TITIK_T3][2];
    double referencePos1_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referencePos2_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referencePos3_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referenceVelo1_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referenceVelo2_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referenceVelo3_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referenceFc1_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referenceFc2_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    double referenceFc3_T3[TrajectoryConfig::JUMLAH_TITIK_T3];
    
    // Pointer aktif
    float (*data_grafik_aktif)[2];
    double *referencePos1_aktif;
    double *referencePos2_aktif;
    double *referencePos3_aktif;
    double *referenceVelo1_aktif;
    double *referenceVelo2_aktif;
    double *referenceVelo3_aktif;
    double *referenceFc1_aktif;
    double *referenceFc2_aktif;
    double *referenceFc3_aktif;
    
    // Konfigurasi aktif
    int trajektori_aktif;
    int JUMLAH_TITIK_AKTIF;
    int GRAFIK_START_INDEX;
    int GRAFIK_END_INDEX;
    int JUMLAH_TITIK_HMI;
    int GAIT_START_INDEX;
    int GAIT_END_INDEX;
    int JUMLAH_TITIK_GAIT;
    
    // Helper functions
    bool loadDataGrafik(const std::string& filename, float array[][2], int jumlah_titik);
    bool loadDoubleArray(const std::string& filename, double* array, int size);
};

#endif //  TRAJECTORY_MANAGER_H