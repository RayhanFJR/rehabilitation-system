//==================================================================
// FILE : server/src/state_machine/StateMachine.h
//==================================================================

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "TrajectoryData.h"
#include "SerialCommunication.h"
#include "ModbusHandler.h"
#include <chrono>

//==================================================================
// SYSTEM STATE
//==================================================================
enum class SystemState {
    IDLE,
    AUTO_REHAB,
    POST_REHAB_DELAY,
    EMERGENCY_STOP,
    RESETTING,
    AUTO_RETREAT
};

//==================================================================
// CLASS SYSTEM CONTROLLER
//==================================================================
class SystemController {
public:
    SystemController(TrajectoryData& traj_data, 
                    SerialCommunication& serial, 
                    ModbusHandler& modbus);
    
    // Main control loop
    void run();
    
    // State handlers
    void handleIdleState();
    void handleAutoRehabState();
    void handleAutoRetreatState();
    void handlePostRehabDelay();
    void handleResettingState();
    void handleEmergencyStop();
    
private:
    // References
    TrajectoryData& traj_data_;
    SerialCommunication& serial_;
    ModbusHandler& modbus_;
    
    // State
    SystemState current_state_;
    std::string arduino_feedback_state_;
    
    // Trajectory control
    int t_controller_;
    int t_grafik_;
    bool animasi_grafik_;
    int hmi_animation_counter_;
    
    // Retreat control
    bool retreat_active_;
    int retreat_index_;
    int last_forward_index_;
    bool message_send_;
    
    // Cycle control
    int target_cycle_;
    int current_cycle_;
    
    // Threshold tracking
    int last_threshold1_;
    int last_threshold2_;
    
    // Timing
    std::chrono::steady_clock::time_point last_tra_time_;
    std::chrono::steady_clock::time_point last_grafik_time_;
    std::chrono::steady_clock::time_point delay_start_time_;
    
    // Helper functions
    void handleManualControl();
    void handleCalibration();
    void handleTrajektoriSelection();
    void updateThresholds();
    void startRehabCycle();
    void startRetreatSequence(int current_index);
    void processRetreatSequence();
    void processArduinoFeedback();
    void processAutoRehab();
    float parseValue(const std::string& data, const std::string& key);
};

#endif // STATE_MACHINE_H