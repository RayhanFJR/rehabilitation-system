//==================================================================
// FILE: server/src/state_machine/StateHandlers.h
// Complete state handlers for all 6 states
//==================================================================

#ifndef STATE_HANDLERS_H
#define STATE_HANDLERS_H

#include <chrono>
#include <string>
#include "StateMachine.h"
#include "../modbus/DataHandler.h"
#include "../serial/SerialPort.h"
#include "../trajectory/TrajectoryManager.h"

// Forward declarations
class ModbusServer;
class DataHandler;
class TrajectoryManager;

class StateHandlers {
public:
    StateHandlers(ModbusServer* modbus, 
                  DataHandler* data_handler,
                  TrajectoryManager* traj_mgr,
                  SerialPort* serial);
    
    // ========== STATE HANDLERS ==========
    
    /// Handle IDLE state
    /// - Wait for user input (buttons)
    /// - Allow manual control
    /// - Accept trajectory selection
    void handleIdle(StateMachine& state_machine, 
                   int& t_controller, 
                   int& t_grafik);
    
    /// Handle AUTO_REHAB state
    /// - Execute trajectory points
    /// - Update HMI animation
    /// - Monitor for retreat
    void handleAutoRehab(StateMachine& state_machine,
                        int& t_controller,
                        int& t_grafik,
                        std::chrono::steady_clock::time_point& last_controller_time,
                        bool& animasi_grafik_berjalan,
                        std::string& arduino_feedback_state);
    
    /// Handle AUTO_RETREAT state
    /// - Execute retreat sequence backward
    /// - Scale velocity
    /// - Return to idle or post-delay
    void handleAutoRetreat(StateMachine& state_machine,
                          SerialPort& serial,
                          std::string& arduino_feedback_state,
                          std::chrono::steady_clock::time_point& last_retreat_time);
    
    /// Handle POST_REHAB_DELAY state
    /// - Wait 5 seconds between cycles
    /// - Check if more cycles remaining
    /// - Either continue or go to IDLE
    void handlePostRehabDelay(StateMachine& state_machine,
                             int& t_controller,
                             int& t_grafik,
                             bool& animasi_grafik_berjalan,
                             std::chrono::steady_clock::time_point& delay_start_time);
    
    /// Handle EMERGENCY_STOP state
    /// - Stop all motors immediately
    /// - Clear animation
    /// - Disable auto control
    /// - Wait for RESET button
    void handleEmergencyStop(StateMachine& state_machine,
                           SerialPort& serial,
                           bool& animasi_grafik_berjalan);
    
    /// Handle RESETTING state
    /// - Calibrate system
    /// - Reset counters
    /// - Return to IDLE
    void handleResetting(StateMachine& state_machine,
                        SerialPort& serial,
                        bool& animasi_grafik_berjalan);

private:
    ModbusServer* modbus_server;
    DataHandler* data_handler;
    TrajectoryManager* trajectory_manager;
    SerialPort* serial_port;
    
    // Constants
    static constexpr int POST_REHAB_DELAY_SEC = 5;
    static constexpr int CONTROLLER_INTERVAL_MS = 100;
    static constexpr int GRAFIK_INTERVAL_MS = 100;

    std::chrono::steady_clock::time_point last_idle_print;
};

#endif  // STATE_HANDLERS_H