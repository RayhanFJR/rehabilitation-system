//==================================================================
// FILE: server/src/main_server.cpp
// Updated main loop with complete state handling
//==================================================================

#include <iostream>
#include <thread>
#include <chrono>
#include <modbus/modbus.h>

#include "trajectory/TrajectoryManager.h"
#include "modbus/ModbusServer.h"
#include "modbus/DataHandler.h"
#include "serial/SerialPort.h"
#include "state_machine/StateMachine.h"
#include "state_machine/StateHandlers.h"

using boost::asio::io_context;

// Global objects
TrajectoryManager traj_manager;
ModbusServer modbus_server;
DataHandler data_handler(&modbus_server, &traj_manager);
io_context io_ctx;
SerialPort serial_port(io_ctx);
StateMachine state_machine;
StateHandlers state_handlers(&modbus_server, &data_handler, 
                             &traj_manager, &serial_port);

// Timing
auto last_controller_time = std::chrono::steady_clock::now();
auto delay_start_time = std::chrono::steady_clock::now();
auto last_retreat_time = std::chrono::steady_clock::now();

// Trajectory tracking
int trajectory_index = 0;
int animation_counter = 0;
bool animasi_grafik_berjalan = false;
std::string arduino_feedback_state = "running";

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "  SISTEM KONTROL REHABILITASI" << std::endl;
    std::cout << "  Multi-Trajectory + Cycle Counter" << std::endl;
    std::cout << "  Complete State Machine" << std::endl;
    std::cout << "==========================================\n" << std::endl;
    
    // Initialize
    std::cout << "Initializing Modbus Server..." << std::endl;
    if (!modbus_server.initialize("0.0.0.0", 5020)) {
        std::cerr << "Failed to initialize Modbus server" << std::endl;
        return 1;
    }
    modbus_server.acceptConnection();
    
    std::cout << "Loading trajectory data..." << std::endl;
    if (!traj_manager.loadAllTrajectories()) {
        std::cerr << "Failed to load trajectories" << std::endl;
        return 1;
    }
    
    std::cout << "Connecting to Arduino..." << std::endl;
    if (!serial_port.open("/dev/ttyACM0", 115200)) {
        std::cerr << "Warning: Could not connect to Arduino" << std::endl;
    }
    
    // Load default trajectory
    data_handler.loadTrajectoryToHMI(1);
    
    std::cout << "\n=== SYSTEM READY ===" << std::endl;
    std::cout << "Initial State: IDLE" << std::endl;
    std::cout << "Waiting for HMI commands...\n" << std::endl;
    
    // ========== MAIN CONTROL LOOP ==========
    while (true) {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc = modbus_server.receiveQuery(query, sizeof(query));
        
        if (rc > 0) {
            modbus_server.sendReply(query, rc);
        }
        
        // ========== STATE MACHINE ==========
        switch (state_machine.getState()) {
            case SystemState::IDLE:
                state_handlers.handleIdle(state_machine, trajectory_index, animation_counter);
                break;
            
            case SystemState::AUTO_REHAB:
                state_handlers.handleAutoRehab(
                    state_machine, 
                    trajectory_index, 
                    animation_counter,
                    last_controller_time,
                    animasi_grafik_berjalan,
                    arduino_feedback_state
                );
                break;
            
            case SystemState::AUTO_RETREAT:
                state_handlers.handleAutoRetreat(
                    state_machine, 
                    serial_port,
                    arduino_feedback_state,
                    last_retreat_time
                );
                break;
            
            case SystemState::POST_REHAB_DELAY:
                state_handlers.handlePostRehabDelay(
                    state_machine,
                    trajectory_index,
                    animation_counter,
                    animasi_grafik_berjalan,
                    delay_start_time
                );
                break;
            
            case SystemState::EMERGENCY_STOP:
                state_handlers.handleEmergencyStop(
                    state_machine,
                    serial_port,
                    animasi_grafik_berjalan
                );
                break;
            
            case SystemState::RESETTING:
                state_handlers.handleResetting(
                    state_machine,
                    serial_port,
                    animasi_grafik_berjalan
                );
                break;
        }
        
        // Small delay to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // Cleanup
    serial_port.close();
    modbus_server.closeConnection();
    
    return 0;
}