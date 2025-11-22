//==================================================================
// FILE: server/src/state_machine/StateHandlers.cpp
// Implementation of all state handlers
// MODIFIED: Print IDLE message dengan interval 5 detik
//==================================================================

#include "StateHandlers.h"
#include "../modbus/ModbusServer.h"
#include "../modbus/DataHandler.h"
#include "../trajectory/TrajectoryManager.h"
#include "../serial/SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>

StateHandlers::StateHandlers(ModbusServer* modbus,
                            DataHandler* data_handler,
                            TrajectoryManager* traj_mgr,
                            SerialPort* serial)
    : modbus_server(modbus),
      data_handler(data_handler),
      trajectory_manager(traj_mgr),
      serial_port(serial),
      last_idle_print(std::chrono::steady_clock::now()) {  // Initialize
}

// ============ STATE 1: IDLE ============

void StateHandlers::handleIdle(StateMachine& state_machine,
                              int& t_controller,
                              int& t_grafik) {
    
    // Print IDLE message hanya setiap 5 detik (bukan setiap loop)
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(
        now - last_idle_print).count() >= 5) {
        
        std::cout << "\n[State] IDLE - Waiting for user input" << std::endl;
        last_idle_print = now;
    }
    
    // Check for trajectory selection
    int selected_traj = data_handler->checkTrajectorySelection();
    if (selected_traj > 0) {
        std::cout << "[State] Trajectory " << selected_traj << " selected" << std::endl;
        data_handler->loadTrajectoryToHMI(selected_traj);
    }
    
    // Check for manual controls
    if (data_handler->isButtonPressed(ModbusAddr::MANUAL_MAJU)) {
        std::cout << "[Control] Manual forward" << std::endl;
        data_handler->clearButton(ModbusAddr::MANUAL_MAJU);
        serial_port->sendManualCommand('1');
    }
    else if (data_handler->isButtonPressed(ModbusAddr::MANUAL_MUNDUR)) {
        std::cout << "[Control] Manual backward" << std::endl;
        data_handler->clearButton(ModbusAddr::MANUAL_MUNDUR);
        serial_port->sendManualCommand('2');
    }
    else if (data_handler->isButtonPressed(ModbusAddr::MANUAL_STOP)) {
        std::cout << "[Control] Manual stop" << std::endl;
        data_handler->clearButton(ModbusAddr::MANUAL_STOP);
        serial_port->sendManualCommand('0');
    }
    
    // Check for calibration
    if (data_handler->isButtonPressed(ModbusAddr::CALIBRATE)) {
        std::cout << "[Control] Calibration requested" << std::endl;
        data_handler->clearButton(ModbusAddr::CALIBRATE);
        serial_port->sendCalibrate();
        data_handler->clearAnimationData();
    }
    
    // Check for START button (begin rehabilitation)
    if (data_handler->isButtonPressed(ModbusAddr::START)) {
        std::cout << "[State] START button pressed - Beginning rehabilitation" << std::endl;
        data_handler->clearButton(ModbusAddr::START);
        
        // Get number of cycles from HMI
        int num_cycles = modbus_server->readRegister(ModbusAddr::JUMLAH_CYCLE);
        if (num_cycles < 1) num_cycles = 1;
        if (num_cycles > 10) num_cycles = 10;  // Limit to 10 cycles max
        
        // Start rehabilitation cycle
        state_machine.startRehabCycle(num_cycles);
        std::cout << "[State] Transitioning to AUTO_REHAB" << std::endl;
        state_machine.setState(SystemState::AUTO_REHAB);
        
        t_controller = 0;
        t_grafik = trajectory_manager->getGraphStartIndex();
    }
    
    // Check for emergency stop (even in IDLE)
    if (data_handler->isButtonPressed(ModbusAddr::EMERGENCY)) {
        std::cout << "[State] EMERGENCY button pressed!" << std::endl;
        data_handler->clearButton(ModbusAddr::EMERGENCY);
        serial_port->sendEmergencyStop();
        state_machine.setState(SystemState::EMERGENCY_STOP);
    }
}

// ============ STATE 2: AUTO_REHAB ============

void StateHandlers::handleAutoRehab(StateMachine& state_machine,
                                   int& t_controller,
                                   int& t_grafik,
                                   std::chrono::steady_clock::time_point& last_controller_time,
                                   bool& animasi_grafik_berjalan,
                                   std::string& arduino_feedback_state) {
    
    auto current_time = std::chrono::steady_clock::now();
    
    int gait_start = trajectory_manager->getGaitStartIndex();
    int gait_end = trajectory_manager->getGaitEndIndex();
    int graph_start = trajectory_manager->getGraphStartIndex();
    int graph_end = trajectory_manager->getGraphEndIndex();
    
    // ========== CONTROLLER INTERVAL (100ms) ==========
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_controller_time).count() >= CONTROLLER_INTERVAL_MS) {
        
        // Check if within valid range
        if (t_controller < (gait_end - gait_start)) {
            // Map controller index to actual trajectory index
            int actual_index = gait_start + t_controller;
            
            // Send trajectory point to Arduino
            TrajectoryPoint point = trajectory_manager->getPoint(actual_index);
            serial_port->sendControlData(
                point.pos1, point.pos2, point.pos3,
                point.velo1, point.velo2, point.velo3,
                point.fc1, point.fc2, point.fc3
            );
            
            // Increment for next point
            t_controller++;
            
            std::cout << "[AUTO_REHAB] Point " << t_controller 
                     << "/" << (gait_end - gait_start) << std::endl;
        } 
        else {
            // Trajectory complete
            std::cout << "\n[AUTO_REHAB] Trajectory complete" << std::endl;
            serial_port->sendManualCommand('0');  // Stop motors
            
            // Transition to POST_REHAB_DELAY
            state_machine.setState(SystemState::POST_REHAB_DELAY);
            state_machine.startDelayTimer();
            
            return;  // Don't update animation below
        }
        
        last_controller_time = current_time;
    }
    
    // ========== ANIMATION UPDATE ==========
    if (animasi_grafik_berjalan && t_grafik < graph_end) {
        TrajectoryPoint graph_point = trajectory_manager->getPoint(t_grafik);
        float* graph_x = trajectory_manager->getGraphDataX();
        float* graph_y = trajectory_manager->getGraphDataY();
        
        if (graph_x && graph_y) {
            data_handler->updateAnimationPoint(
                t_grafik - graph_start,
                graph_x[t_grafik],
                graph_y[t_grafik]
            );
        }
        
        t_grafik++;
    } 
    else if (t_grafik >= graph_end) {
        animasi_grafik_berjalan = false;
    }
    
    // ========== CHECK FOR EMERGENCY ==========
    if (data_handler->isButtonPressed(ModbusAddr::EMERGENCY)) {
        std::cout << "\n[AUTO_REHAB] EMERGENCY button pressed!" << std::endl;
        data_handler->clearButton(ModbusAddr::EMERGENCY);
        serial_port->sendEmergencyStop();
        state_machine.setState(SystemState::EMERGENCY_STOP);
    }
}

// ============ STATE 3: AUTO_RETREAT ============

void StateHandlers::handleAutoRetreat(StateMachine& state_machine,
                                     SerialPort& serial,
                                     std::string& arduino_feedback_state,
                                     std::chrono::steady_clock::time_point& last_retreat_time) {
    
    std::cout << "\n[AUTO_RETREAT] Executing retreat sequence" << std::endl;
    
    auto current_time = std::chrono::steady_clock::now();
    
    // ========== SEND RETREAT COMMANDS ==========
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_retreat_time).count() >= CONTROLLER_INTERVAL_MS) {
        
        // For now, simple retreat: just go backward
        serial.sendManualCommand('2');  // Backward
        
        last_retreat_time = current_time;
    }
    
    // ========== WAIT FOR RETREAT COMPLETE ==========
    // In real implementation, would track retreat progress
    // For now, wait 3 seconds and return to idle
    static auto retreat_start = std::chrono::steady_clock::now();
    
    if (std::chrono::duration_cast<std::chrono::seconds>(
        current_time - retreat_start).count() > 3) {
        
        std::cout << "[AUTO_RETREAT] Retreat complete" << std::endl;
        serial.sendManualCommand('0');  // Stop
        
        state_machine.setState(SystemState::IDLE);
        retreat_start = std::chrono::steady_clock::now();
    }
}

// ============ STATE 4: POST_REHAB_DELAY ============

void StateHandlers::handlePostRehabDelay(StateMachine& state_machine,
                                        int& t_controller,
                                        int& t_grafik,
                                        bool& animasi_grafik_berjalan,
                                        std::chrono::steady_clock::time_point& delay_start_time) {
    
    // Check if delay expired
    if (state_machine.isDelayTimerExpired(POST_REHAB_DELAY_SEC)) {
        std::cout << "\n[POST_REHAB_DELAY] Delay complete" << std::endl;
        
        // Check if more cycles remaining
        if (state_machine.hasCycleRemaining()) {
            state_machine.incrementCycle();
            
            std::cout << "[POST_REHAB_DELAY] Starting cycle " 
                     << state_machine.getCurrentCycle() 
                     << "/" << state_machine.getTargetCycle() << std::endl;
            
            // Reset for next cycle
            t_controller = 0;
            t_grafik = trajectory_manager->getGraphStartIndex();
            animasi_grafik_berjalan = true;
            data_handler->clearAnimationData();
            
            // Go back to AUTO_REHAB
            state_machine.setState(SystemState::AUTO_REHAB);
        } 
        else {
            // All cycles complete
            std::cout << "\n[POST_REHAB_DELAY] All cycles complete!" << std::endl;
            std::cout << "Total cycles executed: " << state_machine.getTargetCycle() << std::endl;
            
            // Return to IDLE
            state_machine.setState(SystemState::IDLE);
        }
    } 
    else {
        // Show remaining time
        int remaining = POST_REHAB_DELAY_SEC - 
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - delay_start_time).count();
        
        if (remaining > 0) {
            std::cout << "[POST_REHAB_DELAY] Waiting... " << remaining << "s remaining\r";
            std::cout.flush();
        }
    }
}

// ============ STATE 5: EMERGENCY_STOP ============

void StateHandlers::handleEmergencyStop(StateMachine& state_machine,
                                       SerialPort& serial,
                                       bool& animasi_grafik_berjalan) {
    
    std::cout << "\n[EMERGENCY_STOP] System in EMERGENCY STOP mode" << std::endl;
    std::cout << "[EMERGENCY_STOP] All motors stopped" << std::endl;
    std::cout << "[EMERGENCY_STOP] Waiting for RESET command..." << std::endl;
    
    // Clear animation
    animasi_grafik_berjalan = false;
    
    // Display message to HMI
    // (This would be done via Modbus registers in real implementation)
    
    // Wait for RESET button
    if (data_handler->isButtonPressed(ModbusAddr::RESET)) {
        std::cout << "\n[EMERGENCY_STOP] RESET button pressed" << std::endl;
        data_handler->clearButton(ModbusAddr::RESET);
        
        // Send reset to Arduino
        serial.sendCommand("R");
        
        // Transition to RESETTING
        state_machine.setState(SystemState::RESETTING);
    }
}

// ============ STATE 6: RESETTING ============

void StateHandlers::handleResetting(StateMachine& state_machine,
                                   SerialPort& serial,
                                   bool& animasi_grafik_berjalan) {
    
    std::cout << "\n[RESETTING] System resetting..." << std::endl;
    
    // Send calibration/reset command to Arduino
    serial.sendCalibrate();
    
    // Wait a bit for Arduino to reset
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Clear animation
    animasi_grafik_berjalan = false;
    data_handler->clearAnimationData();
    
    // Check if reset is complete
    if (data_handler->isButtonPressed(ModbusAddr::CALIBRATE)) {
        std::cout << "[RESETTING] Calibration complete" << std::endl;
        data_handler->clearButton(ModbusAddr::CALIBRATE);
    }
    
    std::cout << "[RESETTING] System ready - returning to IDLE" << std::endl;
    
    // Return to IDLE
    state_machine.setState(SystemState::IDLE);
}