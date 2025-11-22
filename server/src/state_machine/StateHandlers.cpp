//==================================================================
// FILE: server/src/state_machine/StateHandlers.cpp
// FINAL VERSION - All errors fixed
//==================================================================

#include "StateHandlers.h"
#include "../modbus/ModbusServer.h"
#include "../modbus/DataHandler.h"
#include "../trajectory/TrajectoryManager.h"
#include "../serial/SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

StateHandlers::StateHandlers(ModbusServer* modbus,
                            DataHandler* data_handler,
                            TrajectoryManager* traj_mgr,
                            SerialPort* serial)
    : modbus_server(modbus),
      data_handler(data_handler),
      trajectory_manager(traj_mgr),
      serial_port(serial),
      last_idle_print(std::chrono::steady_clock::now()),
      last_feedback_read(std::chrono::steady_clock::now()),
      arduino_state("running"),
      arduino_retreat_triggered(false) {
}

// ========== ARDUINO FEEDBACK READER (DIPANGGIL SETIAP LOOP) ==========
void StateHandlers::readArduinoFeedback() {
    // Baca feedback setiap 50ms untuk menghindari buffer overflow
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_feedback_read).count() >= 50) {
        
        std::string feedback = serial_port->readLine();
        
        if (!feedback.empty()) {
            parseArduinoFeedback(feedback);
        }
        
        last_feedback_read = now;
    }
}

// ========== PARSER UNTUK DATA DARI ARDUINO ==========
void StateHandlers::parseArduinoFeedback(const std::string& feedback) {
    // Format expected from Arduino (sama dengan program asli):
    // - "RETREAT" - trigger retreat
    // - "paused" / "running" / "retreating" - state
    // - "load:12.34" - loadcell value
    
    try {
        // Check for RETREAT command
        if (feedback.find("RETREAT") != std::string::npos) {
            std::cout << "[Arduino] RETREAT command detected!" << std::endl;
            arduino_retreat_triggered = true;
        }
        
        // Check for state messages
        if (feedback.find("paused") != std::string::npos) {
            arduino_state = "paused";
            std::cout << "[Arduino] State: paused" << std::endl;
        } 
        else if (feedback.find("running") != std::string::npos) {
            arduino_state = "running";
        } 
        else if (feedback.find("retreating") != std::string::npos) {
            arduino_state = "retreating";
            std::cout << "[Arduino] State: retreating" << std::endl;
        }
        
        // Parse load cell value: "load:12.34"
        size_t load_pos = feedback.find("load:");
        if (load_pos != std::string::npos) {
            size_t value_start = load_pos + 5;  // length of "load:"
            size_t value_end = feedback.find_first_of(",\n\r ", value_start);
            
            std::string load_str = feedback.substr(value_start, 
                value_end == std::string::npos ? std::string::npos : value_end - value_start);
            
            float load_value = std::stof(load_str);
            
            // Write ke Modbus register sebagai float
            modbus_server->writeFloat(ModbusAddr::REALTIME_LOAD_CELL, load_value);
            
            std::cout << "[Arduino] Load Cell: " << load_value << " N" << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[Arduino] Parse error: " << e.what() << std::endl;
    }
}

// Getter untuk retreat trigger
bool StateHandlers::isRetreatTriggered() const {
    return arduino_retreat_triggered;
}

void StateHandlers::clearRetreatTrigger() {
    arduino_retreat_triggered = false;
}

// ============ STATE 1: IDLE ============

void StateHandlers::handleIdle(StateMachine& state_machine,
                              int& t_controller,
                              int& t_grafik) {
    
    // READ ARDUINO FEEDBACK (PENTING!)
    readArduinoFeedback();
    
    // Print IDLE message hanya setiap 5 detik
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
    
    // Check for START button
    if (data_handler->isButtonPressed(ModbusAddr::START)) {
        std::cout << "[State] START button pressed - Beginning rehabilitation" << std::endl;
        data_handler->clearButton(ModbusAddr::START);
        
        int num_cycles = modbus_server->readRegister(ModbusAddr::JUMLAH_CYCLE);
        if (num_cycles < 1) num_cycles = 1;
        if (num_cycles > 10) num_cycles = 10;
        
        state_machine.startRehabCycle(num_cycles);
        std::cout << "[State] Transitioning to AUTO_REHAB" << std::endl;
        state_machine.setState(SystemState::AUTO_REHAB);
        
        t_controller = 0;
        t_grafik = trajectory_manager->getGraphStartIndex();
    }
    
    // Check for emergency stop
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
                                   std::string& /*arduino_feedback_state*/) {
    
    // READ ARDUINO FEEDBACK (PENTING!)
    readArduinoFeedback();
    
    auto current_time = std::chrono::steady_clock::now();
    
    int gait_start = trajectory_manager->getGaitStartIndex();
    int gait_end = trajectory_manager->getGaitEndIndex();
    int graph_start = trajectory_manager->getGraphStartIndex();
    int graph_end = trajectory_manager->getGraphEndIndex();
    
    // ========== WATCHDOG: Cek timeout ==========
    static auto rehab_start_time = std::chrono::steady_clock::now();
    if (t_controller == 0) {
        rehab_start_time = current_time; // Reset di awal
    }
    
    // Timeout 60 detik (safety)
    if (std::chrono::duration_cast<std::chrono::seconds>(
        current_time - rehab_start_time).count() > 60) {
        
        std::cerr << "\n[AUTO_REHAB] TIMEOUT! Forcing stop" << std::endl;
        serial_port->sendManualCommand('0');
        state_machine.setState(SystemState::IDLE);
        return;
    }
    
    // ========== CONTROLLER INTERVAL (100ms) ==========
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_controller_time).count() >= CONTROLLER_INTERVAL_MS) {
        
        if (t_controller < (gait_end - gait_start)) {
            int actual_index = gait_start + t_controller;
            
            TrajectoryPoint point = trajectory_manager->getPoint(actual_index);
            
            bool send_success = serial_port->sendControlData(
                point.pos1, point.pos2, point.pos3,
                point.velo1, point.velo2, point.velo3,
                point.fc1, point.fc2, point.fc3
            );
            
            if (!send_success) {
                std::cerr << "[AUTO_REHAB] Failed to send data to Arduino!" << std::endl;
            }
            
            t_controller++;
            
            if (t_controller % 10 == 0) {
                std::cout << "[AUTO_REHAB] Point " << t_controller 
                         << "/" << (gait_end - gait_start) << std::endl;
            }
        } 
        else {
            std::cout << "\n[AUTO_REHAB] Trajectory complete" << std::endl;
            serial_port->sendManualCommand('0');
            
            state_machine.setState(SystemState::POST_REHAB_DELAY);
            state_machine.startDelayTimer();
            
            return;
        }
        
        last_controller_time = current_time;
    }
    
    // ========== ANIMATION UPDATE ==========
    if (animasi_grafik_berjalan && t_grafik < graph_end) {
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
                                     std::string& /*arduino_feedback_state*/,
                                     std::chrono::steady_clock::time_point& last_retreat_time) {
    
    readArduinoFeedback();
    
    static auto retreat_start = std::chrono::steady_clock::now();
    static bool retreat_initialized = false;
    
    if (!retreat_initialized) {
        std::cout << "\n[AUTO_RETREAT] Executing retreat sequence" << std::endl;
        retreat_start = std::chrono::steady_clock::now();
        retreat_initialized = true;
    }
    
    auto current_time = std::chrono::steady_clock::now();
    
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_retreat_time).count() >= CONTROLLER_INTERVAL_MS) {
        
        serial.sendManualCommand('2');
        last_retreat_time = current_time;
    }
    
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - retreat_start).count();
    
    if (elapsed > 5) {
        std::cout << "[AUTO_RETREAT] Retreat complete" << std::endl;
        serial.sendManualCommand('0');
        
        state_machine.setState(SystemState::IDLE);
        retreat_initialized = false;
    }
    else {
        std::cout << "[AUTO_RETREAT] Retreating... " << (5 - elapsed) << "s\r";
        std::cout.flush();
    }
}

// ============ STATE 4: POST_REHAB_DELAY ============

void StateHandlers::handlePostRehabDelay(StateMachine& state_machine,
                                        int& t_controller,
                                        int& t_grafik,
                                        bool& animasi_grafik_berjalan,
                                        std::chrono::steady_clock::time_point& delay_start_time) {
    
    readArduinoFeedback();
    
    if (state_machine.isDelayTimerExpired(POST_REHAB_DELAY_SEC)) {
        std::cout << "\n[POST_REHAB_DELAY] Delay complete" << std::endl;
        
        if (state_machine.hasCycleRemaining()) {
            state_machine.incrementCycle();
            
            std::cout << "[POST_REHAB_DELAY] Starting cycle " 
                     << state_machine.getCurrentCycle() 
                     << "/" << state_machine.getTargetCycle() << std::endl;
            
            t_controller = 0;
            t_grafik = trajectory_manager->getGraphStartIndex();
            animasi_grafik_berjalan = true;
            data_handler->clearAnimationData();
            
            state_machine.setState(SystemState::AUTO_REHAB);
        } 
        else {
            std::cout << "\n[POST_REHAB_DELAY] All cycles complete!" << std::endl;
            std::cout << "Total cycles executed: " << state_machine.getTargetCycle() << std::endl;
            
            state_machine.setState(SystemState::IDLE);
        }
    } 
    else {
        int elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - delay_start_time).count();
        int remaining = POST_REHAB_DELAY_SEC - elapsed;
        
        if (remaining > 0 && elapsed % 1 == 0) {
            std::cout << "[POST_REHAB_DELAY] Waiting... " << remaining << "s remaining\r";
            std::cout.flush();
        }
    }
}

// ============ STATE 5: EMERGENCY_STOP ============

void StateHandlers::handleEmergencyStop(StateMachine& state_machine,
                                       SerialPort& serial,
                                       bool& animasi_grafik_berjalan) {
    
    readArduinoFeedback();
    
    static bool emergency_logged = false;
    
    if (!emergency_logged) {
        std::cout << "\n[EMERGENCY_STOP] System in EMERGENCY STOP mode" << std::endl;
        std::cout << "[EMERGENCY_STOP] All motors stopped" << std::endl;
        std::cout << "[EMERGENCY_STOP] Waiting for RESET command..." << std::endl;
        emergency_logged = true;
    }
    
    animasi_grafik_berjalan = false;
    
    if (data_handler->isButtonPressed(ModbusAddr::RESET)) {
        std::cout << "\n[EMERGENCY_STOP] RESET button pressed" << std::endl;
        data_handler->clearButton(ModbusAddr::RESET);
        
        serial.sendCommand("R");
        
        state_machine.setState(SystemState::RESETTING);
        emergency_logged = false;
    }
}

// ============ STATE 6: RESETTING ============

void StateHandlers::handleResetting(StateMachine& state_machine,
                                   SerialPort& serial,
                                   bool& animasi_grafik_berjalan) {
    
    readArduinoFeedback();
    
    static auto reset_start_time = std::chrono::steady_clock::now();
    static bool reset_initialized = false;
    
    if (!reset_initialized) {
        std::cout << "\n[RESETTING] System resetting..." << std::endl;
        serial.sendCalibrate();
        reset_start_time = std::chrono::steady_clock::now();
        reset_initialized = true;
    }
    
    animasi_grafik_berjalan = false;
    data_handler->clearAnimationData();
    
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - reset_start_time).count();
    
    if (elapsed >= 3 || arduino_state == "READY") {
        std::cout << "[RESETTING] System ready - returning to IDLE" << std::endl;
        
        state_machine.setState(SystemState::IDLE);
        reset_initialized = false;
    }
    else {
        std::cout << "[RESETTING] Waiting for Arduino... " << (3 - elapsed) << "s\r";
        std::cout.flush();
    }
}