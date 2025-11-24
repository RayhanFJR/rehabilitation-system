//==================================================================
// FILE : server/src/state_machine/StateMachine.cpp
//==================================================================

#include "StateMachine.h"
#include <iostream>
#include <thread>
#include <cerrno>

using namespace std;

SystemController::SystemController(TrajectoryData& traj_data,
                                 SerialCommunication& serial,
                                 ModbusHandler& modbus)
    : traj_data_(traj_data), serial_(serial), modbus_(modbus),
      current_state_(SystemState::IDLE), arduino_feedback_state_("running"),
      t_controller_(0), t_grafik_(0), animasi_grafik_(false),
      hmi_animation_counter_(0), retreat_active_(false), retreat_index_(0),
      last_forward_index_(0), message_send_(false),
      target_cycle_(1), current_cycle_(0),
      last_threshold1_(-1), last_threshold2_(-1) {
    
    last_tra_time_ = chrono::steady_clock::now();
    last_grafik_time_ = chrono::steady_clock::now();
    delay_start_time_ = chrono::steady_clock::now();
}

void SystemController::handleManualControl() {
    auto* mapping = modbus_.getMapping();
    
    if (mapping->tab_registers[ModbusAddr::MANUAL_MAJU] == 1) {
        serial_.sendCommand("1");
        mapping->tab_registers[ModbusAddr::MANUAL_MAJU] = 0;
    } else if (mapping->tab_registers[ModbusAddr::MANUAL_MUNDUR] == 1) {
        serial_.sendCommand("2");
        mapping->tab_registers[ModbusAddr::MANUAL_MUNDUR] = 0;
    } else if (mapping->tab_registers[ModbusAddr::MANUAL_STOP] == 1) {
        serial_.sendCommand("0");
        mapping->tab_registers[ModbusAddr::MANUAL_STOP] = 0;
    }
}

void SystemController::handleCalibration() {
    serial_.sendCommand("X");
    cout << "\nPerintah Kalibrasi & Reset Grafik dikirim." << endl;
    
    animasi_grafik_ = false;
    modbus_.resetGraphData();
    modbus_.getMapping()->tab_registers[ModbusAddr::CALIBRATE] = 0;
}

void SystemController::handleTrajektoriSelection() {
    auto* mapping = modbus_.getMapping();
    
    if (mapping->tab_registers[ModbusAddr::TRAJEKTORI_1] == 1) {
        traj_data_.switchTrajectory(1);
        mapping->tab_registers[ModbusAddr::TRAJEKTORI_1] = 0;
    }
    else if (mapping->tab_registers[ModbusAddr::TRAJEKTORI_2] == 1) {
        traj_data_.switchTrajectory(2);
        mapping->tab_registers[ModbusAddr::TRAJEKTORI_2] = 0;
    }
    else if (mapping->tab_registers[ModbusAddr::TRAJEKTORI_3] == 1) {
        traj_data_.switchTrajectory(3);
        mapping->tab_registers[ModbusAddr::TRAJEKTORI_3] = 0;
    }
}

void SystemController::updateThresholds() {
    auto* mapping = modbus_.getMapping();
    int hmi_thresh1 = mapping->tab_registers[ModbusAddr::THRESHOLD_1];
    int hmi_thresh2 = mapping->tab_registers[ModbusAddr::THRESHOLD_2];

    if (hmi_thresh1 != last_threshold1_ || hmi_thresh2 != last_threshold2_) {
        stringstream ss;
        ss << "T" << hmi_thresh1 << "," << hmi_thresh2;
        serial_.sendCommand(ss.str());
        
        cout << "\nUpdate threshold: T1=" << hmi_thresh1 << ", T2=" << hmi_thresh2 << endl;

        last_threshold1_ = hmi_thresh1;
        last_threshold2_ = hmi_thresh2;
    }
}

void SystemController::startRehabCycle() {
    auto* mapping = modbus_.getMapping();
    
    target_cycle_ = mapping->tab_registers[ModbusAddr::JUMLAH_CYCLE];
    if (target_cycle_ < 1) target_cycle_ = 1;
    
    current_cycle_ = 1;
    
    cout << "\n=== MEMULAI REHABILITASI ===" << endl;
    cout << "Trajektori: " << traj_data_.getActiveTrajectory() << endl;
    cout << "Target Cycle: " << target_cycle_ << endl;
    cout << "Current Cycle: " << current_cycle_ << "/" << target_cycle_ << endl;
    cout << "Main gait range: " << traj_data_.getGaitStartIndex() 
         << " to " << traj_data_.getGaitEndIndex() << endl;
    
    modbus_.loadTrajectoryData(traj_data_.getActiveGraphData(),
                               traj_data_.getGrafikStartIndex(),
                               traj_data_.getGrafikEndIndex());
    modbus_.clearChannel1Data();
    modbus_.updateLoadCell(0.0f);
    
    mapping->tab_registers[ModbusAddr::COMMAND_REG] = 1;
    animasi_grafik_ = true;
    t_controller_ = 0;
    t_grafik_ = traj_data_.getGrafikStartIndex();
    hmi_animation_counter_ = 0;
    
    retreat_active_ = false;
    retreat_index_ = 0;
    last_forward_index_ = 0;
    
    last_tra_time_ = chrono::steady_clock::now();
    last_grafik_time_ = chrono::steady_clock::now();
    
    mapping->tab_registers[ModbusAddr::START] = 0;
}

void SystemController::startRetreatSequence(int current_index) {
    retreat_active_ = true;
    retreat_index_ = current_index - 1;
    last_forward_index_ = current_index;
    
    cout << "\n=== RETREAT SEQUENCE STARTED ===" << endl;
    cout << "Starting from index: " << retreat_index_ << endl;
}

float SystemController::parseValue(const string& data, const string& key) {
    size_t key_pos = data.find(key);
    if (key_pos == string::npos) return -1.0f;

    size_t value_start = key_pos + key.length();
    size_t value_end = data.find_first_of(",\n\r", value_start);
    
    try {
        return stof(data.substr(value_start, value_end - value_start));
    } catch (const exception& e) {
        return -1.0f;
    }
}

void SystemController::processArduinoFeedback() {
    if (serial_.dataAvailable()) {
        string result = serial_.readData();
        
        if (result.find("RETREAT") != string::npos && 
            current_state_ == SystemState::AUTO_REHAB) {
            cout << "\n!!! RETREAT COMMAND RECEIVED !!!" << endl;
            startRetreatSequence(t_controller_);
            current_state_ = SystemState::AUTO_RETREAT;
        }
        
        if (result.find("paused") != string::npos) {
            arduino_feedback_state_ = "paused";
        } else if (result.find("running") != string::npos) {
            arduino_feedback_state_ = "running";
        } else if (result.find("retreating") != string::npos) {
            arduino_feedback_state_ = "retreating";
        }
        
        if (current_state_ == SystemState::AUTO_REHAB || 
            current_state_ == SystemState::AUTO_RETREAT) {
            float load_value = parseValue(result, "load:");
            if (load_value != -1.0f) {
                modbus_.updateLoadCell(load_value);
            }
        } else {
            modbus_.updateLoadCell(0.0f);
        }
    }
}

void SystemController::processRetreatSequence() {
    auto current_time = chrono::steady_clock::now();
    
    if ((current_time - last_tra_time_) >= chrono::milliseconds(TimingConfig::JEDA_KONTROLER_MS)) {
        if (retreat_index_ >= 0) {
            serial_.sendRetreatData(retreat_index_,
                                   traj_data_.getActivePos1(),
                                   traj_data_.getActivePos2(),
                                   traj_data_.getActivePos3(),
                                   traj_data_.getActiveVelo1(),
                                   traj_data_.getActiveVelo2(),
                                   traj_data_.getActiveVelo3(),
                                   traj_data_.getActiveFc1(),
                                   traj_data_.getActiveFc2(),
                                   traj_data_.getActiveFc3());
            retreat_index_--;
            last_tra_time_ = current_time;
        } else {
            serial_.sendCommand("RETREAT_COMPLETE");
            serial_.sendCommand("2");
            retreat_active_ = false;
            cout << "\n=== RETREAT SEQUENCE COMPLETED ===" << endl;
        }
    }
}

void SystemController::processAutoRehab() {
    auto current_time = chrono::steady_clock::now();
    
    if ((current_time - last_tra_time_) >= chrono::milliseconds(TimingConfig::JEDA_KONTROLER_MS)) {
        if (arduino_feedback_state_ == "running") {
            if (t_controller_ < traj_data_.getJumlahTitikGait()) {
                int actual_index = traj_data_.getGaitStartIndex() + t_controller_;
                serial_.sendControllerData(actual_index,
                                          traj_data_.getActivePos1(),
                                          traj_data_.getActivePos2(),
                                          traj_data_.getActivePos3(),
                                          traj_data_.getActiveVelo1(),
                                          traj_data_.getActiveVelo2(),
                                          traj_data_.getActiveVelo3(),
                                          traj_data_.getActiveFc1(),
                                          traj_data_.getActiveFc2(),
                                          traj_data_.getActiveFc3());
                t_controller_++;
            } else {
                current_state_ = SystemState::POST_REHAB_DELAY;
                delay_start_time_ = current_time;
            }
            
            if (animasi_grafik_ && t_grafik_ < traj_data_.getGrafikEndIndex()) {
                modbus_.updateGraphAnimation(t_grafik_,
                                            traj_data_.getActiveGraphData(),
                                            traj_data_.getGrafikStartIndex(),
                                            traj_data_.getGrafikEndIndex(),
                                            hmi_animation_counter_);
                t_grafik_++;
            } else if (t_grafik_ >= traj_data_.getGrafikEndIndex()) {
                animasi_grafik_ = false;
            }
            
            last_tra_time_ = current_time;
        }
    }
}

void SystemController::handleIdleState() {
    auto* mapping = modbus_.getMapping();
    
    handleManualControl();
    handleTrajektoriSelection();
    
    if (mapping->tab_registers[ModbusAddr::CALIBRATE] == 1) {
        handleCalibration();
    }
    
    if (mapping->tab_registers[ModbusAddr::START] == 1) {
        startRehabCycle();
        current_state_ = SystemState::AUTO_REHAB;
    }
    
    updateThresholds();
}

void SystemController::handleAutoRehabState() {
    processAutoRehab();
}

void SystemController::handleAutoRetreatState() {
    auto* mapping = modbus_.getMapping();
    
    if (retreat_active_) {
        processRetreatSequence();
        return;
    }
    
    if (!retreat_active_ && !message_send_) {
        cout << "\n=== RETREAT COMPLETED ===" << endl;
        cout << "System stopped. Ready for RESET." << endl;
        message_send_ = true;
    }
    
    if (mapping->tab_registers[ModbusAddr::RESET] == 1) {
        serial_.sendCommand("R");
        cout << "\nSistem di-reset." << endl;
        mapping->tab_registers[ModbusAddr::RESET] = 0;
        arduino_feedback_state_ = "running";
        retreat_active_ = false;
        retreat_index_ = 0;
        message_send_ = false;
        current_state_ = SystemState::RESETTING;
    }
}

void SystemController::handlePostRehabDelay() {
    if ((chrono::steady_clock::now() - delay_start_time_) >= 
        chrono::seconds(TimingConfig::POST_REHAB_DELAY_SEC)) {
        
        if (current_cycle_ < target_cycle_) {
            current_cycle_++;
            cout << "\n=== MELANJUTKAN KE CYCLE " << current_cycle_ 
                 << "/" << target_cycle_ << " ===" << endl;
            
            modbus_.clearChannel1Data();
            t_controller_ = 0;
            t_grafik_ = traj_data_.getGrafikStartIndex();
            animasi_grafik_ = true;
            hmi_animation_counter_ = 0;
            
            last_tra_time_ = chrono::steady_clock::now();
            last_grafik_time_ = chrono::steady_clock::now();
            
            current_state_ = SystemState::AUTO_REHAB;
        } 
        else {
            serial_.sendCommand("0");
            cout << "\n=== SEMUA CYCLE SELESAI ===" << endl;
            cout << "Total cycle completed: " << current_cycle_ << endl;
            cout << "Kembali ke IDLE." << endl;
            
            current_cycle_ = 0;
            current_state_ = SystemState::IDLE;
        }
    }
}

void SystemController::handleResettingState() {
    auto* mapping = modbus_.getMapping();
    
    if (mapping->tab_registers[ModbusAddr::CALIBRATE] == 1) {
        cout << "\nSistem Siap. Kembali ke mode IDLE." << endl;
        mapping->tab_registers[ModbusAddr::CALIBRATE] = 0;
        mapping->tab_registers[ModbusAddr::START] = 0;
        current_state_ = SystemState::IDLE;
    }
}

void SystemController::handleEmergencyStop() {
    auto* mapping = modbus_.getMapping();
    
    if (mapping->tab_registers[ModbusAddr::RESET] == 1) {
        current_state_ = SystemState::RESETTING;
        retreat_active_ = false;
        serial_.sendCommand("R");
        cout << "\nSistem di-reset dari Emergency Stop." << endl;
        mapping->tab_registers[ModbusAddr::RESET] = 0;
    }
}

void SystemController::run() {
    cout << "\n===========================================";
    cout << "\n  SISTEM KONTROL REHABILITASI";
    cout << "\n  Multi-Trajectory + Cycle Counter";
    cout << "\n===========================================" << endl;
    
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    
    while (true) {
        int rc = modbus_.receive(query);
        
        if (rc > 0) {
            auto* mapping = modbus_.getMapping();
            
            // Check emergency stop
            if (mapping->tab_registers[ModbusAddr::EMERGENCY] == 1) {
                if (current_state_ != SystemState::EMERGENCY_STOP) {
                    current_state_ = SystemState::EMERGENCY_STOP;
                    animasi_grafik_ = false;
                    retreat_active_ = false;
                    serial_.sendCommand("E");
                    cout << "\n!!! EMERGENCY STOP DIAKTIFKAN !!!" << endl;
                }
                mapping->tab_registers[ModbusAddr::EMERGENCY] = 0;
            }
            
            // Handle states
            switch (current_state_) {
                case SystemState::IDLE:
                    handleIdleState();
                    break;
                    
                case SystemState::AUTO_REHAB:
                    handleAutoRehabState();
                    break;
                    
                case SystemState::AUTO_RETREAT:
                    handleAutoRetreatState();
                    break;
                    
                case SystemState::POST_REHAB_DELAY:
                    handlePostRehabDelay();
                    break;
                    
                case SystemState::RESETTING:
                    handleResettingState();
                    break;
                    
                case SystemState::EMERGENCY_STOP:
                    handleEmergencyStop();
                    break;
            }
            
            modbus_.reply(query, rc);
        }
        else if (rc == -1) {
            if (errno != EWOULDBLOCK && errno != EAGAIN) {
                cerr << "\nKoneksi HMI terputus: " << modbus_strerror(errno) 
                     << ". Menunggu koneksi baru..." << endl;
                modbus_.close();
                if (!modbus_.acceptConnection()) {
                    cerr << "Gagal reconnect. Keluar..." << endl;
                    break;
                }
                cout << "Koneksi HMI berhasil disambungkan kembali." << endl;
            }
        }
        
        // Process Arduino feedback
        processArduinoFeedback();
        
        // Small delay
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}