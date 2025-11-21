//==================================================================
// FILE : server/src/main_server.cpp - INTEGRATION (FIXED VERSION)
//==================================================================

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>

#include "trajectory/TrajectoryManager.h"
#include "modbus/ModbusServer.h"
#include "modbus/DataHandler.h"
#include "serial/SerialPort.h"
#include "state_machine/StateMachine.h"

using boost::asio::io_context;

// ============ GLOBAL VARIABLES ============
TrajectoryManager traj_manager;
ModbusServer modbus_server;
DataHandler data_handler(&modbus_server, &traj_manager);
io_context io_ctx;
SerialPort arduino_serial(io_ctx);   // <-- FIXED: rename from serial_port
StateMachine state_machine;

const int CONTROLLER_INTERVAL_MS = 100;
const int GRAFIK_INTERVAL_MS = 100;
const int POST_REHAB_DELAY_SEC = 5;

int trajectory_index = 0;
int animation_counter = 0;

// ==================== STATE HANDLERS ====================

void handleIdleState() {
    if (data_handler.isButtonPressed(ModbusAddr::START)) {
        data_handler.clearButton(ModbusAddr::START);

        int num_cycles = modbus_server.readRegister(ModbusAddr::JUMLAH_CYCLE);
        if (num_cycles < 1) num_cycles = 1;

        state_machine.startRehabCycle(num_cycles);

        std::cout << "\n=== STARTING REHABILITATION ===\n";
        std::cout << "Target cycles: " << num_cycles << std::endl;
    }

    int selected_traj = data_handler.checkTrajectorySelection();
    if (selected_traj > 0) 
        data_handler.loadTrajectoryToHMI(selected_traj);

    if (data_handler.isButtonPressed(ModbusAddr::MANUAL_MAJU)) {
        data_handler.clearButton(ModbusAddr::MANUAL_MAJU);
        arduino_serial.sendManualCommand('1');
    } 
    else if (data_handler.isButtonPressed(ModbusAddr::MANUAL_MUNDUR)) {
        data_handler.clearButton(ModbusAddr::MANUAL_MUNDUR);
        arduino_serial.sendManualCommand('2');
    } 
    else if (data_handler.isButtonPressed(ModbusAddr::MANUAL_STOP)) {
        data_handler.clearButton(ModbusAddr::MANUAL_STOP);
        arduino_serial.sendManualCommand('0');
    }

    if (data_handler.isButtonPressed(ModbusAddr::CALIBRATE)) {
        data_handler.clearButton(ModbusAddr::CALIBRATE);
        arduino_serial.sendCalibrate();
        data_handler.clearAnimationData();
    }
}

void handleAutoRehabState() {
    int gait_start = traj_manager.getGaitStartIndex();
    int gait_end = traj_manager.getGaitEndIndex();
    int graph_start = traj_manager.getGraphStartIndex();
    int graph_end = traj_manager.getGraphEndIndex();

    TrajectoryPoint point = traj_manager.getPoint(gait_start + trajectory_index);

    arduino_serial.sendControlData(
        point.pos1, point.pos2, point.pos3,
        point.velo1, point.velo2, point.velo3,
        point.fc1, point.fc2, point.fc3
    );

    if (graph_start + animation_counter < graph_end) {
        float* gx = traj_manager.getGraphDataX();
        float* gy = traj_manager.getGraphDataY();

        if (gx && gy) {
            data_handler.updateAnimationPoint(
                animation_counter,
                gx[graph_start + animation_counter],
                gy[graph_start + animation_counter]
            );
        }

        animation_counter++;
    }

    if (trajectory_index >= (gait_end - gait_start)) {
        std::cout << "\n=== TRAJECTORY COMPLETE ===\n";

        if (state_machine.hasCycleRemaining()) {
            state_machine.setState(SystemState::POST_REHAB_DELAY);
            state_machine.startDelayTimer();
        } else {
            state_machine.setState(SystemState::IDLE);
            arduino_serial.sendManualCommand('0');
        }
    } 
    else {
        trajectory_index++;
    }
}

void handlePostRehabDelayState() {
    if (state_machine.isDelayTimerExpired(POST_REHAB_DELAY_SEC)) {
        state_machine.incrementCycle();
        trajectory_index = 0;
        animation_counter = 0;
        data_handler.clearAnimationData();

        std::cout << "\n=== CYCLE " << state_machine.getCurrentCycle()
                  << "/" << state_machine.getTargetCycle() << " ===\n";

        state_machine.setState(SystemState::AUTO_REHAB);
    }
}

void handleArduinoFeedback() {
    if (arduino_serial.hasData()) {
        std::string msg = arduino_serial.readLine();

        if (msg.find("RETREAT") != std::string::npos) {
            std::cout << "\n!!! RETREAT TRIGGERED !!!\n";
            state_machine.setRetreatTriggered(true);
        }

        if (msg.find("status:") != std::string::npos) {
            auto status = arduino_serial.parseStatusMessage(msg);
            data_handler.setRealTimeLoadCell(status.load);
        }
    }
}

// ==================== MAIN LOOP ====================

int main() {
    std::cout << "==========================================\n";
    std::cout << "  SISTEM KONTROL REHABILITASI\n";
    std::cout << "  Multi-Trajectory + Cycle Counter\n";
    std::cout << "==========================================\n\n";

    std::cout << "Initializing Modbus Server...\n";
    if (!modbus_server.initialize("0.0.0.0", 5020)) {
        std::cerr << "Failed to initialize Modbus server\n";
        return 1;
    }
    modbus_server.acceptConnection();

    std::cout << "\nLoading trajectory data...\n";
    if (!traj_manager.loadAllTrajectories()) {
        std::cerr << "Failed to load trajectories\n";
        return 1;
    }

    std::cout << "\nConnecting to Arduino...\n";
    if (!arduino_serial.open("/dev/ttyACM0", 115200)) {
        std::cerr << "Warning: Could not connect to Arduino\n";
    }

    data_handler.loadTrajectoryToHMI(1);

    std::cout << "\n=== SYSTEM READY ===\n";
    std::cout << "Waiting for HMI commands...\n";

    auto last_controller_time = std::chrono::steady_clock::now();

    while (true) {

        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc = modbus_server.receiveQuery(query, sizeof(query));

        if (rc > 0) 
            modbus_server.sendReply(query, rc);

        switch (state_machine.getState()) {
            case SystemState::IDLE:
                handleIdleState();
                break;

            case SystemState::AUTO_REHAB:
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - last_controller_time).count()
                        >= CONTROLLER_INTERVAL_MS) {

                    handleAutoRehabState();
                    last_controller_time = std::chrono::steady_clock::now();
                }
                break;

            case SystemState::POST_REHAB_DELAY:
                handlePostRehabDelayState();
                break;

            case SystemState::EMERGENCY_STOP:
                arduino_serial.sendEmergencyStop();
                break;
        }

        handleArduinoFeedback();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    arduino_serial.close();
    modbus_server.closeConnection();
    return 0;
}
