#include "StateHandlers.h"
#include "../modbus/ModbusServer.h"
#include "../modbus/DataHandler.h"
#include "../trajectory/TrajectoryManager.h"
#include "../serial/SerialPort.h"
#include <iostream>

StateHandlers::StateHandlers(
    ModbusServer* modbus,
    DataHandler* data_handler,
    TrajectoryManager* traj_mgr,
    SerialPort* serial
)
: modbus_server(modbus),
  data_handler(data_handler),
  trajectory_manager(traj_mgr),
  serial_port(serial),
  arduino_retreat_triggered(false)
{
    last_idle_print    = std::chrono::steady_clock::now();
    last_feedback_read = std::chrono::steady_clock::now();
}

//================= STATE HANDLERS =================//

void StateHandlers::handleIdle(
    StateMachine& state_machine,
    int& t_controller,
    int& t_grafik
) {
    // TODO: Implement idle logic
}

void StateHandlers::handleAutoRehab(
    StateMachine& state_machine,
    int& t_controller,
    int& t_grafik,
    std::chrono::steady_clock::time_point& last_controller_time,
    bool& animasi_grafik_berjalan,
    std::string& arduino_feedback_state
) {
    // TODO: Implement rehab logic
}

void StateHandlers::handleAutoRetreat(
    StateMachine& state_machine,
    SerialPort& serial,
    std::string& arduino_feedback_state,
    std::chrono::steady_clock::time_point& last_retreat_time
) {
    // TODO: Implement retreat logic
}

void StateHandlers::handlePostRehabDelay(
    StateMachine& state_machine,
    int& t_controller,
    int& t_grafik,
    bool& animasi_grafik_berjalan,
    std::chrono::steady_clock::time_point& delay_start_time
) {
    // TODO: Implement delay logic
}

void StateHandlers::handleEmergencyStop(
    StateMachine& state_machine,
    SerialPort& serial,
    bool& animasi_grafik_berjalan
) {
    // TODO: Implement emergency stop logic
}

void StateHandlers::handleResetting(
    StateMachine& state_machine,
    SerialPort& serial,
    bool& animasi_grafik_berjalan
) {
    // TODO: Implement resetting logic
}

//================= ARDUINO FEEDBACK =================//

void StateHandlers::readArduinoFeedback() {
    if (!serial_port) return;

    std::string data = serial_port->readLine();

    if (!data.empty()) {
        parseArduinoFeedback(data);
    }
}

void StateHandlers::parseArduinoFeedback(const std::string& feedback) {

    arduino_state = feedback;

    if (feedback == "RETREAT") {
        arduino_retreat_triggered = true;
    }
}

bool StateHandlers::isRetreatTriggered() const {
    return arduino_retreat_triggered;
}

void StateHandlers::clearRetreatTrigger() {
    arduino_retreat_triggered = false;
}
