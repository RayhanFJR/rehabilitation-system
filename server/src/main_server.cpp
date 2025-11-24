//==================================================================
// FILE: server/src/main_server.cpp
// Updated main loop with complete state handling
//==================================================================

#include <iostream>
#include <thread>
#include <chrono>
#include <modbus/modbus.h>

#include "config/config.h"
#include "trajectory/TrajectoryManager.h"
#include "modbus/ModbusServer.h"
#include "serial/SerialPort.h"
#include "state_machine/StateMachine.h"

using boost::asio::io_context;
using namespace std;

int main() {
    // Load trajectory data
    TrajectoryData traj_data;
    if (!traj_data.loadAllTrajectories()) {
        cerr << "Fatal Error: Cannot load trajectory data. Exiting..." << endl;
        return 1;
    }
    
    // Set default trajectory
    traj_data.switchTrajectory(1);
    
    // Initialize serial communication
    boost::asio::io_context io;
    SerialCommunication serial(io);
    if (!serial.initialize(SerialConfig::DEVICE_PATH, SerialConfig::BAUD_RATE)) {
        cerr << "Fatal Error: Cannot initialize serial. Exiting..." << endl;
        return 1;
    }
    serial.sendCommand("0"); // Stop motors initially
    
    // Initialize Modbus handler
    ModbusHandler modbus;
    if (!modbus.initialize(ModbusConfig::IP_ADDRESS, 
                          ModbusConfig::PORT, 
                          ModbusConfig::SLAVE_ID)) {
        cerr << "Fatal Error: Cannot initialize Modbus. Exiting..." << endl;
        return 1;
    }
    
    if (!modbus.acceptConnection()) {
        cerr << "Fatal Error: Cannot accept Modbus connection. Exiting..." << endl;
        return 1;
    }
    
    // Create system controller
    SystemController controller(traj_data, serial, modbus);
    
    // Run main control loop
    controller.run();
    
    // Cleanup
    serial.close();
    modbus.close();
    
    return 0;
}