#ifndef GRAPHMANAGER_H
#define GRAPHMANAGER_H

#include <modbus/modbus.h>
#include "ModbusHandler.h"
#include "TrajectoryManager.h"

class GraphManager {
public:
    GraphManager(ModbusHandler& modbus, TrajectoryManager& trajectory);
    
    // Graph management
    void resetGraphData();
    void loadTrajectoryData();
    void clearChannel1Data();
    void updateGraphAnimation(int t_grafik);
    
    // Animation control
    void resetAnimationCounter() { hmi_animation_counter = 0; }
    void setAnimationCounter(int value) { hmi_animation_counter = value; }
    int getAnimationCounter() const { return hmi_animation_counter; }

private:
    ModbusHandler& modbusHandler;
    TrajectoryManager& trajectoryManager;
    int hmi_animation_counter;
};

#endif // GRAPHMANAGER_H

