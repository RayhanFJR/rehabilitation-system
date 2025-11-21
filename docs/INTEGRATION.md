# System Integration Guide - Connect Firmware, Server, Vision

## 📊 System Architecture Overview

```
┌──────────────────────────────────────────────────────────┐
│                    HMI (Windows/Web)                     │
│              Modbus TCP Client (Port 5020)               │
└────────────────────────┬─────────────────────────────────┘
                         │ Modbus TCP
┌────────────────────────┴─────────────────────────────────┐
│         Control Server (Linux/Windows/Mac)               │
│  - Trajectory Manager                                    │
│  - State Machine                                         │
│  - Modbus Server (0.0.0.0:5020)                         │
└────────────┬────────────────────────────────────────────┘
             │ Serial Port (115200)
             │ /dev/ttyACM0 (Linux)
             │ COM3 (Windows)
             │ /dev/tty.usbmodem (Mac)
             │
┌────────────┴────────────────────────────────────────────┐
│            Arduino Microcontroller                       │
│  - Motor Controller                                      │
│  - Sensor Reading (encoders, load cell, current)       │
│  - Load-based Adaptive Control                          │
└────────────┬────────────────────────────────────────────┘
             │ Analog/Digital I/O
             │
        ┌────┴───────────────────────┐
        │   Hardware Actuators        │
        │  - 3 DC Motors + H-bridge   │
        │  - Encoders                 │
        │  - Load Cell (HX711)        │
        │  - Current Sensors          │
        └─────────────────────────────┘

OPTIONAL: Vision System (Separate Process)
┌──────────────────────────────────────────────────────────┐
│              Vision System (Python)                      │
│  - MediaPipe Pose Detection                             │
│  - Foot Angle Calculation                               │
│  - PID Control Output                                   │
│  - Real-time Display                                    │
└──────────────────────────────────────────────────────────┘
```

---

## 🔌 Connection Diagram

### Hardware Connections

```
PC/Laptop ← USB → Arduino

Arduino Pins:
  D3, D5 → Motor 1 PWM (RPWM1, LPWM1)
  D6, D9 → Motor 2 PWM (RPWM2, LPWM2)
  D10, D11 → Motor 3 PWM (RPWM3, LPWM3)
  D4, D2, D8 → Encoders (ENC1, ENC2, ENC3)
  A0, A1, A2 → Current Sensors
  D12, D13 → Load Cell (HX711)
  
  ↓↓↓
  
  Motor Drivers (H-bridge modules) ← PWM signals
  ↓
  DC Motors (3x)
  
  Sensors ← Feedback
  Encoders → Position
  Current Sensors → Current
  Load Cell → Force
```

### Network Connections

```
HMI (Client)
  ↓ Modbus TCP (Port 5020)
  ↓
Server (192.168.x.x or localhost)
  ↓ Serial Port
  ↓
Arduino (via USB)
```

---

## 📋 Step-by-Step Integration

### Step 1: Hardware Setup

```bash
1. Connect Arduino to PC via USB
   → Check COM port (Windows) or /dev/ttyACM0 (Linux)
   
2. Upload firmware
   pio run -t upload
   
3. Verify motor control
   Open serial monitor → Send "1" → Motor should move
   
4. Verify sensors
   Open serial monitor → Check position/load readings
```

### Step 2: Server Setup

```bash
1. Navigate to server folder
   cd server/build

2. Run server
   ./rehab_server
   
   Expected output:
   ✓ Modbus Server listening on port 5020
   ✓ Trajectory 1/2/3 loaded
   ✓ Waiting for HMI connection
   ✓ Arduino connected
```

### Step 3: HMI Connection (Modbus)

```bash
Option A: Use Modbus Master (Windows)
  - Software: QModbus, ModbusTest, etc
  - IP: localhost or 127.0.0.1
  - Port: 5020
  - Connect → Can read registers

Option B: Create Custom HMI
  - Use libmodbus library
  - Connect to server
  - Read/write registers
  
Option C: Web-based HMI
  - Create web interface
  - Use node-modbus or similar
  - Browser access
```

### Step 4: Vision System (Optional)

```bash
1. In separate terminal/process
   cd vision
   python src/main.py
   
2. Calibrate foot angle (press 'c')

3. System outputs angle & PID signal

4. Can integrate with server via network socket
```

---

## 🔧 Communication Protocols

### Arduino ← → Server (Serial)

**Commands from Server to Arduino:**

```
"S1.0,1.0,1.0,0.5,0.5,0.5,0.1,0.1,0.1"
 ↑ Trajectory command
   ↑ Position motor 1, 2, 3
                    ↑ Velocity motor 1, 2, 3
                                    ↑ Force motor 1, 2, 3

"T20,40"
 ↑ Threshold command
   ↑ Threshold1=20N, Threshold2=40N

"X"     = Calibrate
"E"     = Emergency stop
"1"     = Manual forward
"2"     = Manual backward
"0"     = Manual stop
```

**Responses from Arduino to Server:**

```
"status:running,mode:forward,load:25.50,scale:0.95,pos:100.50,105.20,98.70"
 ↑ Status message with:
   - running/paused
   - forward/retreat/manual
   - load value
   - motor positions
   
"RETREAT"
 ↑ Retreat triggered (high load)
```

### HMI ← → Server (Modbus TCP)

**Register Map (Subset):**

```
Address 99:  MANUAL_MAJU (1=forward)
Address 100: MANUAL_STOP (1=stop)
Address 101: MANUAL_MUNDUR (1=backward)
Address 102: CALIBRATE (1=calibrate)
Address 103: START (1=start rehab)
Address 104: EMERGENCY (1=emergency stop)
Address 105: RESET (1=reset)
Address 106: TRAJEKTORI_1 (1=select trajectory 1)
Address 107: TRAJEKTORI_2 (1=select trajectory 2)
Address 108: TRAJEKTORI_3 (1=select trajectory 3)

Address 130: THRESHOLD_1 (load threshold 1)
Address 131: THRESHOLD_2 (load threshold 2)
Address 132: JUMLAH_CYCLE (number of cycles)

Address 121: NUM_OF_DATA_CH0 (trajectory point count)
Address 122: NUM_OF_DATA_CH1 (animation counter)
Address 126: REALTIME_LOAD_CELL (float - load value)

Address 200+: X_DATA_CH0 (trajectory X coordinates)
Address 2000+: Y_DATA_CH0 (trajectory Y coordinates)
Address 4000+: X_DATA_CH1 (animation X points)
Address 6000+: Y_DATA_CH1 (animation Y points)
```

---

## 🎯 Integration Scenarios

### Scenario 1: Basic Manual Control

```
User → HMI (Click "Forward") 
  → Modbus: Write(MANUAL_MAJU, 1)
  → Server: Receives button press
  → Server: sendManualCommand(1) via Serial
  → Arduino: Receives "1"
  → Arduino: Move motors forward
  → Motor feedback via encoders
  → Server receives status via Serial
  → HMI displays position in Modbus registers
```

### Scenario 2: Automated Rehabilitation

```
User → HMI (Select Trajectory 1, Cycles=3, Click "Start")
  → Modbus: Write(TRAJEKTORI_1, 1) + Write(JUMLAH_CYCLE, 3) + Write(START, 1)
  → Server: startRehabCycle(3)
    ├─ Load Trajectory 1
    ├─ State → AUTO_REHAB
    ├─ Loop: For each point in trajectory:
    │  ├─ Send point to Arduino: "S1.0,1.0,1.0,0.5,0.5,0.5,0.1,0.1,0.1"
    │  ├─ Arduino executes motor movement
    │  ├─ Update HMI animation (Modbus registers)
    │  ├─ Update load cell value
    │  └─ If load > threshold2 → RETREAT triggered
    │
    ├─ After cycle complete:
    │  ├─ State → POST_REHAB_DELAY (5 seconds)
    │  ├─ If cycles remaining → back to AUTO_REHAB
    │  └─ Else → back to IDLE
    │
    └─ HMI shows animation in real-time + load cell graph
```

### Scenario 3: Load-Based Retreat

```
During AUTO_REHAB:
  Motor hits high resistance
  ↓
  Arduino Load Cell: detects load > THRESHOLD_2
  ↓
  Arduino: Set manipulatorState=1 (pause)
  ↓
  Arduino: Send "RETREAT" via Serial
  ↓
  Server: Receive "RETREAT"
  ↓
  Server: State → AUTO_RETREAT
  ↓
  Server: Send retreat trajectory (backward) via Serial
  ↓
  Arduino: Execute backward motion with scaled velocity (1.5x)
  ↓
  Arduino: Send "ACK_RETREAT_COMPLETE"
  ↓
  Server: Back to IDLE or POST_REHAB_DELAY
```

---

## 💻 Example: Complete Integration Test

### Python Client Script

```python
#!/usr/bin/env python3
"""
Example: Connect to rehabilitation server and control system
"""

import socket
import struct
import time

class ModbusClient:
    def __init__(self, host='127.0.0.1', port=5020):
        self.host = host
        self.port = port
        self.socket = None
        self.transaction_id = 0
    
    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print(f"[Client] Connected to {self.host}:{self.port}")
    
    def write_register(self, address, value):
        """Write single register"""
        # Simplified Modbus TCP write (actual protocol is more complex)
        # This is pseudo-code - use libmodbus for production
        print(f"[Modbus] Write register {address} = {value}")
    
    def read_register(self, address):
        """Read single register"""
        print(f"[Modbus] Read register {address}")
        return 0  # Pseudo-code
    
    def close(self):
        if self.socket:
            self.socket.close()
        print("[Client] Disconnected")


def main():
    client = ModbusClient(host='127.0.0.1', port=5020)
    client.connect()
    
    try:
        # Test 1: Manual forward
        print("\n[Test 1] Manual Forward...")
        client.write_register(99, 1)  # MANUAL_MAJU
        time.sleep(2)
        client.write_register(100, 1)  # MANUAL_STOP
        
        # Test 2: Start trajectory
        print("\n[Test 2] Start Trajectory 1...")
        client.write_register(106, 1)  # TRAJEKTORI_1
        time.sleep(0.5)
        client.write_register(132, 3)  # JUMLAH_CYCLE = 3
        client.write_register(103, 1)  # START
        
        # Test 3: Monitor load
        print("\n[Test 3] Monitoring for 10 seconds...")
        for i in range(10):
            load = client.read_register(126)  # REALTIME_LOAD_CELL
            animation_count = client.read_register(122)
            print(f"  Load: {load:.2f}N, Animation: {animation_count}")
            time.sleep(1)
        
        # Test 4: Emergency stop
        print("\n[Test 4] Emergency Stop...")
        client.write_register(104, 1)  # EMERGENCY
        
    finally:
        client.close()


if __name__ == "__main__":
    main()
```

### Actual Modbus Test (using pymodbus)

```bash
# Install library
pip install pymodbus

# Create test script
cat > test_integration.py << 'EOF'
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient(host='127.0.0.1', port=5020)
client.connect()

# Test: Read trajectory point count
result = client.read_holding_registers(121, 1)
print(f"Trajectory points: {result.registers[0]}")

# Test: Start manual forward
client.write_register(99, 1)
print("Manual forward sent")

# Test: Read load cell
result = client.read_holding_registers(126, 2)
load = result.registers  # Float value
print(f"Load: {load}")

client.close()
EOF

python test_integration.py
```

---

## 🚀 Startup Sequence

### Complete System Startup (Correct Order)

```bash
Terminal 1: Arduino Serial Monitor (Optional - for debugging)
$ pio device monitor -p /dev/ttyACM0 -b 115200

Terminal 2: Upload Firmware (if changed)
$ cd firmware
$ pio run -t upload

Terminal 3: Start Server
$ cd server/build
$ ./rehab_server

Terminal 4: Start Vision (Optional)
$ cd vision
$ python src/main.py

Terminal 5: Connect HMI
$ # Open Modbus client software
$ # Connect to localhost:5020

Expected Sequence:
1. Arduino boots → Serial output shows initialization
2. Server starts → "Waiting for HMI connection"
3. Vision starts (optional) → Camera displays
4. HMI connects → Server shows "HMI connected"
5. You can now control the system!
```

### Verify Each Component

```bash
# 1. Check Arduino connection
ls -la /dev/ttyACM*     # Linux
COM port in Device Manager  # Windows
ls -la /dev/tty.usb*    # Mac

# 2. Test serial communication
pio device monitor -b 115200

# 3. Test server startup
./rehab_server
# Should see: "Waiting for HMI connection..."

# 4. Test Modbus connection
python3 << 'EOF'
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
result = s.connect_ex(('127.0.0.1', 5020))
if result == 0:
    print("Server is listening on port 5020")
else:
    print("Server is NOT listening")
s.close()
EOF

# 5. Test motor response
# Via serial: send "1" (manual forward)
# Motor should move
```

---

## 📊 Data Flow During Operation

### Timeline: Normal Operation (5 seconds)

```
t=0s: User clicks "START" in HMI
├─ HMI → Modbus: Write(START, 1)
├─ Server receives START command
├─ Server: Load trajectory, State=AUTO_REHAB
└─ Server: Send first trajectory point to Arduino

t=0.1s: Server sends trajectory point #1
├─ Server → Arduino: "S1.0,1.0,1.0,0.5,0.5,0.5,0.1,0.1,0.1"
├─ Arduino: Parse command
├─ Arduino: Execute motor control (PD + CTC)
├─ Motors move to position
└─ Server updates HMI with animation point

t=0.2s: Arduino sends status
├─ Arduino → Server: "status:running,mode:forward,load:5.2,pos:1.2,1.3,1.1"
├─ Server parses status
├─ Server: Write load to Modbus register 126
└─ HMI reads load from Modbus → Display updates

t=0.2-5.0s: Repeat every 100ms
├─ Send trajectory point
├─ Arduino moves
├─ Receive status
├─ Update HMI
└─ Check for retreat

t=5.0s+: After trajectory complete
├─ Server: State = POST_REHAB_DELAY
├─ Wait 5 seconds
├─ If cycles remaining:
│  └─ State = AUTO_REHAB (next cycle)
└─ Else:
   └─ State = IDLE
```

---

## ✅ Integration Checklist

- [ ] Arduino firmware uploaded successfully
- [ ] Serial communication working (pio device monitor shows output)
- [ ] Server compiles without errors
- [ ] Server can connect to Arduino (message "Arduino connected")
- [ ] Server listening on port 5020 (can telnet localhost 5020)
- [ ] Modbus client can connect to server
- [ ] Can read trajectory point count from Modbus
- [ ] Can write MANUAL_MAJU → Motors move forward
- [ ] Can read load cell value from Modbus
- [ ] Can start trajectory → Animation updates in real-time
- [ ] Load-based retreat works (high load triggers retreat)
- [ ] Vision system detects pose (if enabled)
- [ ] System handles emergency stop (EMERGENCY button)

---

## 🐛 Troubleshooting Integration

### Problem: "Arduino not detected"

**Solution:**
```bash
# Linux
ls -la /dev/ttyACM*

# Update platformio.ini
upload_port = /dev/ttyACM0

# Or auto-detect
pio run -t upload  # Auto-finds port
```

### Problem: "Server can't connect to Arduino"

**Solution:**
```bash
# 1. Check serial port
pio device list

# 2. Update main_server.cpp
serial_port.open("/dev/ttyACM0");  // or COM3

# 3. Rebuild server
cd server/build
cmake .. && make
```

### Problem: "Modbus client can't connect"

**Solution:**
```bash
# 1. Check if server is running
ps aux | grep rehab_server

# 2. Check if port 5020 is listening
netstat -tlnp | grep 5020
lsof -i :5020

# 3. Firewall issue?
sudo ufw allow 5020
```

### Problem: "Motors don't move"

**Solution:**
```bash
# 1. Check Arduino serial output
pio device monitor

# 2. Send test command manually
# Via serial monitor: send "1"
# Motor should move

# 3. Check PWM pins connected
# D3, D5, D6, D9, D10, D11

# 4. Check H-bridge connections
```

### Problem: "Trajectory not executing"

**Solution:**
```bash
# 1. Check data files loaded
./rehab_server
# Should show: "Loaded Trajectory 1: 816 points"

# 2. Check HMI sends START command
# Add debug output in server

# 3. Check state machine
# Should go: IDLE → AUTO_REHAB
```

---

## 📈 Performance Monitoring

### Real-time Metrics

```bash
# Monitor Arduino
watch -n 0.1 'tail -1 /dev/ttyACM0'

# Monitor server
# Check console output every 100ms

# Monitor network
wireshark -i lo  # Capture Modbus packets

# System load
top
# Check CPU usage (should be <20%)
```

### Logging for Analysis

```bash
# Server debug output
./rehab_server 2>&1 | tee server.log

# Vision output
python src/main.py 2>&1 | tee vision.log

# Analyze later
tail -f server.log | grep "RETREAT"
grep "error" vision.log
```

---

## 🎉 Congratulations!

When all components communicate successfully:

✅ **Firmware** ↔️ **Server** ↔️ **HMI**
✅ **Real-time** Motor control
✅ **Load-based** Retreat
✅ **Cycle** Management
✅ **Animation** in HMI

**System is fully integrated! 🚀**

Next: Testing & Validation