//==================================================================
// ARDUINO MOTOR CONTROL SYSTEM - FULL INTEGRATED VERSION
// Features:
// - Adaptive CTC Control (Model-based parameter estimation)
// - Load-based adaptive scaling
// - Trajectory-based retreat communication
// - Manual/Auto modes with safety system
//==================================================================

//==================================================================
// CONSTANTS & CONFIGURATION
//==================================================================

// Motor Control Speed
const int MANUAL_SPEED = 125;
const int RETREAT_SPEED = 150;

// Load Cell Configuration
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;
int threshold1 = 20;
int threshold2 = 40;
long loadCellOffset = 0;
float latestValidLoad = 0.0;

// Adaptive Load Control Parameters
const float LOAD_SCALE_MIN = 0.15;
const float LOAD_SCALE_MAX = 1.0;
const float KP_DAMPING_FACTOR = 0.6;
const bool ENABLE_ADAPTIVE_MANUAL = true;
float smoothedLoad = 0.0;
const float LOAD_ALPHA = 0.3;

// Retreat Control Parameters
const float RETREAT_VELOCITY_SCALE = 1.5;
bool retreatRequestSent = false;

// Motor Control Parameters
const float GR = 0.2786;
const float kt = 0.0663;

// Current Sensor Parameters
const int adcMax = 1023;
const int nSamples = 3;
const float CalFac = 3.40;

// Timing Intervals (milliseconds)
const int loadCellInterval = 100;
const int encoderInterval = 1;
const int veloInterval = 10000;
const int CTCcalculationInterval = 100;
const int PDcalculationInterval = 100;
const int adaptiveUpdateInterval = 100;

//==================================================================
// PIN DEFINITIONS
//==================================================================

const int RPWM1 = 3, LPWM1 = 5;
const int RPWM2 = 6, LPWM2 = 9;
const int RPWM3 = 10, LPWM3 = 11;

const int ENC1 = 4;
const int ENC2 = 2;
const int ENC3 = 8;

const int CurrSen1 = A0;
const int CurrSen2 = A1;
const int CurrSen3 = A2;

//==================================================================
// SYSTEM STATE VARIABLES
//==================================================================

int operatingMode = 0;  // 0=Manual, 1=Auto Forward, 2=Auto Retreat
int manualCommand = 0;
int manipulatorState = 0;
bool retreatHasBeenTriggered = false;

String receivedData = "";

//==================================================================
// ADAPTIVE CONTROL PARAMETERS
//==================================================================

const int NUM_PARAMS = 9;

// Parameter estimates: [m_platform, m_cylinder, m_piston, b1, b2, b3, I_platform, I_piston, I_sleeve]
float P_hat[NUM_PARAMS] = {
    0.330871,  // m_platform (Kg)
    0.42286,   // m_cylinder (Kg) 
    0.030890,  // m_piston (Kg)
    0.1,       // b1 - friction motor 1
    0.1,       // b2 - friction motor 2
    0.1,       // b3 - friction motor 3
    0.05,      // I_platform
    0.02,      // I_piston
    0.03       // I_sleeve
};

float P_initial[NUM_PARAMS] = {
    0.330871, 0.42286, 0.030890,
    0.1, 0.1, 0.1,
    0.05, 0.02, 0.03
};

// Adaptive gains
float Gamma[NUM_PARAMS] = {
    0.5, 0.5, 0.5,  // Mass parameters
    1.0, 1.0, 1.0,  // Friction parameters (faster adaptation)
    0.3, 0.3, 0.3   // Inertia parameters
};

float Lambda[3] = {5.0, 5.0, 5.0};

// Control variables
float s[3] = {0.0, 0.0, 0.0};
float Y[3][NUM_PARAMS];
bool adaptiveControlEnabled = true;
const float ADAPTIVE_DT = 0.1;

//==================================================================
// MOTOR 1 VARIABLES
//==================================================================

float kp1 = 110.0, kd1 = 0.1;
float kpc1 = 30.0, kdc1 = 0.1;

float refPos1 = 0.0, refVelo1 = 0.0, refFc1 = 0.0, refCurrent1 = 0.0;
float refAccel1 = 0.0, prevRefVelo1 = 0.0;

float ActPos1 = 0.0, ActVelo1 = 0.0, ActCurrent1 = 0.0;
int position1 = 0, prevState1 = 0;

float controlValue1 = 0.0, ErrPos1 = 0.0, error1 = 0.0;
float prevError1 = 0.0, prevPos1 = 0.0;

//==================================================================
// MOTOR 2 VARIABLES
//==================================================================

float kp2 = 142.0, kd2 = 0.6;
float kpc2 = 33.0, kdc2 = 0.1;

float refPos2 = 0.0, refVelo2 = 0.0, refFc2 = 0.0, refCurrent2 = 0.0;
float refAccel2 = 0.0, prevRefVelo2 = 0.0;

float ActPos2 = 0.0, ActVelo2 = 0.0, ActCurrent2 = 0.0;
int position2 = 0, prevState2 = 0;

float controlValue2 = 0.0, ErrPos2 = 0.0, error2 = 0.0;
float prevError2 = 0.0, prevPos2 = 0.0;

//==================================================================
// MOTOR 3 VARIABLES
//==================================================================

float kp3 = 150.0, kd3 = 0.3;
float kpc3 = 38.0, kdc3 = 0.1;

float refPos3 = 0.0, refVelo3 = 0.0, refFc3 = 0.0, refCurrent3 = 0.0;
float refAccel3 = 0.0, prevRefVelo3 = 0.0;

float ActPos3 = 0.0, ActVelo3 = 0.0, ActCurrent3 = 0.0;
int position3 = 0, prevState3 = 0;

float controlValue3 = 0.0, ErrPos3 = 0.0, error3 = 0.0;
float prevError3 = 0.0, prevPos3 = 0.0;

//==================================================================
// TIMING VARIABLES
//==================================================================

long lastLoadTime = 0;
long lastEncTime = 0;
long lastVeloTime = 0;
long lastCTCCalcTime = 0;
long lastPDCalcTime = 0;
long lastPrnTime = 0;
long lastAdaptiveTime = 0;

//==================================================================
// ADAPTIVE CONTROL FUNCTIONS
//==================================================================

void constrainParameters() {
    P_hat[0] = constrain(P_hat[0], 0.1, 1.0);
    P_hat[1] = constrain(P_hat[1], 0.1, 1.0);
    P_hat[2] = constrain(P_hat[2], 0.01, 0.5);
    P_hat[3] = constrain(P_hat[3], 0.01, 0.5);
    P_hat[4] = constrain(P_hat[4], 0.01, 0.5);
    P_hat[5] = constrain(P_hat[5], 0.01, 0.5);
    P_hat[6] = constrain(P_hat[6], 0.01, 0.2);
    P_hat[7] = constrain(P_hat[7], 0.01, 0.1);
    P_hat[8] = constrain(P_hat[8], 0.01, 0.15);
}

void resetAdaptiveParameters() {
    for (int i = 0; i < NUM_PARAMS; i++) {
        P_hat[i] = P_initial[i];
    }
}

void computeRegressorMatrix() {
    for (int motor = 0; motor < 3; motor++) {
        float accel, vel, pos;
        
        if (motor == 0) {
            accel = refAccel1;
            vel = ActVelo1;
            pos = ActPos1;
        } else if (motor == 1) {
            accel = refAccel2;
            vel = ActVelo2;
            pos = ActPos2;
        } else {
            accel = refAccel3;
            vel = ActVelo3;
            pos = ActPos3;
        }
        
        float mass_factor = accel / 3.0;
        Y[motor][0] = mass_factor;
        Y[motor][1] = mass_factor;
        Y[motor][2] = mass_factor;
        
        Y[motor][3] = (motor == 0) ? vel : 0.0;
        Y[motor][4] = (motor == 1) ? vel : 0.0;
        Y[motor][5] = (motor == 2) ? vel : 0.0;
        
        float inertia_factor = accel * 0.01;
        Y[motor][6] = inertia_factor;
        Y[motor][7] = inertia_factor;
        Y[motor][8] = inertia_factor;
    }
}

void updateAdaptiveLaw() {
    float ErrVelo1 = Error(refVelo1, ActVelo1);
    float ErrVelo2 = Error(refVelo2, ActVelo2);
    float ErrVelo3 = Error(refVelo3, ActVelo3);
    
    s[0] = ErrVelo1 + Lambda[0] * ErrPos1;
    s[1] = ErrVelo2 + Lambda[1] * ErrPos2;
    s[2] = ErrVelo3 + Lambda[2] * ErrPos3;
    
    for (int i = 0; i < NUM_PARAMS; i++) {
        float update = 0.0;
        for (int j = 0; j < 3; j++) {
            update += Y[j][i] * s[j];
        }
        P_hat[i] += -Gamma[i] * update * ADAPTIVE_DT;
    }
    
    constrainParameters();
}

//==================================================================
// LOAD-BASED ADAPTIVE FUNCTIONS
//==================================================================

float getLoadScaling() {
    smoothedLoad = (LOAD_ALPHA * latestValidLoad) + ((1.0 - LOAD_ALPHA) * smoothedLoad);
    
    if (smoothedLoad < threshold1) {
        return LOAD_SCALE_MAX;
    } else if (smoothedLoad >= threshold2) {
        return LOAD_SCALE_MIN;
    } else {
        float loadRatio = (smoothedLoad - threshold1) / (float)(threshold2 - threshold1);
        return LOAD_SCALE_MAX - (loadRatio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN));
    }
}

float getAdaptiveKp(float baseKp) {
    if (smoothedLoad < threshold1) {
        return baseKp;
    } else if (smoothedLoad >= threshold2) {
        return baseKp * (1.0 - KP_DAMPING_FACTOR);
    } else {
        float loadRatio = (smoothedLoad - threshold1) / (float)(threshold2 - threshold1);
        float damping = 1.0 - (loadRatio * KP_DAMPING_FACTOR);
        return baseKp * damping;
    }
}

//==================================================================
// CONTROL FUNCTIONS
//==================================================================

float Error(float ref, float act) {
    return ref - act;
}

float CTC(float PosError, float kp, float VeloError, float kd, float ForceID, float GR, float kt) {
    return ((PosError * kp) + (VeloError * kd) + ForceID) * GR * kt;
}

float PD(float error, float kpc, float errordev, float kdc) {
    return (error * kpc) + (errordev * kdc);
}

//==================================================================
// SENSOR READING FUNCTIONS
//==================================================================

long readHX711() {
    long result = 0;
    while (digitalRead(LOADCELL_DOUT_PIN));
    
    for (int i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        delayMicroseconds(1);
        result = result << 1;
        if (digitalRead(LOADCELL_DOUT_PIN)) {
            result++;
        }
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(1);
    
    if (result & 0x800000) {
        result |= ~0xFFFFFF;
    }
    
    return result;
}

float avg1() {
    float val1 = 0;
    for (int i = 0; i < nSamples; i++) {
        val1 += analogRead(CurrSen1);
        delay(1);
    }
    return val1 / adcMax / nSamples;
}

float avg2() {
    float val2 = 0;
    for (int i = 0; i < nSamples; i++) {
        val2 += analogRead(CurrSen2);
        delay(1);
    }
    return val2 / adcMax / nSamples;
}

float avg3() {
    float val3 = 0;
    for (int i = 0; i < nSamples; i++) {
        val3 += analogRead(CurrSen3);
        delay(1);
    }
    return val3 / adcMax / nSamples;
}

//==================================================================
// COMMAND PARSING FUNCTIONS
//==================================================================

void parseTrajectoryCommand(String data, bool isRetreat) {
    data.replace("S", "");
    data.replace("R", "");
    
    int commaIndex1 = data.indexOf(',');
    String refPos1Str = data.substring(0, commaIndex1);
    data = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = data.indexOf(',');
    String refPos2Str = data.substring(0, commaIndex2);
    data = data.substring(commaIndex2 + 1);
    
    int commaIndex3 = data.indexOf(',');
    String refPos3Str = data.substring(0, commaIndex3);
    data = data.substring(commaIndex3 + 1);
    
    int commaIndex4 = data.indexOf(',');
    String refVelo1Str = data.substring(0, commaIndex4);
    data = data.substring(commaIndex4 + 1);
    
    int commaIndex5 = data.indexOf(',');
    String refVelo2Str = data.substring(0, commaIndex5);
    data = data.substring(commaIndex5 + 1);
    
    int commaIndex6 = data.indexOf(',');
    String refVelo3Str = data.substring(0, commaIndex6);
    data = data.substring(commaIndex6 + 1);
    
    int commaIndex7 = data.indexOf(',');
    String refFc1Str = data.substring(0, commaIndex7);
    data = data.substring(commaIndex7 + 1);
    
    int commaIndex8 = data.indexOf(',');
    String refFc2Str = data.substring(0, commaIndex8);
    String refFc3Str = data.substring(commaIndex8 + 1);
    
    refPos1 = refPos1Str.toFloat();
    refPos2 = refPos2Str.toFloat();
    refPos3 = refPos3Str.toFloat();
    refVelo1 = refVelo1Str.toFloat();
    refVelo2 = refVelo2Str.toFloat();
    refVelo3 = refVelo3Str.toFloat();
    refFc1 = refFc1Str.toFloat();
    refFc2 = refFc2Str.toFloat();
    refFc3 = refFc3Str.toFloat();
    
    if (isRetreat) {
        refVelo1 *= RETREAT_VELOCITY_SCALE;
        refVelo2 *= RETREAT_VELOCITY_SCALE;
        refVelo3 *= RETREAT_VELOCITY_SCALE;
    }
}

void parseOuterLoopGains(String data) {
    data.replace("K", "");
    
    int commaIndex1 = data.indexOf(',');
    String motorNumStr = data.substring(0, commaIndex1);
    data = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = data.indexOf(',');
    String kpStr = data.substring(0, commaIndex2);
    String kdStr = data.substring(commaIndex2 + 1);
    
    int motorNum = motorNumStr.toInt();
    float newKp = kpStr.toFloat();
    float newKd = kdStr.toFloat();
    
    if (motorNum == 1) {
        kp1 = newKp; kd1 = newKd;
        Serial.print("Motor 1 Outer Loop: Kp=");
        Serial.print(kp1); Serial.print(", Kd="); Serial.println(kd1);
    } else if (motorNum == 2) {
        kp2 = newKp; kd2 = newKd;
        Serial.print("Motor 2 Outer Loop: Kp=");
        Serial.print(kp2); Serial.print(", Kd="); Serial.println(kd2);
    } else if (motorNum == 3) {
        kp3 = newKp; kd3 = newKd;
        Serial.print("Motor 3 Outer Loop: Kp=");
        Serial.print(kp3); Serial.print(", Kd="); Serial.println(kd3);
    }
    Serial.println("");
}

void parseInnerLoopGains(String data) {
    data.replace("P", "");
    
    int commaIndex1 = data.indexOf(',');
    String motorNumStr = data.substring(0, commaIndex1);
    data = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = data.indexOf(',');
    String kpcStr = data.substring(0, commaIndex2);
    String kdcStr = data.substring(commaIndex2 + 1);
    
    int motorNum = motorNumStr.toInt();
    float newKpc = kpcStr.toFloat();
    float newKdc = kdcStr.toFloat();
    
    if (motorNum == 1) {
        kpc1 = newKpc; kdc1 = newKdc;
        Serial.print("Motor 1 Inner Loop: Kpc=");
        Serial.print(kpc1); Serial.print(", Kdc="); Serial.println(kdc1);
    } else if (motorNum == 2) {
        kpc2 = newKpc; kdc2 = newKdc;
        Serial.print("Motor 2 Inner Loop: Kpc=");
        Serial.print(kpc2); Serial.print(", Kdc="); Serial.println(kdc2);
    } else if (motorNum == 3) {
        kpc3 = newKpc; kdc3 = newKdc;
        Serial.print("Motor 3 Inner Loop: Kpc=");
        Serial.print(kpc3); Serial.print(", Kdc="); Serial.println(kdc3);
    }
    Serial.println("");
}

void parseAdaptiveGains(String data) {
    bool isGamma = data.startsWith("A");
    data.replace("A", "");
    data.replace("L", "");
    
    int commaIndex = data.indexOf(',');
    String indexStr = data.substring(0, commaIndex);
    String valueStr = data.substring(commaIndex + 1);
    
    int index = indexStr.toInt();
    float value = valueStr.toFloat();
    
    if (isGamma && index >= 0 && index < NUM_PARAMS) {
        Gamma[index] = value;
        Serial.print("Gamma["); Serial.print(index);
        Serial.print("] = "); Serial.println(value);
    } else if (!isGamma && index >= 0 && index < 3) {
        Lambda[index] = value;
        Serial.print("Lambda["); Serial.print(index);
        Serial.print("] = "); Serial.println(value);
    }
}

void parseThresholds(String data) {
    data.replace("T", "");
    
    int commaIndex = data.indexOf(',');
    String t1Str = data.substring(0, commaIndex);
    String t2Str = data.substring(commaIndex + 1);
    
    threshold1 = t1Str.toInt();
    threshold2 = t2Str.toInt();
    
    Serial.print("Thresholds: T1=");
    Serial.print(threshold1);
    Serial.print(", T2=");
    Serial.println(threshold2);
    Serial.println("");
}

void resetSystem() {
    operatingMode = 0;
    manualCommand = 0;
    manipulatorState = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent = false;
    
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
    
    loadCellOffset = readHX711();
    smoothedLoad = 0.0;
    
    position1 = 0; position2 = 0; position3 = 0;
    ActPos1 = 0.0; ActPos2 = 0.0; ActPos3 = 0.0;
    prevPos1 = 0.0; prevPos2 = 0.0; prevPos3 = 0.0;
    ErrPos1 = 0.0; ErrPos2 = 0.0; ErrPos3 = 0.0;
    error1 = 0.0; error2 = 0.0; error3 = 0.0;
    prevError1 = 0.0; prevError2 = 0.0; prevError3 = 0.0;
    ActVelo1 = 0.0; ActVelo2 = 0.0; ActVelo3 = 0.0;
    
    Serial.println("System Reset: Tare & Zero OK\n");
}

void emergencyStop() {
    operatingMode = 0;
    manualCommand = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent = false;
    
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
    
    Serial.println("EMERGENCY_STOP");
}

//==================================================================
// MOTOR CONTROL FUNCTIONS
//==================================================================

void updateEncoders() {
    int currentState1 = digitalRead(ENC1);
    if (currentState1 > prevState1) {
        if (controlValue1 > 0) position1++;
        else if (controlValue1 < 0) position1--;
    }
    ActPos1 = position1 * 0.245;
    prevState1 = currentState1;
    
    int currentState2 = digitalRead(ENC2);
    if (currentState2 > prevState2) {
        if (controlValue2 > 0) position2++;
        else if (controlValue2 < 0) position2--;
    }
    ActPos2 = position2 * 0.245;
    prevState2 = currentState2;
    
    int currentState3 = digitalRead(ENC3);
    if (currentState3 > prevState3) {
        if (controlValue3 > 0) position3++;
        else if (controlValue3 < 0) position3--;
    }
    ActPos3 = position3 * 0.245;
    prevState3 = currentState3;
}

void updateVelocities() {
    ActVelo1 = (ActPos1 - prevPos1) / 10;
    ActVelo2 = (ActPos2 - prevPos2) / 10;
    ActVelo3 = (ActPos3 - prevPos3) / 10;
    prevPos1 = ActPos1;
    prevPos2 = ActPos2;
    prevPos3 = ActPos3;
}

void calculateCTC() {
    ErrPos1 = Error(refPos1, ActPos1);
    ErrPos2 = Error(refPos2, ActPos2);
    ErrPos3 = Error(refPos3, ActPos3);
    
    float ErrVelo1 = Error(refVelo1, ActVelo1);
    float ErrVelo2 = Error(refVelo2, ActVelo2);
    float ErrVelo3 = Error(refVelo3, ActVelo3);
    
    float kp1_adaptive = getAdaptiveKp(kp1);
    float kp2_adaptive = getAdaptiveKp(kp2);
    float kp3_adaptive = getAdaptiveKp(kp3);
    
    refCurrent1 = CTC(ErrPos1, kp1_adaptive, ErrVelo1, kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, kp2_adaptive, ErrVelo2, kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, kp3_adaptive, ErrVelo3, kd3, refFc3, GR, kt);
}

void calculateAdaptiveCTC() {
    // Calculate reference acceleration
    refAccel1 = (refVelo1 - prevRefVelo1) / ADAPTIVE_DT;
    refAccel2 = (refVelo2 - prevRefVelo2) / ADAPTIVE_DT;
    refAccel3 = (refVelo3 - prevRefVelo3) / ADAPTIVE_DT;
    
    prevRefVelo1 = refVelo1;
    prevRefVelo2 = refVelo2;
    prevRefVelo3 = refVelo3;
    
    // Compute errors
    ErrPos1 = Error(refPos1, ActPos1);
    ErrPos2 = Error(refPos2, ActPos2);
    ErrPos3 = Error(refPos3, ActPos3);
    
    float ErrVelo1 = Error(refVelo1, ActVelo1);
    float ErrVelo2 = Error(refVelo2, ActVelo2);
    float ErrVelo3 = Error(refVelo3, ActVelo3);
    
    // Compute regressor matrix
    computeRegressorMatrix();
    
    // Compute adaptive torque: τ = Y * P_hat - Kd*ė - Kp*e
    float tau1 = 0.0, tau2 = 0.0, tau3 = 0.0;
    
    for (int j = 0; j < NUM_PARAMS; j++) {
        tau1 += Y[0][j] * P_hat[j];
        tau2 += Y[1][j] * P_hat[j];
        tau3 += Y[2][j] * P_hat[j];
    }
    
    // Apply load-based adaptive gains
    float kp1_adaptive = getAdaptiveKp(kp1);
    float kp2_adaptive = getAdaptiveKp(kp2);
    float kp3_adaptive = getAdaptiveKp(kp3);
    
    // Add PD feedback
    tau1 -= (kd1 * ErrVelo1 + kp1_adaptive * ErrPos1);
    tau2 -= (kd2 * ErrVelo2 + kp2_adaptive * ErrPos2);
    tau3 -= (kd3 * ErrVelo3 + kp3_adaptive * ErrPos3);
    
    // Add feedforward
    tau1 += refFc1;
    tau2 += refFc2;
    tau3 += refFc3;
    
    // Convert to current
    refCurrent1 = tau1 * GR * kt;
    refCurrent2 = tau2 * GR * kt;
    refCurrent3 = tau3 * GR * kt;
    
    // Update adaptive parameters
    if (adaptiveControlEnabled) {
        updateAdaptiveLaw();
    }
}

void calculatePD() {
    error1 = Error(refCurrent1, ActCurrent1);
    error2 = Error(refCurrent2, ActCurrent2);
    error3 = Error(refCurrent3, ActCurrent3);
    
    float DerError1 = (error1 - prevError1) / 0.1;
    float DerError2 = (error2 - prevError2) / 0.1;
    float DerError3 = (error3 - prevError3) / 0.1;
    
    controlValue1 = PD(error1, kpc1, DerError1, kdc1);
    controlValue2 = PD(error2, kpc2, DerError2, kdc2);
    controlValue3 = PD(error3, kpc3, DerError3, kdc3);
    
    prevError1 = error1;
    prevError2 = error2;
    prevError3 = error3;
    
    // Apply load-based speed scaling (only in forward mode)
    if (operatingMode == 1) {
        float loadScale = getLoadScaling();
        controlValue1 = constrain(controlValue1 * loadScale, -255, 255);
        controlValue2 = constrain(controlValue2 * loadScale, -255, 255);
        controlValue3 = constrain(controlValue3 * loadScale, -255, 255);
    } else {
        controlValue1 = constrain(controlValue1, -255, 255);
        controlValue2 = constrain(controlValue2, -255, 255);
        controlValue3 = constrain(controlValue3, -255, 255);
    }
}

void applyMotorControl() {
    // Motor 1
    if (ErrPos1 > 0) {
        analogWrite(RPWM1, abs(controlValue1));
        analogWrite(LPWM1, 0);
    } else if (ErrPos1 < 0) {
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, abs(controlValue1));
    } else {
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, 0);
    }
    
    // Motor 2
    if (ErrPos2 > 0) {
        analogWrite(RPWM2, abs(controlValue2));
        analogWrite(LPWM2, 0);
    } else if (ErrPos2 < 0) {
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, abs(controlValue2));
    } else {
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, 0);
    }
    
    // Motor 3
    if (ErrPos3 > 0) {
        analogWrite(RPWM3, abs(controlValue3));
        analogWrite(LPWM3, 0);
    } else if (ErrPos3 < 0) {
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, abs(controlValue3));
    } else {
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, 0);
    }
}

void stopAllMotors() {
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
}

void manualModeControl() {
    int effectiveSpeed = MANUAL_SPEED;
    
    if (ENABLE_ADAPTIVE_MANUAL) {
        float loadScale = getLoadScaling();
        effectiveSpeed = MANUAL_SPEED * loadScale;
    }
    
    if (manualCommand == 1) {
        analogWrite(RPWM1, effectiveSpeed); analogWrite(LPWM1, 0);
        analogWrite(RPWM2, effectiveSpeed); analogWrite(LPWM2, 0);
        analogWrite(RPWM3, effectiveSpeed); analogWrite(LPWM3, 0);
    } else if (manualCommand == 2) {
        analogWrite(RPWM1, 0); analogWrite(LPWM1, effectiveSpeed);
        analogWrite(RPWM2, 0); analogWrite(LPWM2, effectiveSpeed);
        analogWrite(RPWM3, 0); analogWrite(LPWM3, effectiveSpeed);
    } else {
        stopAllMotors();
    }
}

//==================================================================
// SETUP
//==================================================================

void setup() {
    Serial.begin(115200);
    
    // Motor PWM pins
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);
    pinMode(RPWM3, OUTPUT);
    pinMode(LPWM3, OUTPUT);
    
    // Encoder pins
    pinMode(ENC1, INPUT);
    pinMode(ENC2, INPUT);
    pinMode(ENC3, INPUT);
    
    // Current sensor pins
    pinMode(CurrSen1, INPUT);
    pinMode(CurrSen2, INPUT);
    pinMode(CurrSen3, INPUT);
    
    // Load cell pins
    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN, OUTPUT);
    
    while (!Serial) {
        ;
    }
    
    Serial.println("===========================================");
    Serial.println("  3-RPS Parallel Robot Control System");
    Serial.println("  Adaptive CTC + Load-based Control");
    Serial.println("  Trajectory Retreat System");
    Serial.println("===========================================");
    Serial.println("");
}

//==================================================================
// MAIN LOOP
//==================================================================

void loop() {
    long currentTime = millis();
    
    //================================================================
    // SERIAL COMMAND PROCESSING
    //================================================================
    
    if (Serial.available() > 0) {
        char receivedChar = Serial.read();
        receivedData += receivedChar;
        
        if (receivedChar == '\n') {
            receivedData.trim();
            
            // Forward trajectory command (S)
            if (receivedData.startsWith("S")) {
                operatingMode = 1;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
                manipulatorState = 0;
                parseTrajectoryCommand(receivedData, false);
            }
            // Retreat trajectory command (R with positions)
            else if (receivedData.startsWith("R") && receivedData.indexOf(',') > 0) {
                operatingMode = 2;
                manipulatorState = 0;
                parseTrajectoryCommand(receivedData, true);
            }
            // Retreat finished notification
            else if (receivedData == "RETREAT_COMPLETE") {
                operatingMode = 0;
                manualCommand = 0;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
                stopAllMotors();
                Serial.println("ACK_RETREAT_COMPLETE");
            }
            // Tare and reset (X)
            else if (receivedData.startsWith("X")) {
                resetSystem();
            }
            // Emergency stop (E)
            else if (receivedData.startsWith("E")) {
                emergencyStop();
            }
            // Manual forward (1)
            else if (receivedData == "1") {
                operatingMode = 0;
                manualCommand = 1;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
            }
            // Manual backward (2)
            else if (receivedData == "2") {
                operatingMode = 0;
                manualCommand = 2;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
            }
            // Manual stop (0)
            else if (receivedData == "0") {
                operatingMode = 0;
                manualCommand = 0;
            }
            // Set outer loop gains (K)
            else if (receivedData.startsWith("K")) {
                operatingMode = 0;
                manualCommand = 0;
                parseOuterLoopGains(receivedData);
            }
            // Set inner loop gains (P)
            else if (receivedData.startsWith("P")) {
                operatingMode = 0;
                manualCommand = 0;
                parseInnerLoopGains(receivedData);
            }
            // Set thresholds (T)
            else if (receivedData.startsWith("T")) {
                operatingMode = 0;
                manualCommand = 0;
                parseThresholds(receivedData);
            }
            // Adaptive control commands
            else if (receivedData == "ADAPTIVE_ON") {
                adaptiveControlEnabled = true;
                Serial.println("Adaptive Control ENABLED");
            }
            else if (receivedData == "ADAPTIVE_OFF") {
                adaptiveControlEnabled = false;
                Serial.println("Adaptive Control DISABLED");
            }
            else if (receivedData == "ADAPTIVE_RESET") {
                resetAdaptiveParameters();
                Serial.println("Adaptive parameters RESET");
            }
            else if (receivedData.startsWith("A") && receivedData.indexOf(',') > 0) {
                parseAdaptiveGains(receivedData);
            }
            else if (receivedData.startsWith("L") && receivedData.indexOf(',') > 0) {
                parseAdaptiveGains(receivedData);
            }
            else if (receivedData == "ADAPTIVE_STATUS") {
                Serial.println("\n=== Adaptive Control Status ===");
                Serial.print("Enabled: ");
                Serial.println(adaptiveControlEnabled ? "YES" : "NO");
                Serial.println("\nParameter Estimates:");
                Serial.print("m_platform: "); Serial.println(P_hat[0], 6);
                Serial.print("m_cylinder: "); Serial.println(P_hat[1], 6);
                Serial.print("m_piston: "); Serial.println(P_hat[2], 6);
                Serial.print("b1: "); Serial.println(P_hat[3], 6);
                Serial.print("b2: "); Serial.println(P_hat[4], 6);
                Serial.print("b3: "); Serial.println(P_hat[5], 6);
                Serial.print("I_platform: "); Serial.println(P_hat[6], 6);
                Serial.print("I_piston: "); Serial.println(P_hat[7], 6);
                Serial.print("I_sleeve: "); Serial.println(P_hat[8], 6);
                Serial.println("==============================\n");
            }
            
            receivedData = "";
        }
    }
    
    //================================================================
    // AUTO MODE EXECUTION (FORWARD OR RETREAT)
    //================================================================
    
    if (operatingMode == 1 || operatingMode == 2) {
        
        // Load cell monitoring (only in forward mode)
        if (operatingMode == 1 && currentTime - lastLoadTime >= loadCellInterval) {
            if (retreatHasBeenTriggered) {
                manipulatorState = 1;
            } else {
                long effectiveValue = readHX711() - loadCellOffset;
                latestValidLoad = effectiveValue / 10000.0;
                int roundValue = round(latestValidLoad);
                
                if (roundValue >= threshold2) {
                    manipulatorState = 1;
                    retreatHasBeenTriggered = true;
                    
                    if (!retreatRequestSent) {
                        Serial.println("RETREAT");
                        retreatRequestSent = true;
                    }
                } else if (roundValue >= threshold1) {
                    manipulatorState = 1;
                } else {
                    manipulatorState = 0;
                }
            }
            lastLoadTime = currentTime;
        }
        
        // Encoder reading
        if (currentTime - lastEncTime >= encoderInterval && manipulatorState != 1) {
            updateEncoders();
            lastEncTime = currentTime;
        }
        
        // Control calculations (only in running state)
        if (manipulatorState == 0) {
            
            // Velocity calculation
            if (currentTime - lastVeloTime >= veloInterval) {
                updateVelocities();
                lastVeloTime = currentTime;
            }
            
            // CTC (outer loop) calculation
            if (currentTime - lastCTCCalcTime >= CTCcalculationInterval) {
                if (adaptiveControlEnabled) {
                    calculateAdaptiveCTC();
                } else {
                    calculateCTC();
                }
                lastCTCCalcTime = currentTime;
            }
            
            // PD (inner loop) calculation
            if (currentTime - lastPDCalcTime >= PDcalculationInterval) {
                calculatePD();
                lastPDCalcTime = currentTime;
            }
        }
        
        // Motor control execution
        if (manipulatorState == 0) {
            applyMotorControl();
        } else {
            stopAllMotors();
        }
        
        // Status reporting
        if (currentTime - lastPrnTime >= loadCellInterval) {
            String statusStr;
            String modeStr;
            
            if (operatingMode == 1) {
                modeStr = "forward";
            } else {
                modeStr = "retreat";
            }
            
            if (manipulatorState == 0) {
                statusStr = "running";
            } else if (manipulatorState == 1) {
                statusStr = "paused";
            } else if (manipulatorState == 2) {
                statusStr = "retreating";
            }
            
            float currentScale = getLoadScaling();
            Serial.print("status:");
            Serial.print(statusStr);
            Serial.print(",mode:");
            Serial.print(modeStr);
            Serial.print(",load:");
            Serial.print(latestValidLoad, 2);
            Serial.print(",scale:");
            Serial.print(currentScale, 2);
            Serial.print(",pos:");
            Serial.print(ActPos1, 2);
            Serial.print(",");
            Serial.print(ActPos2, 2);
            Serial.print(",");
            Serial.print(ActPos3, 2);
            
            // Add adaptive info if enabled
            if (adaptiveControlEnabled && operatingMode == 1) {
                Serial.print(",b1:");
                Serial.print(P_hat[3], 4);
                Serial.print(",b2:");
                Serial.print(P_hat[4], 4);
                Serial.print(",b3:");
                Serial.print(P_hat[5], 4);
            }
            
            Serial.println("");
            
            lastPrnTime = currentTime;
        }
        
    }
    
    //================================================================
    // MANUAL MODE EXECUTION (WITH ADAPTIVE CONTROL)
    //================================================================
    
    else {
        // Read load cell even in manual mode
        if (currentTime - lastLoadTime >= loadCellInterval && ENABLE_ADAPTIVE_MANUAL) {
            long effectiveValue = readHX711() - loadCellOffset;
            latestValidLoad = effectiveValue / 10000.0;
            lastLoadTime = currentTime;
        }
        
        manualModeControl();
    }
}