//
// Created by Theo on 10/17/2015.
//

#ifndef KRAKEN_AX_UTILITIES_H
#define KRAKEN_AX_UTILITIES_H

#include "Physical_Config.h"

#define FREE_MOVE_SLOPE 0x80
#define HOLDING_SLOPE 0x20

#define dxlSuccessThreshold 2
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1) for 1000000 baud  <-OpenCM9.04
Dynamixel Dxl(DXL_BUS_SERIAL1);

enum AxManagerError_t{
    AxM_Healthy = 0,
    AxM_InvalidResponseFromServo,//No response or corrupt
    AxM_NumStatusStates
};

class AxManager {
public:
    AxManager(void);
    ~AxManager();
    void initAxM();

    //--------------//
    //  Interpolation Control
    void initInterpolate(unsigned long currentTimeMillis);
    bool interpolatePoses(unsigned long currentTimeMillis);
    bool interpolatePoses(unsigned long currentTimeMillis, bool zenoParadox);
    bool interpolatePoses(unsigned long currentTimeMillis, bool doPush, bool zenoParadox);
    void setTimeToArrival(unsigned long t2a);
    bool interpolateFinished();
    
    void holdingMode(unsigned long idx_setMask);
    void freeMoveMode(unsigned long idx_setMask);
    void holdingMode();
    void freeMoveMode();

    //--------------//
    //  Position
    bool pushPose(int start_idx, int length_idx);
    bool pushPose(); //Takes an average of 1270 microsecs without speeds
    bool setAllPositions(word pos);
    bool getPositions();
    void setPosition(int servo_idx, word pos, bool useTXRX);
    int getPosition(int servo_idx, bool useTXRX, bool * success);
    int getPosition(int servo_idx, bool useTXRX);

    //--------------//
    //  Speed
    void speedLimits(word speed);
    void speedLimit(int servo_idx, word speed);

    //--------------//
    //  Torque
    bool torqueRange(int start_idx, int length_idx, word torque);
    int torque(int servo_idx);
    void torque(int servo_idx, word torque);
    void toggleTorques(int start_idx, int length_idx, bool turnOn);
    void toggleTorques(bool turnOn);
    void toggleTorque(int servo_idx, bool turnOn);

    //--------------//
    //  Misc
    float servoVoltage(int servo_idx);
    bool is_moving(int servo_idx);

private:
    unsigned long lastInterpolationTimeMillis;
    unsigned long timeToArrivalMillis;
    int errorState;

    void _initAxM();
//    bool bulkTransmitTable(uint32_t idx_setMask, word * table, int registerIndx, int regLength);
//    bool bulkTransmit(uint32_t idx_setMask, word value, int registerIndx, int regLength);
    bool bulkTransmitTable(int start_idx, int length_idx, word * table, int registerIndx, int regLength);
    bool bulkTransmit(int start_idx, int length_idx, word value, int registerIndx, int regLength);
};

AxManager::AxManager(void){
    errorState = AxM_Healthy;
    lastInterpolationTimeMillis = 0;
    timeToArrivalMillis = 0;
}

AxManager::~AxManager(){

}

void AxManager::initAxM(){
  _initAxM();
}

void AxManager::_initAxM(){
    Dxl.begin(3);

    // Inits

    //Use bulk commands when possible

    //Init pose arrays
    for(int servo_idx=0;servo_idx<NUMSERVOS;servo_idx++) {
        Dxl.jointMode(servoTable_ID[servo_idx]);
        servoTable_Pose[servo_idx] = 512;
        servoTable_Torque[servo_idx] = 0;
    }

    //Grab current servo positions
    bool error = getPositions();
    if(error)
        errorState |= 1<<AxM_InvalidResponseFromServo;

    //init target array
    for(int servo_idx=0;servo_idx<NUMSERVOS;servo_idx++) {
        servoTable_Target[servo_idx] = servoTable_Pose[servo_idx];
    }

    Dxl.writeWord( BROADCAST_ID, AXM_MOVING_SPEED_L, 0 );
}

//--------------//
//  Interpolation Control
void AxManager::initInterpolate(unsigned long currentTimeMillis){
    lastInterpolationTimeMillis = currentTimeMillis;
}

//currentTimeMillis: get from millis(), rolls over in ~50 days, so no concern for a hexapod
//zenoParadox:  makes the servo endlessly approach but never reach its target
bool AxManager::interpolatePoses(unsigned long currentTimeMillis) {
    return interpolatePoses(currentTimeMillis, true, false);
}

//currentTimeMillis: get from millis(), rolls over in ~50 days, so no concern for a hexapod
//zenoParadox:  makes the servo endlessly approach but never reach its target
bool AxManager::interpolatePoses(unsigned long currentTimeMillis, bool zenoParadox) {
    return interpolatePoses(currentTimeMillis, true, zenoParadox);
}

//currentTimeMillis: get from millis(), rolls over in ~50 days, so no concern for a hexapod
//doPush:       pushes interpolation to servos
//zenoParadox:  makes the servo endlessly approach but never reach its target
bool AxManager::interpolatePoses(unsigned long currentTimeMillis, bool doPush, bool zenoParadox) {
    bool isSuccess = true;
    unsigned long elapseTime = currentTimeMillis - lastInterpolationTimeMillis;
    float distanceFraction, reverseFraction = 0;
    if (timeToArrivalMillis > elapseTime) {
        distanceFraction = ((float)elapseTime) / ((float)timeToArrivalMillis);
        reverseFraction = 1.0f - distanceFraction;
        if (!zenoParadox)
            timeToArrivalMillis -= elapseTime;
    } else {
        distanceFraction = 1;
        reverseFraction = 0;
        if (!zenoParadox)
            timeToArrivalMillis = 0;
    }

    for(int servo_idx=0; servo_idx<NUMSERVOS; servo_idx++ ){
        servoTable_Pose[servo_idx] = distanceFraction*servoTable_Target[servo_idx]
                                     + reverseFraction*servoTable_Pose[servo_idx];
    }

    if(doPush){
        isSuccess = pushPose();
    }

    lastInterpolationTimeMillis = currentTimeMillis;
    return isSuccess;
}

void AxManager::setTimeToArrival(unsigned long t2a){
    timeToArrivalMillis = t2a;
}

////Returns the servo's masked to the normal hold values for exerting force
//void AxManager::holdingMode(unsigned long idx_setMask){
//    word slope = DXL_MAKEWORD(HOLDING_SLOPE, HOLDING_SLOPE);
//    bulkTransmit(idx_setMask, slope, AXM_CW_COMPLIANCE_SLOPE, 2);
//}
//
////Switches servo's masked to a free-move state for no resistance movements to eliminate shaking
//void AxManager::freeMoveMode(unsigned long idx_setMask){
//    word slope = DXL_MAKEWORD(FREE_MOVE_SLOPE, FREE_MOVE_SLOPE);
//    bulkTransmit(idx_setMask, slope, AXM_CW_COMPLIANCE_SLOPE, 2);
//}

//Returns the servo's masked to the normal hold values for exerting force
void AxManager::holdingMode(){
    word slope = DXL_MAKEWORD(HOLDING_SLOPE, HOLDING_SLOPE);
    Dxl.writeWord( BROADCAST_ID, AXM_CW_COMPLIANCE_SLOPE, slope );
}

//Switches servo's masked to a free-move state for no resistance movements to eliminate shaking
void AxManager::freeMoveMode(){
    word slope = DXL_MAKEWORD(FREE_MOVE_SLOPE, FREE_MOVE_SLOPE);
    Dxl.writeWord( BROADCAST_ID, AXM_CW_COMPLIANCE_SLOPE, slope );
}

bool AxManager::interpolateFinished(){
    return (timeToArrivalMillis == 0);
}

//--------------//
//  Position
bool AxManager::pushPose(int start_idx, int length_idx){
    return bulkTransmitTable(start_idx, length_idx, servoTable_Pose, AXM_GOAL_POSITION_L, 2);
}

bool AxManager::pushPose(){
    return bulkTransmitTable(0, NUMSERVOS, servoTable_Pose, AXM_GOAL_POSITION_L, 2);
}

bool AxManager::setAllPositions(word pos){
    for(int servo_idx=0; servo_idx < NUMSERVOS; servo_idx++) {
        servoTable_Pose[servo_idx] = pos;
    }
    return pushPose();
}

//This is a good test for servo presence!
bool AxManager::getPositions(){
    bool success = true;
    for(int servo_idx = 0; servo_idx<NUMSERVOS; servo_idx++)
        getPosition(servo_idx,true,&success);
    return success;
}

void AxManager::setPosition(int servo_idx, word pos, bool useTXRX){
    pos = LIMIT(pos,0,1023);
    servoTable_Pose[servo_idx] = pos;
    if(useTXRX)
        Dxl.goalPosition(servoTable_ID[servo_idx],pos);
}

int AxManager::getPosition(int servo_idx, bool useTXRX, bool * success) {
    if (useTXRX) {
        word pos = Dxl.getPosition(servoTable_ID[servo_idx]);
        servoTable_Pose[servo_idx] = (pos <= 1023) && (pos >= 0) ? pos : servoTable_Pose[servo_idx];
        *success = (pos <= 1023) && (pos >= 0) ? (*success): false;
    }
    return servoTable_Pose[servo_idx];
}

int AxManager::getPosition(int servo_idx, bool useTXRX){
    if (useTXRX) {
        word pos = Dxl.getPosition(servoTable_ID[servo_idx]);
        servoTable_Pose[servo_idx] = (pos <= 1023) && (pos >= 0) ? pos : servoTable_Pose[servo_idx];
    }
    return servoTable_Pose[servo_idx];
}

//--------------//
//  Speed

void AxManager::speedLimits(word speed){
    Dxl.writeWord( BROADCAST_ID, AXM_MOVING_SPEED_L, speed );
}

void AxManager::speedLimit(int servo_idx, word speed){
    Dxl.goalSpeed(servoTable_ID[servo_idx], speed);
}

//--------------//
//  Torque
bool AxManager::torqueRange(int start_idx, int length_idx, word torque){
    for(int servo_idx = start_idx; servo_idx < (start_idx + length_idx); servo_idx++)
        servoTable_Torque[servo_idx] = torque;
    return bulkTransmitTable(start_idx, length_idx, servoTable_Torque, AXM_MAX_TORQUE_L, 2);
}

int AxManager::torque(int servo_idx){
    return Dxl.getLoad(servoTable_ID[servo_idx]);
}

void AxManager::torque(int servo_idx, word torque){
    servoTable_Torque[servo_idx] = torque;
    Dxl.goalTorque(servoTable_ID[servo_idx],torque);
}

void AxManager::toggleTorques(int start_idx, int length_idx, bool turnOn){
    //Turning on switches the servo to use the max torque setting as the torque limit
    // (my servos are configed to 1023)
    word val = turnOn?MAX_SERVO_TORQUE:0;
    for(int servo_idx = start_idx; servo_idx < (start_idx + length_idx); servo_idx++){
        servoTable_Torque[servo_idx] = val;
    }
    bulkTransmit(start_idx, length_idx, val, AXM_TORQUE_ENABLE, turnOn?1:0);
}

void AxManager::toggleTorques(bool turnOn){
    //Turning on switches the servo to use the max torque setting as the torque limit
    // (my servos are configed to 1023)
    word val = turnOn?MAX_SERVO_TORQUE:0;
    for(int servo_idx = 0; servo_idx<NUMSERVOS;servo_idx++)
        servoTable_Torque[servo_idx] = val;
    Dxl.writeWord( BROADCAST_ID, AXM_TORQUE_ENABLE, turnOn?1:0 );
}

void AxManager::toggleTorque(int servo_idx, bool turnOn){
    //Turning on switches the servo to use the max torque setting as the torque limit
    // (my servos are configed to 1023)
    int val = turnOn?MAX_SERVO_TORQUE:0;
    servoTable_Torque[servo_idx] = val;
    if(turnOn)
        Dxl.torqueEnable(servoTable_ID[servo_idx]);
    else
        Dxl.torqueDisable(servoTable_ID[servo_idx]);
}

//--------------//
//  Misc
float AxManager::servoVoltage(int servo_idx){
    //AX returns a value equal to .1V
    int decVolt = Dxl.getVolt(servoTable_ID[servo_idx]);
//    decVolt = decVolt==255:-1:decVolt; //255 means the servo is not responding
    return 0.1f*decVolt;
}

//Return true if traveling
bool AxManager::is_moving(int servo_idx){
    return (Dxl.isMoving(servoTable_ID[servo_idx])!=0);
}

bool AxManager::bulkTransmitTable(int start_idx, int length_idx, word * table, int registerIndx, int regLength){
    Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);

    Dxl.pushByte(registerIndx);
    Dxl.pushByte(regLength);

    for(int servo_idx=start_idx; servo_idx< (start_idx + length_idx); servo_idx++ ){
        if ( (registerIndx == AXM_GOAL_POSITION_L) && (servoTable_Torque[servo_idx] == 0) )
            continue;
        Dxl.pushByte(servoTable_ID[servo_idx]);
        if (regLength == 2) {
            Dxl.pushByte(DXL_LOBYTE(table[servo_idx]));
            Dxl.pushByte(DXL_HIBYTE(table[servo_idx]));
        } else if(regLength == 1){
            Dxl.pushByte(DXL_LOBYTE(table[servo_idx]));
        }
    }
    Dxl.flushPacket();

    return Dxl.getResult() < dxlSuccessThreshold; //Return codes above 1 are errors
}

bool AxManager::bulkTransmit(int start_idx, int length_idx, word value, int registerIndx, int regLength){
    Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);

    Dxl.pushByte(registerIndx);
    Dxl.pushByte(regLength);

    for(int servo_idx=start_idx; servo_idx < (start_idx + length_idx); servo_idx++ ){
        if ( (registerIndx == AXM_GOAL_POSITION_L) && (servoTable_Torque[servo_idx] == 0) )
            continue;
        Dxl.pushByte(servoTable_ID[servo_idx]);
        if (regLength == 2) {
            Dxl.pushByte(DXL_LOBYTE(value));
            Dxl.pushByte(DXL_HIBYTE(value));
        } else if(regLength == 1){
            Dxl.pushByte(DXL_LOBYTE(value));
        }
    }
    Dxl.flushPacket();

    return Dxl.getResult() < dxlSuccessThreshold; //Return codes above 1 are errors
}

//bool AxManager::bulkTransmit(uint32_t idx_setMask, word value, int registerIndx, int regLength)
//{
//    Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
//    
//    Dxl.pushByte((byte)(registerIndx));
//    Dxl.pushByte((byte)(regLength));
//
//    for(int servo_idx = 0; servo_idx < NUMSERVOS; servo_idx++ ){
//        if ( (idx_setMask & (1<<servo_idx)) == 0 ){
//            continue;
//        }
//        if ( (registerIndx == AXM_GOAL_POSITION_L) && (servoTable_Torque[servo_idx] == 0) ){
//            continue;
//        }
//        Dxl.pushByte((byte)(servoTable_ID[servo_idx]));
//        if (regLength == 2) {
//            Dxl.pushByte((byte)(DXL_LOBYTE(value)));
//            Dxl.pushByte((byte)(DXL_HIBYTE(value)));
//        } else if(regLength == 1){
//            Dxl.pushByte((byte)(DXL_LOBYTE(value)));
//        }
//    }
//    Dxl.flushPacket();
//    
//    bool success = Dxl.getResult() < dxlSuccessThreshold; //Return codes above 1 are errors
//}
//
//bool AxManager::bulkTransmitTable(uint32_t idx_setMask, word * table, int registerIndx, int regLength)
//{
//    Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
//    
//    Dxl.pushByte(registerIndx);
//    Dxl.pushByte(regLength);
//
//    for(int servo_idx=0; servo_idx< NUMSERVOS; servo_idx++ ){
//        if ( (idx_setMask & (1<<servo_idx)) == 0 )
//            continue;
//        if ( (registerIndx == AXM_GOAL_POSITION_L) && (servoTable_Torque[servo_idx] == 0) )
//            continue;
//        Dxl.pushByte(servoTable_ID[servo_idx]);
//        if (regLength == 2) {
//            Dxl.pushByte(DXL_LOBYTE(table[servo_idx]));
//            Dxl.pushByte(DXL_HIBYTE(table[servo_idx]));
//        } else if(regLength == 1){
//            Dxl.pushByte(DXL_LOBYTE(table[servo_idx]));
//        }
//    }
//    Dxl.flushPacket();
//    
//    bool success = Dxl.getResult() < dxlSuccessThreshold; //Return codes above 1 are errors
//}

extern AxManager axm;

#endif //KRAKEN_AX_UTILITIES_H
