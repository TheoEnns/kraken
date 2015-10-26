#include <dxl_devices.h>

#include "AX_Utilities.h"
#include "Physical_Config.h"
#include "LegIKEngine.h"
#include "SerialComm.h"

AxManager axm;
LegIKEngine legIK;

unsigned long curTime;
unsigned long volt_Timer = 0;

void report_Voltage(){
    startJsonMsg();
    printVar("Right Battery Voltage", axm.voltage(RIGHT_BATTERY_SERVO_ID));
    printVar("Left Battery Voltage", axm.voltage(LEFT_BATTERY_SERVO_ID));
    endJsonMsg();
}
  
void setup() {
//    Serial1.begin(115200);
//    addSerial( &Serial1, false);
    Serial2.begin(115200);
    addSerial( &Serial2, false);
    SerialUSB.begin();
    addSerial( &SerialUSB, true);

    axm.initAxM();
    delay(100);

    axm.getPositions();
    delay(5);

    //Move to init pose
    axm.speedLimits(350);
    
    //Move HipH to init pose
    delay(5);
    axm.toggleTorques(hipH_Index, 6, true);
    delay(5);
    initServos(hipH_Index, hipH_Index+6, 512);
    axm.pushPose(hipH_Index, 6);
    delay(750);
//    waitOnServos(hipH_Index, hipH_Index + 6);
    SerialUSB.println("HipH");

    //Move HipV to init pose
    delay(5);
    axm.toggleTorques(hipV_cw_Index, 12, true);
    delay(5);
    initServos(hipV_cw_Index, hipV_cw_Index+6, 819);
    initServos(hipV_ccw_Index, hipV_ccw_Index+6, 205);
    axm.pushPose(hipV_cw_Index, 12);
    delay(750);
//    waitOnServos(hipV_cw_Index, hipV_cw_Index+12);
    SerialUSB.println("HipV");

    //Move Knee to init pose
    delay(5);
    axm.toggleTorques(knee_Index, 6, true);
    delay(5);
    initServos(knee_Index, knee_Index+6, 819);
    axm.pushPose(knee_Index, 6);
    delay(750);
//    waitOnServos(knee_Index, knee_Index+6);
    SerialUSB.println("Knee");

    //Move Foot to init pose
    delay(5);
    axm.toggleTorques(foot_Index, 6, true);
    delay(5);
    initServos(foot_Index, foot_Index+6, 512);
    axm.pushPose(foot_Index, 6);
    delay(750);
//    waitOnServos(foot_Index, foot_Index+6);
    SerialUSB.println("Foot");
    
    for(int servo_Idx = 0; servo_Idx < NUMSERVOS; servo_Idx++){
        servoTable_Target[servo_Idx] = servoTable_Pose[servo_Idx];
    }
    
    delay(100);
    axm.speedLimits(0);
    delay(5);
    axm.toggleTorques(hipH_Index, maxJoint_Index, true);
    delay(5);
    axm.torqueRange(hipH_Index, foot_Index, 900);
    delay(5);
    axm.torqueRange(foot_Index, maxJoint_Index, 300);
    delay(5);
    axm.freeMoveMode();
    delay(5);
}


void initServos(int startIndx, int endIndx, word pos){
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        axm.setPosition(servo_Idx,pos,false);
    }
}

void waitOnServos(int startIndx, int endIndx){
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        while (axm.is_moving(servo_Idx)){
            delay(5);
            if (volt_Timer < curTime) {
                report_Voltage();
                volt_Timer = 4000 + curTime;
            }
        }
    }
}

void oscillate(int swing, int jointOffset){
    int servo_idx;
    for(int i = 0; i < legCount; i++) {
        servo_idx = i + jointOffset;
        servoTable_Target[servo_idx] = 512 + swing;
    }
}

void loop() {
    float x0 = 95.25 + 175.05;
    float y0 = 95.25 + 175.05;
    float z0 = -(122.76 + 32.25);
    float p0 = 60.0*3.14159/180.0;
    int error = 0;
    while(true){
        curTime = millis();
        if (axm.interpolateFinished()) {
            COORD3D effector;
            effector.x = (x0)*cos(p0);
            effector.y = (y0)*sin(p0);
            effector.z = (z0);
            legIK.effector(legIndex_ForeRight, effector);
            error = legIK.leg3DOF_Inverse(legIndex_ForeRight);
            if (error>0) {
                startJsonMsg();
                printVar("IK_Error", error);
                endJsonMsg();
                error = 0;
            }else{
//                servoTable_Target[hipH_Index + legIndex_ForeRight] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipH(legIndex_ForeRight),true),0,1023);
//                servoTable_Target[hipV_cw_Index + legIndex_ForeRight] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndex_ForeRight),false),0,1023);
//                servoTable_Target[hipV_ccw_Index + legIndex_ForeRight] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndex_ForeRight),true),0,1023);
//                servoTable_Target[knee_Index + legIndex_ForeRight] = LIMIT(ANGLE_TO_SERVO(legIK.angle_knee(legIndex_ForeRight),true),0,1023);
//                servoTable_Target[foot_Index + legIndex_ForeRight] = LIMIT(ANGLE_TO_SERVO(legIK.angle_ankle(legIndex_ForeRight),true),0,1023);
                
                axm.setTimeToArrival(3000);
                axm.initInterpolate(curTime);
            
                startJsonMsg(true);
                printVar("e.x", legIK.effector(legIndex_ForeRight).x,true);
                printVar("e.y", legIK.effector(legIndex_ForeRight).y,true);
                printVar("e.z", legIK.effector(legIndex_ForeRight).z,true);
                printVar("hipH", legIK.angle_hipH(legIndex_ForeRight),true);
                printVar("hipV_cw", legIK.angle_hipV(legIndex_ForeRight),true);
                printVar("hipV_ccw", legIK.angle_hipV(legIndex_ForeRight),true);
                printVar("knee", legIK.angle_knee(legIndex_ForeRight),true);
                printVar("foot", legIK.angle_ankle(legIndex_ForeRight),true);
                endJsonMsg(true);
                
                startJsonMsg(true);
                printVar("hipH", servoTable_Target[hipH_Index + legIndex_ForeRight],true);
                printVar("hipV_cw", servoTable_Target[hipV_cw_Index + legIndex_ForeRight],true);
                printVar("hipV_ccw", servoTable_Target[hipV_ccw_Index + legIndex_ForeRight],true);
                printVar("knee", servoTable_Target[knee_Index + legIndex_ForeRight],true);
                printVar("foot", servoTable_Target[foot_Index + legIndex_ForeRight],true);
                endJsonMsg(true);
            }
        }
        if (volt_Timer < curTime) {
            report_Voltage();
            volt_Timer = 4000 + curTime;
        }
        axm.interpolatePoses(curTime);
        delay(5);
    }
}
