#include <dxl_devices.h> //AX_Utilities,
#include <math.h>  //Physical_Config, Physical_Types_And_Conversions, LegIKEngine,
#include <Dynamixel.h> //AX_Utilities,
#include <string.h> //LegIKEngine (for memset),

#include <usb_Serial.h>
#include <HardwareSerial.h>

#include "Physical_Types_And_Conversions.h"
#include "Physical_Config.h"
#include "AX_Utilities.h"
#include "LegIKEngine.h"
#include "SerialComm.h"

SerialComm serialComms;
AxManager axm;
LegIKEngine legIK;

unsigned long curTime;
unsigned long volt_Timer = 0;

void report_Voltage(){
    serialComms.startJsonMsg();
    serialComms.printVar("Right Battery Voltage", axm.servoVoltage(RIGHT_BATTERY_SERVO_ID));
    serialComms.printVar("Left Battery Voltage", axm.servoVoltage(LEFT_BATTERY_SERVO_ID));
    serialComms.endJsonMsg();
}
  
void setup() {
    Serial2.begin(115200);
    SerialUSB.begin();
    delay(1000);

    axm.initAxM();
    delay(100);
    report_Voltage();
    delay(1000);

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
    SerialUSB.println("HipH positioning...");

    //Move HipV to init pose
    delay(5);
    axm.toggleTorques(hipV_cw_Index, 12, true);
    delay(5);
    initServos(hipV_cw_Index, hipV_cw_Index+6, 819);
    initServos(hipV_ccw_Index, hipV_ccw_Index+6, 205);
    axm.pushPose(hipV_cw_Index, 12);
    delay(750);
//    waitOnServos(hipV_cw_Index, hipV_cw_Index+12);
    SerialUSB.println("HipV positioning...");

    //Move Knee to init pose
    delay(5);
    axm.toggleTorques(knee_Index, 6, true);
    delay(5);
    initServos(knee_Index, knee_Index+6, 819);
    axm.pushPose(knee_Index, 6);
    delay(750);
//    waitOnServos(knee_Index, knee_Index+6);
    SerialUSB.println("Knee positioning...");

    //Move Foot to init pose
    delay(5);
    axm.toggleTorques(foot_Index, 6, true);
    delay(5);
    initServos(foot_Index, foot_Index+6, 512);
    axm.pushPose(foot_Index, 6);
    delay(750);
//    waitOnServos(foot_Index, foot_Index+6);
    SerialUSB.println("Foot positioning...");
    
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

float r0 = 95.25 + 175.05;
float z0 = -(122.76 + 32.25);
float p0 = 60.0*3.14159/180.0;
COORD3D effector;
void setTarget(int i, int phase, float period, int legMod, float dr){
    LegIndex indx = LegIndex(i);
    bool isUp = (phase == (i%legMod));
    float periodS; 
    float periodC;
    float vel;
    if(isUp){
        periodS = sin(M_PI*period);
        periodC = cos(M_PI*period);
        vel = dr*(legMod-1);
    }else{
        periodS = 0;
        periodC = cos(M_PI*(period + phase - (i%legMod) - 1)/(legMod-1));
        vel = dr*(-1);
    }
    effector.x = (r0)*cos( p0*(i+1) + vel*periodC);
    effector.y = (r0)*sin( p0*(i+1) + vel*periodC);
    effector.z = (z0) + fabs(90.0*periodS);
    legIK.effector(indx, effector);
    int error = legIK.leg3DOF_Inverse(indx);
    if (error>0) {
        serialComms.startJsonMsg();
        serialComms.printVar("IK_Error", error);
        serialComms.endJsonMsg();
    }else{
        servoTable_Target[hipH_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipH(indx),true),0,1023);
        servoTable_Target[hipV_cw_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(indx),true),0,1023);
        servoTable_Target[hipV_ccw_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(indx),false),0,1023);
        servoTable_Target[knee_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_knee(indx),false),0,1023);
        servoTable_Target[foot_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_ankle(indx),false),0,1023);
        
        axm.setTimeToArrival(100);
        axm.initInterpolate(curTime);
    }
}

void loop() {
    int averagePerLegIK = 0;
    int ikCounter = 0;
    unsigned long effTimer;
    int legMod = 3;
    float stepTime = 2000.0;
//    float x0 = 95.25 + 175.05;
//    float y0 = 95.25 + 175.05;
//    float z0 = -(122.76 + 32.25);
//    float p0 = 60.0*3.14159/180.0;
    SerialUSB.println("ik positioning...");  
   
    while(true){
        curTime = millis();
        if (axm.interpolateFinished()) {
//            float periodS = sin(M_PI*(millis()/2000.0f));
//            float periodC = cos(M_PI*(millis()/2000.0f));
            int phase = ((int)(curTime/(stepTime)))%legMod;
            float period = ((float)curTime)/stepTime;
            period = period -  floor(period);
            for(int i =0;i<6;i++){
//                LegIndex indx = LegIndex(i);
                  effTimer = micros();
                  setTarget(i, phase, period, legMod, M_PI*30.0/180);
//                COORD3D effector;
//                int isUp = periodS>=0?i%2:(i+1)%2;
//                int dir = ((1-2*i)%2);
//                effector.x = (x0)*cos( p0*(i+1) + dir*((0) - (p0/2)*periodC));
//                effector.y = (y0)*sin( p0*(i+1) + dir*((0) - (p0/2)*periodC));
//                effector.z = (z0) + fabs(90.0*periodS)*isUp;
//                legIK.effector(indx, effector);
//                error = legIK.leg3DOF_Inverse(indx);
//                if (error>0) {
//                    serialComms.startJsonMsg();
//                    serialComms.printVar("IK_Error", error);
//                    serialComms.endJsonMsg();
//                    error = 0;
//                }else{
//                    servoTable_Target[hipH_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipH(indx),true),0,1023);
//                    servoTable_Target[hipV_cw_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(indx),true),0,1023);
//                    servoTable_Target[hipV_ccw_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(indx),false),0,1023);
//                    servoTable_Target[knee_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_knee(indx),false),0,1023);
//                    servoTable_Target[foot_Index + i] = LIMIT(ANGLE_TO_SERVO(legIK.angle_ankle(indx),false),0,1023);
//                    
//                    axm.setTimeToArrival(100);
//                    axm.initInterpolate(curTime);
//                }
                effTimer = micros()-effTimer;
                averagePerLegIK += effTimer;
                ikCounter++;
            }
            delayMicroseconds(500);
        }else{
          delay(4);
        }
        if (volt_Timer < curTime) {
            report_Voltage();
            volt_Timer = 4000 + curTime;
            serialComms.startJsonMsg(true);
            serialComms.printVar("IK per Leg (us)", (averagePerLegIK/(40)),true);
            serialComms.endJsonMsg(true);
            averagePerLegIK = 0;
            ikCounter = 0;
        }
        axm.interpolatePoses(curTime);
        delay(6);
    }
}
