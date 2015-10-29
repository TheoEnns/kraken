#include <dxl_devices.h> //AX_Utilities,
#include <math.h>  //Physical_Config, Physical_Types_And_Conversions, LegIKEngine,
#include <Dynamixel.h> //AX_Utilities,
#include <string.h> //LegIKEngine (for memset),

#include <usb_Serial.h> //SerialComm.h
#include <HardwareSerial.h> //SerialComm.h

//#define RUN_WITHOUT_SERVOS 1

#include "Physical_Types_And_Conversions.h"
#include "Physical_Config.h"
#include "AX_Utilities.h"
#include "LegIKEngine.h"
#include "GaitGenerator.h"
#include "SerialComm.h"


SerialComm serialComms;
AxManager axm;
LegIKEngine legIK;
GaitManager gaitGen;

unsigned long curTime;
unsigned long volt_Timer = 0;
unsigned long gait_Timer = 0;
unsigned long servo_Timer = 0;
unsigned long volt_Rate = 4000;
unsigned long gait_Rate = GAIT_INTERPOLATION_RATE;
unsigned long servo_Rate = 10;
unsigned long minRate = 5;
unsigned long curMicros = 0;
float averageElapse;
float maxElapse;
int cycleCount;

void report_Health(){
    serialComms.startJsonMsg();
    if(cycleCount>0) {
      serialComms.printVar("Rate_us", averageElapse/(cycleCount));
    } else {
      serialComms.printVar("Rate_us", 0);
    }
    serialComms.printVar("Peak_us", (int)maxElapse);
#ifndef RUN_WITHOUT_SERVOS
    serialComms.printVar("Right_Voltage", axm.servoVoltage(RIGHT_BATTERY_SERVO_ID));
    serialComms.printVar("Left_Voltage", axm.servoVoltage(LEFT_BATTERY_SERVO_ID));
#endif
    serialComms.endJsonMsg();
    averageElapse = 0;
    maxElapse = 0;
    cycleCount = 0;
}
  
void setup() {
    Serial2.begin(115200);
    SerialUSB.begin();
    delay(1000);

    axm.initAxM();
    delay(1000);

    axm.getPositions();
    delay(5);

    //Move to init pose
    axm.speedLimits(350);
    
    //Move HipH to init pose
    SerialUSB.println("HipH positioning...");
    delay(5);
    axm.toggleTorques(hipH_Index, 6, true);
    delay(5);
    initServos(hipH_Index, hipH_Index+6, 512);
    axm.pushPose(hipH_Index, 6);
    delay(750);
    waitOnServos(hipH_Index, hipH_Index + 6);

    //Move HipV to init pose
    SerialUSB.println("HipV positioning...");
    delay(5);
    axm.toggleTorques(hipV_cw_Index, 12, true);
    delay(5);
    initServos(hipV_cw_Index, hipV_cw_Index+6, 819);
    initServos(hipV_ccw_Index, hipV_ccw_Index+6, 205);
    axm.pushPose(hipV_cw_Index, 12);
    delay(750);
    waitOnServos(hipV_cw_Index, hipV_cw_Index+12);

    //Move Knee to init pose
    SerialUSB.println("Knee positioning...");
    delay(5);
    axm.toggleTorques(knee_Index, 6, true);
    delay(5);
    initServos(knee_Index, knee_Index+6, 819);
    axm.pushPose(knee_Index, 6);
    delay(750);
    waitOnServos(knee_Index, knee_Index+6);

    //Move Foot to init pose
    SerialUSB.println("Foot positioning...");
    delay(5);
    axm.toggleTorques(foot_Index, 6, true);
    delay(5);
    initServos(foot_Index, foot_Index+6, 512);
    axm.pushPose(foot_Index, 6);
    delay(750);
//    waitOnServos(foot_Index, foot_Index+6);
    
    for(int servo_Idx = 0; servo_Idx < NUMSERVOS; servo_Idx++){
        servoTable_Target[servo_Idx] = servoTable_Pose[servo_Idx];
    }
    
    delay(100);
    axm.speedLimits(0);
    delay(5);
    axm.toggleTorques(hipH_Index, maxJoint_Index, true);
    delay(5);
//    axm.torqueRange(hipH_Index, foot_Index, 900);
//    axm.torqueRange(hipH_Index, foot_Index, 1023);
    delay(5);
//    axm.torqueRange(foot_Index, maxJoint_Index, 300);
//    axm.torqueRange(foot_Index, maxJoint_Index, 900);
    delay(5);
    axm.freeMoveMode();
//    axm.holdingMode();
    delay(5);
}


void initServos(int startIndx, int endIndx, word pos){
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        axm.setPosition(servo_Idx,pos,false);
    }
}

void waitOnServos(int startIndx, int endIndx){
#ifndef RUN_WITHOUT_SERVOS
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        while (axm.is_moving(servo_Idx)){
            delay(5);
            if (volt_Timer < curTime) {
                report_Health();
                volt_Timer = 4000 + curTime;
            }
        }
    }
#endif
}

void loop() {
    int legCalcIndex = 0;
    int elapse = 0;
    int delta = 0;

    gaitGen.switchGaitModulus(2);
    gaitGen.generateCenters();

    volt_Timer = millis() + volt_Timer;
    gait_Timer = millis() + gait_Rate;
    servo_Timer = millis() + servo_Rate;
    cycleCount = 0;
    bool ignoreCycle = false;

    SerialUSB.println("ik positioning...");
    while(true){
        curTime = millis();
        curMicros = micros();
        if ( (legCalcIndex >= CNT_LEGS) && (gait_Timer < curTime)) {
            gaitGen.setNextTrajectory(0,100,0.0*M_PI);
            gaitGen.pushIKtoTarget();
            axm.initInterpolate(curTime);
            axm.setTimeToArrival(GAIT_INTERPOLATION_TARGET_TIME);

            gaitGen.updateTargetTime(curTime);
            gait_Timer = gait_Rate + curTime;
            legCalcIndex = 0;
        }
        if (servo_Timer < curTime) {
            axm.interpolatePoses(curTime);
            servo_Timer = servo_Rate + curTime;
        }
        if (volt_Timer < curTime) {
            report_Health();
            volt_Timer = volt_Rate + curTime;
            ignoreCycle = true;
        }else{
            ignoreCycle = false;
        }
        if (legCalcIndex < CNT_LEGS) {
            gaitGen.interpolateNextWalk(legCalcIndex);
            legCalcIndex++;
        }
        elapse = micros() - curMicros; 
        //peak: ~32-30 ms when report_Health(),
       //    2 ms peak otherwise
        delta = minRate*1000 - elapse;
        if(!ignoreCycle){
            averageElapse += elapse;
            maxElapse = maxElapse>elapse?maxElapse:elapse;
            cycleCount++;
        }
        if(delta > 0) {
            delayMicroseconds(delta);
        }
    }
}
