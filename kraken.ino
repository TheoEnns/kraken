#include <dxl_devices.h> //AX_Utilities,
#include <math.h>  //Physical_Config, Physical_Types_And_Conversions, LegIKEngine,
#include <Dynamixel.h> //AX_Utilities,
#include <string.h> //LegIKEngine (for memset),

#include <usb_Serial.h> //SerialComm.h
#include <HardwareSerial.h> //SerialComm.h

// COMPILE OPTIONS
#define RUN_WITHOUT_SERVOS 1
//#define REPORT_CPU_STATS 1

//Local includes
#include "Physical_Types_And_Conversions.h"
#include "Physical_Config.h"
#include "AX_Utilities.h"
#include "LegIKEngine.h"
#include "GaitGenerator.h"
#include "SerialComm.h"
#include "Commander.h"

//Hexapod Objects
SerialComm serialComms;
AxManager axm;
LegIKEngine legIK;
GaitManager gaitGen;
Commander command = Commander();

//Runtime variables
unsigned long cpu_Timer;
int cpu_TimeOffset;
unsigned long health_Timer = 0;
unsigned long gait_Timer = 0;
unsigned long servo_Timer = 0;
unsigned long health_Rate = 4000;
unsigned long gait_Rate = GAIT_INTERPOLATION_RATE;
unsigned long servo_Rate = 10;
unsigned long minCPURate = 5;
unsigned long curMicros = 0;
float averageElapse;
float maxElapse;
int cycleCount;
float velY = 0;
float velX = 0;
float velR = 0;
int legIndex_IKFinished = 0;
int elapseCPU = 0;
int delta_CPUCycleTime = 0;
int toggleRobotPowerCounter; //On/Off button on remote

#define ROBOT_POWER_TRIGGER  30

enum HEALTH_ERROR {
    HEALTH_ERR_RIGHT_BATT_EXHAUSTED,
    HEALTH_ERR_LEFT_BATT_EXHAUSTED,
    HEALTH_ERR_CNT, //Limit 32 error states (form a bitmask)
};

#define BATTERY_SHUTDOWN_VOLTAGE   10.2
  
void setup() {
    Serial2.begin(115200);
    SerialUSB.begin();
    delay(2000);
}

void loop() {
    //Init Gait
    axm.initAxM();
    gaitGen.switchGaitModulus(2);
    gaitGen.generateCenters();
    toggleRobotPowerCounter = 0;

    //Prep Timers
    health_Timer = millis() + health_Timer;
    gait_Timer = millis() + gait_Rate;
    servo_Timer = millis() + servo_Rate;
    cycleCount = 0;
    
    //Check health
    bool isHealthy = report_Health() == 0;
    
    if(isHealthy) {
        //Get to safe start pose
        initToSafePosition();
        axm.toggleTorques(true);
        delay(100);
    
        //Stand up
        SerialUSB.println("standing...");
        delay(5);
        axm.holdingMode();
        delay(5);
        gaitGen.setNextTrajectory( 0, 0, 0);
        for(int i=0; i < CNT_LEGS;i++) {
          gaitGen.interpolateNextWalk((LegIndex)i);
        }
        gaitGen.pushIKtoTarget();
        cpu_TimeOffset = 1300;
        axm.initInterpolate(millis());
        axm.setTimeToArrival(cpu_TimeOffset);
        while(!axm.interpolateFinished()){
            delay(servo_Rate);
            cpu_Timer = millis();
            axm.interpolatePoses(cpu_Timer);
        }    
        
        //Enter Walk State
        SerialUSB.println("ready to walk...");
        command.begin(38400); 
        while( isHealthy && (toggleRobotPowerCounter < ROBOT_POWER_TRIGGER)){
          isHealthy = performCycle(true, true, true, true);
        }
        
        
        //Sit Down
        delay(50);
        SerialUSB.println("sitting down...");
        for(int i=0; i < CNT_LEGS;i++){
            COORD3D curVal = legIK.effector( (LegIndex)i);
            curVal.z = 10;
            legIK.effector( (LegIndex)i, curVal);
            legIK.leg3DOF_Inverse((LegIndex)i);
        }
        gaitGen.pushIKtoTarget();
        cpu_TimeOffset = 2500;
        axm.initInterpolate(millis());
        axm.setTimeToArrival(cpu_TimeOffset);
        delay(5);
        axm.holdingMode();
        delay(5);
        while(!axm.interpolateFinished()){
            delay(servo_Rate);
            cpu_Timer = millis();
            axm.interpolatePoses(cpu_Timer);
        }    
        initServoTable(hipH_Index, hipH_Index+6, 512);
        initServoTable(hipV_cw_Index, hipV_cw_Index+6, 819);
        initServoTable(hipV_ccw_Index, hipV_ccw_Index+6, 155);
        initServoTable(knee_Index, knee_Index+6, 750);
//      initServoTable(foot_Index, foot_Index+6, 512);
        axm.pushPose(0, NUMSERVOS);
        waitOnServos(0, NUMSERVOS);
        
        
        axm.toggleTorques(false);
        delay(100);
    }
    
    //Battery Save!
    toggleRobotPowerCounter = 0;
    while( true & (toggleRobotPowerCounter < ROBOT_POWER_TRIGGER)){
        performCycle(true, false, false, false);
    }
}

void initServoTable(int startIndx, int endIndx, word pos){
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        axm.setPosition(servo_Idx,pos,false);
    }
}

void waitOnServos(int startIndx, int endIndx){
   delay(250);
#ifndef RUN_WITHOUT_SERVOS
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        while (axm.is_moving(servo_Idx)){
            delay(5);
            if (health_Timer < cpu_Timer) {
                report_Health();
                health_Timer = 2500 + cpu_Timer;
            }
        }
    }
#endif
}

int report_Health(){
    int healthCode = 0;
    float rightBatt = axm.servoVoltage(RIGHT_BATTERY_SERVO_ID);
    float leftBatt = axm.servoVoltage(LEFT_BATTERY_SERVO_ID);
    if (rightBatt < BATTERY_SHUTDOWN_VOLTAGE) {
      healthCode = healthCode | (1<<HEALTH_ERR_RIGHT_BATT_EXHAUSTED);
    }
    if (leftBatt < BATTERY_SHUTDOWN_VOLTAGE) {
      healthCode = healthCode | (1<<HEALTH_ERR_LEFT_BATT_EXHAUSTED);
    }    
    
    serialComms.startJsonMsg();
    serialComms.printVar("Health", healthCode);
#ifdef REPORT_CPU_STATS
    if(cycleCount>0) {
      serialComms.printVar("Rate_us", averageElapse/(cycleCount));
    } else {
      serialComms.printVar("Rate_us", 0);
    }
    serialComms.printVar("Peak_us", (int)maxElapse);
#endif
    serialComms.printVar("Right_Voltage", rightBatt);
    serialComms.printVar("Left_Voltage", leftBatt);
    serialComms.endJsonMsg();
    
    //Reset CPU Stats
    averageElapse = 0;
    maxElapse = 0;
    cycleCount = 0;
    
    return healthCode;
}

bool performCycle(bool reportOnHealth, bool useAxmInterpolation, bool useLegIK, bool useGaitGen){
    bool stopTrigger = false;
    bool ignoreCycle = false;
    cpu_Timer = millis();
    curMicros = micros();
    
    if (useGaitGen) { //Push IK and Prep the next IK
      if ( (legIndex_IKFinished >= CNT_LEGS) && (gait_Timer < cpu_Timer)) {
          gaitGen.setNextTrajectory( velX, velY, velR);// |Vel| <= 90, |Rot| <= 0.2*pi
          gaitGen.pushIKtoTarget();
          cpu_TimeOffset = gaitGen.getInterpolationTime();
          axm.initInterpolate(cpu_Timer);
          axm.setTimeToArrival(cpu_TimeOffset);
  
          gaitGen.updateTargetTime(cpu_Timer);
          gait_Timer = gait_Rate + cpu_Timer;
          legIndex_IKFinished = 0;
      }
    }
        
    if (useAxmInterpolation) {
      if (servo_Timer < cpu_Timer) {
          axm.interpolatePoses(cpu_Timer);
          servo_Timer = servo_Rate + cpu_Timer;
      }
    }
      
    if (reportOnHealth) {
      if (health_Timer < cpu_Timer) {
          if(report_Health() > 0)
              stopTrigger = true;
          health_Timer = health_Rate + cpu_Timer;
          ignoreCycle = true;
      }
    }
      
    if (useLegIK) { //Do partial IK calc
      if (legIndex_IKFinished < CNT_LEGS) {
          gaitGen.interpolateNextWalk(legIndex_IKFinished);
          legIndex_IKFinished++;
      }
    }
    
    if(command.ReadMsgs()>0){
        if ( ((command.buttons&BUT_LT) > 0) && ((command.buttons&BUT_RT) > 0) ) {
            toggleRobotPowerCounter += 1;
        } else if((command.buttons&BUT_LT) > 0){
            gaitGen.switchGaitModulus(2);
        
        } else if((command.buttons&BUT_RT) > 0) {
            gaitGen.switchGaitModulus(3);
        }else{
          toggleRobotPowerCounter = 0;
        }
        velX = (fabs(command.walkH)>20.0)?DEF_MAX_VEL_TRANS*(command.walkH)/(103.0 + 1.42*fabs(command.walkV)):0;
        velY = (fabs(command.walkV)>20.0)?DEF_MAX_VEL_TRANS*(command.walkV)/(103.0 + 1.42*fabs(command.walkH)):0;
        velR = (fabs(command.lookH)>20.0)?DEF_MAX_VEL_ROTATE*(command.lookH)/(103.0):0;
    }
    
    //Time delay
    elapseCPU = micros() - curMicros; 
    //peak: ~32-30 ms when report_Health(),
    //    2 ms peak otherwise
    delta_CPUCycleTime = minCPURate*1000 - elapseCPU;
    if(!ignoreCycle){
        averageElapse += elapseCPU;
        maxElapse = maxElapse>elapseCPU?maxElapse:elapseCPU;
        cycleCount++;
    } else {
      ignoreCycle = false;
    }
    if(delta_CPUCycleTime > 0) {
        delayMicroseconds(delta_CPUCycleTime);
    }
    return !stopTrigger;
}

void initToSafePosition(){
    axm.getPositions();
    delay(5);

    //Move to init pose
    axm.speedLimits(350);
    delay(5);
//    axm.freeMoveMode();
    delay(5);
    
    //Move HipH to init pose
    SerialUSB.println("HipH positioning...");
    delay(5);
    axm.toggleTorques(hipH_Index, 6, true);
    delay(5);
    initServoTable(hipH_Index, hipH_Index+6, 512);
    axm.pushPose(hipH_Index, 6);
    waitOnServos(hipH_Index, hipH_Index + 6);

    //Move HipV to init pose
    SerialUSB.println("HipV positioning...");
    delay(5);
    axm.toggleTorques(hipV_cw_Index, 12, true);
    delay(5);
    initServoTable(hipV_cw_Index, hipV_cw_Index+6, 819);
    initServoTable(hipV_ccw_Index, hipV_ccw_Index+6, 205);
    axm.pushPose(hipV_cw_Index, 12);
    waitOnServos(hipV_cw_Index, hipV_cw_Index+12);

    //Move Knee to init pose
    SerialUSB.println("Knee positioning...");
    delay(5);
    axm.toggleTorques(knee_Index, 6, true);
    delay(5);
    initServoTable(knee_Index, knee_Index+6, 750);
    axm.pushPose(knee_Index, 6);
    waitOnServos(knee_Index, knee_Index+6);

//    //Move Foot to init pose
//    SerialUSB.println("Foot positioning...");
//    delay(5);
//    axm.toggleTorques(foot_Index, 6, true);
//    delay(5);
//    initServoTable(foot_Index, foot_Index+6, 512);
//    axm.pushPose(foot_Index, 6);
//    waitOnServos(foot_Index, foot_Index+6);
    
    for(int servo_Idx = 0; servo_Idx < NUMSERVOS; servo_Idx++){
        servoTable_Target[servo_Idx] = servoTable_Pose[servo_Idx];
    }
    
    delay(5);
    axm.speedLimits(0);
    delay(5);
    axm.toggleTorques(true);
//    axm.toggleTorques(hipH_Index, maxJoint_Index, true);
    delay(5);
//    axm.holdingMode();
    delay(5);
}

