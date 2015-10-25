#include <dxl_devices.h>

#include "AX_Utilities.h"
#include "Physical_Config.h"

AxManager axm;

#define NUM_SERIALS     2
Stream * mSerial[NUM_SERIALS] = {
//        &Serial1,
        &Serial2,
        &SerialUSB
};
bool isJsonStart = false;

void initSerialOutput(){
    for(int i=0;i<NUM_SERIALS;i++) {
        if(mSerial[i] == &SerialUSB)
            mSerial[i]->begin();
        else
            mSerial[i]->begin(115200);
    }
}

void startJsonMsg(){
    for (int i = 0; i < NUM_SERIALS; i++) {
        mSerial[i].print("{");
    }
    isJsonStart = true;
}

void printVar_F(char* varName, float value) {
    for (int i = 0; i < NUM_SERIALS; i++) {
        if(!isJsonStart)
            mSerial[i].print(",");
        mSerial[i].print("\"");
        mSerial[i].print(varName);
        mSerial[i].print("\":");
        mSerial[i].print(varName, 4);
    }
    isJsonStart = false;
}

void printVar_D(char* varName, int value){
    for (int i = 0; i < NUM_SERIALS; i++) {
        if(!isJsonStart)
            mSerial[i].print(",");
        mSerial[i].print("\"");
        mSerial[i].print(varName);
        mSerial[i].print("\": ");
        mSerial[i].print(varName, DEC);
    }
    isJsonStart = false;
}

void endJsonMsg(){
    for (int i = 0; i < NUM_SERIALS; i++) {
        mSerial[i].print("}");
    }
    isJsonStart = false;
}

void report_Voltage(){
    startJsonMsg();
    printVar_F("Right Battery Voltage", axm.voltage(RIGHT_BATTERY_SERVO_ID));
    printVar_F("Left Battery Voltage", axm.voltage(LEFT_BATTERY_SERVO_ID));
    endJsonMsg();
}
  
void setup() {
    initSerialOutput();
    axm.initAxM();
    delay(3000);

    axm.toggleTorques(hipH_Index, maxJoint_Index, true);
    delay(5);
    axm.torqueRange(hipH_Index, foot_Index, 900);
    delay(5);
    axm.torqueRange(foot_Index, maxJoint_Index, 300);
    delay(5);
    axm.freeMoveMode();
    delay(5);

    axm.getPositions();
    delay(5);

    initServos(hipH_Index, hipH_Index+6, 512);
    axm.pushPose(hipH_Index, 6);
    delay(100);
    axm.waitOnServos(hipH_Index, hipH_Index+6);

    initServos(hipV_cw_Index, hipV_cw_Index+6, 205);
    initServos(hipV_ccw_Index, hipV_ccw_Index+6, 819);
    axm.pushPose(hipV_cw_Index, 12);
    delay(100);
    waitOnServos(hipV_cw_Index, hipV_cw_Index+12);

    initServos(knee_Index, knee_Index+6, 0);
    axm.pushPose(knee_Index, 6);
    delay(100);
    waitOnServos(knee_Index, knee_Index+6);

    initServos(foot_Index, foot_Index+6, 512);
    axm.pushPose(foot_Index, 6);
    delay(100);
    waitOnServos(foot_Index, foot_Index+6);
}


void initServos(int startIndx, int endIndx, word pos){
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        axm.position(servo_Idx,pos,false);
    }
}

void waitOnServos(int startIndx, int endIndx){
    for (int servo_Idx = startIndx; servo_Idx < endIndx; servo_Idx++){
        while (axm.is_moving(servo_Idx)){
            delay(5);
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

}
