#include <dxl_devices.h>

#include "AX_Utilities.h"
#include "Physical_Config.h"

AxManager axm;
  
void setup() {
  SerialUSB.begin();
  axm.initAxM(&SerialUSB);
  delay(3000);
  axm.toggleTorques(hipH_Index, 30, true);
  axm.torqueRange(hipH_Index, 24, 1023);
  axm.torqueRange(foot_Index, 6, 300);
  SerialUSB.print("Current Voltage on right: ");
  SerialUSB.println(axm.voltage(0),2);
  SerialUSB.print("Current Voltage on left: ");
  SerialUSB.println(axm.voltage(1),2);
  
  oscillate(-100, knee_Index);
  oscillate(-100, hipV_cw_Index);
  oscillate(100, hipV_ccw_Index);
  axm.pushPose();
}

void oscillate(int swing, int jointOffset){
  int servo_idx;
  for(int i = 0; i < legCount; i++) {
      servo_idx = i + jointOffset;
      servoTable_Target[servo_idx] = 512 + swing;
  }
}

void loop() {
    int swing = 100;
    int direction = 1;
//  axm.freeMoveMode(0x00FC0000); // knee only
    axm.freeMoveMode(0x3FFFFFFF); // knee and hips
    while(true) {
        int target = direction*swing;
        int start = -direction*swing;
        
        oscillate(direction*swing/2, foot_Index);
        oscillate(direction*swing, knee_Index);
        oscillate(direction*swing, hipV_cw_Index);
        oscillate(-direction*swing, hipV_ccw_Index);
        oscillate(direction*swing, hipH_Index);

        axm.initInterpolate(millis());
        axm.setTimeToArrival(2000);
        while (axm.interpolateFinished()) {
            delay(10);
            axm.interpolatePoses(millis());
        }

        direction = -direction;
        SerialUSB.print("Current Voltage on right: ");
        SerialUSB.println(axm.voltage(0),2);
        SerialUSB.print("Current Voltage on left: ");
        SerialUSB.println(axm.voltage(1),2);
    }
}
