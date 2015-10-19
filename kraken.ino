#include <dxl_devices.h>

#include "AX_Utilities.h"
#include "Physical_Config.h"

AxManager axm;
  
void setup() {
  axm.initAxM();
  SerialUSB.begin();
  delay(3000);
  SerialUSB.print("Current Voltage on right: ");
  SerialUSB.println(axm.voltage(0),2);
  SerialUSB.print("Current Voltage on left: ");
  SerialUSB.println(axm.voltage(1),2);
  oscillate(-50, hipV_cw_Index);
  oscillate(50, hipV_ccw_Index);
  oscillate(-50, knee_Index);
  axm.pushPose();
}

void oscillate(int swing, int jointOffset){
  int servo_idx;
  for(int i = 0; i < legCount; i++) {
      servo_idx = i + jointOffset;
      servoTable_Target[servo_idx + poseIndex_position] = 512 + swing;
  }
}

void loop() {
    int swing = 100;
    int direction = 1;
    while(true) {
        int target = direction*swing;
        int start = -direction*swing;
        oscillate(-direction*swing, hipV_cw_Index);
        oscillate(direction*swing, hipV_ccw_Index);
        oscillate(-direction*swing, knee_Index);

        axm.initInterpolate(millis());
        axm.setTimeToArrival(2000);
        while (!axm.interpolateFinished()) {
            delay(5);
            axm.interpolatePoses(millis());
        }

        direction = -direction;
        SerialUSB.print("Current Voltage on right: ");
        SerialUSB.println(axm.voltage(0),2);
        SerialUSB.print("Current Voltage on left: ");
        SerialUSB.println(axm.voltage(1),2);
    }
}
