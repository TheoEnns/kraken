//
// Created by Theo on 10/18/2015.
//

#ifndef KRAKEN_PHYSICAL_CONFIG_H_H
#define KRAKEN_PHYSICAL_CONFIG_H_H

#define NUM_HEAD_SERVOS     0
#define NUM_SERVOS_PER_LEG  5
#define CNT_LEGS            6
#define NUMSERVOS           (NUM_SERVOS_PER_LEG*CNT_LEGS + NUM_HEAD_SERVOS)

#define LEFT_BATTERY_SERVO_ID       2
#define RIGHT_BATTERY_SERVO_ID      1
#define VOLTAGE_IDLE_POLL_RATE      2000 //ms
#define VOLTAGE_SHUTDOWN_VAL        9.5 //volt

#define MAX_SERVO_TORQUE            1023 //Preset on the servo's flash, AXM_MAX_TORQUE_L = 14

//Start with the front right leg going counter-clockwise
//              LegIndex=1      LegIndex=0
//          LegIndex=2              LegIndex=5
//              LegIndex=3      LegIndex=4
enum LegIndex{
    legIndex_ForeRight = 0,
    legIndex_ForeLeft,
    legIndex_CenterLeft,
    legIndex_BackLeft,
    legIndex_BackRight,
    legIndex_CenterRight,
    legCount
};

//First row is hip horizontal rotation
//Second row is hip vertical clockwise
//third row is hip vertical counter clockwise
//fourth row is knee
//fifth row is foot
//sixth row is an optional head/turret
enum JointIndex{
    hipH_Index = 0,
    hipV_cw_Index = 6,
    hipV_ccw_Index = 12,
    knee_Index = 18,
    foot_Index = 24,
    head_Index = 30,
    maxJoint_Index = NUMSERVOS
};

word      servoTable_Pose[NUMSERVOS];
word      servoTable_Target[NUMSERVOS];
word      servoTable_Torque[NUMSERVOS];

static const byte servoTable_ID[NUMSERVOS] = {
    19, 20, 21, 22, 23, 24,     //hipH
    7,  8,  9,  10, 11, 12,     //hipV_cw
    13, 14, 15, 16, 17, 18,     //hipV_ccw
    1,  2,  3,  4,  5,  6,      //knee
    101,102,103,104,105,106,    //foot
    //head
};

#endif //KRAKEN_PHYSICAL_CONFIG_H_H
