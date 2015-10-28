//
// Created by Theo on 10/18/2015.
//

#ifndef KRAKEN_PHYSICAL_CONFIG_H_H
#define KRAKEN_PHYSICAL_CONFIG_H_H

#include "Physical_Types_And_Conversions.h"

//--------------//
//  Servo Identification

#define NUM_HEAD_SERVOS     0
#define NUM_SERVOS_PER_LEG  5
#define CNT_LEGS            6
#define NUMSERVOS           (NUM_SERVOS_PER_LEG*CNT_LEGS + NUM_HEAD_SERVOS)

//Servos for polling battery values
#define LEFT_BATTERY_SERVO_ID       2
#define RIGHT_BATTERY_SERVO_ID      1

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
//    head_Index = 30,
    maxJoint_Index = NUMSERVOS
};

enum JointTypes{
    hipH_Type = 0,
    hipV_Type,
    knee_Type,
    ankle_Type,
    NUM_JOINTS,
};

enum SegmentTypes{
    coxa_Type = 0,
    femur_Type,
    tibia_Type,
    foot_Type,
    NUM_SEGMENTS,
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
//    , ,                            //head
};

//--------------//
//  Physical Vectors
//      (I am using mm, but this is unit agnostic)

//Hip Vector: Vector defining the robots center bottom to the hip horizontal vector
//X-Axis: The direction to the right of the robot
//Y-Axis: The forwards direction for the robot
//Z-Axis: The upwards direction for the robot
//effectorAngle is the rotation of the hip relative to the x-axis counter-clockwise. min and max angles define
//      the range of the leg relative to an atan2 from hip to end effector.
//              LegIndex=1      LegIndex=0
//          LegIndex=2              LegIndex=5
//              LegIndex=3      LegIndex=4
const SEGMENT3D hipSegment[CNT_LEGS] =
    {
        {{ 47.625f, 82.489f, 0.0f}, (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,   (60.0f)*M_PI/180.0f  },
        {{-47.625f, 82.489f, 0.0f}, (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,   (120.0f)*M_PI/180.0f },
        {{-95.25f,  0.0f,    0.0f}, (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,   (180.0f)*M_PI/180.0f },
        {{-47.625f,-82.489f, 0.0f}, (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,   (240.0f)*M_PI/180.0f },
        {{ 47.625f,-82.489f, 0.0f}, (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,   (300.0f)*M_PI/180.0f },
        {{ 95.25f,  0.0f,    0.0f}, (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,   (0.0f)*M_PI/180.0f   },
    };

//Coxa Vector: Vector defining the hip horizontal joint to the hip vertical vector
//X-Axis: The direction the Servo points at center (512)
//Y-Axis: The upwards direction for the robot
//Z-Axis: The clockwise direction about the robot for the leg
const SEGMENT3D coxaSegment =
        {{ 53.505f, 0.0f, 0.0f},    (30.0f)*M_PI/180.0f,   (-30.0f)*M_PI/180.0f,    0.0f};
const JOINT3D coxaStructure[CNT_LEGS] =
    {
        //leg 0 horizontal hip -> coxa
        {&coxaSegment, false, 0, true, -1, false},
        //leg 1 horizontal hip -> coxa
        {&coxaSegment, false, 1, true, -1, false},
        //leg 2 horizontal hip -> coxa
        {&coxaSegment, false, 2, true, -1, false},
        //leg 3 horizontal hip -> coxa
        {&coxaSegment, false, 3, true, -1, false},
        //leg 4 horizontal hip -> coxa
        {&coxaSegment, false, 4, true, -1, false},
        //leg 5 horizontal hip -> coxa
        {&coxaSegment, false, 5, true, -1, false},
    };

//Femur Vector: Vector defining the hip vertical joint to the knee vector
//X-Axis: The direction the Servo points at center (512)
//Y-Axis: The upwards direction for the robot
//Z-Axis: The clockwise direction about the robot for the leg
const SEGMENT3D femurSegment =
        {{ 121.541f, 0.0f, 0.0f},    (105.0f)*M_PI/180.0f,   (-75.0f)*M_PI/180.0f,    0.0f};
const JOINT3D femurStructure[CNT_LEGS] =
    {
        //leg 0  hip vertical -> femur
        {&femurSegment, true, 6, false,  12, true},
        //leg 1  hip vertical -> femur
        {&femurSegment, true, 7, false,  13, true},
        //leg 2  hip vertical -> femur
        {&femurSegment, true, 8, false,  14, true},
        //leg 3  hip vertical -> femur
        {&femurSegment, true, 9, false,  15, true},
        //leg 4  hip vertical -> femur
        {&femurSegment, true, 10, false, 16, true},
        //leg 5  hip vertical -> femur
        {&femurSegment, true, 11, false, 17, true},
   };

//Tibia Vector: Vector defining the knee joint to the foot/foot joint
//X-Axis: The direction the Servo points at center (512)
//Y-Axis: The upwards direction for the robot
//Z-Axis: The clockwise direction about the robot for the leg
const SEGMENT3D tibiaSegment =
        {{ 34.429f, -131.0f, 0.0f},    (90.0f)*M_PI/180.0f,   (-90.0f)*M_PI/180.0f,    0.257004f};
const JOINT3D tibiaStructure[CNT_LEGS] =
    {
        //leg 0  knee -> tibia
        {&tibiaSegment, false, 18, true, -1, false},
        //leg 1  knee -> tibia
        {&tibiaSegment, false, 19, true, -1, false},
        //leg 2  knee -> tibia
        {&tibiaSegment, false, 20, true, -1, false},
        //leg 3  knee -> tibia
        {&tibiaSegment, false, 21, true, -1, false},
        //leg 4  knee -> tibia
        {&tibiaSegment, false, 22, true, -1, false},
        //leg 5  knee -> tibia
        {&tibiaSegment, false, 23, true, -1, false},
    };

//Foot Vector: Vector defining the knee joint to the foot/foot joint
//X-Axis: height of the foot from ankle to ground
//Y-Axis: (unused right now) length of the foot parallel to ground (heel to toes)
//Z-Axis: (unused right now) width of the foot
const SEGMENT3D footSegment =
        {{ 32.25f, 47.439f, 52.16f},    (90.0f)*M_PI/180.0f,   (-90.0f)*M_PI/180.0f,    0.0f};
const JOINT3D footStructure[CNT_LEGS] =
    {
        //leg 0  foot -> ground
        {&footSegment, false, 24, true, -1, false},
        //leg 1  foot -> ground
        {&footSegment, false, 25, true, -1, false},
        //leg 2  foot -> ground
        {&footSegment, false, 26, true, -1, false},
        //leg 3  foot -> ground
        {&footSegment, false, 27, true, -1, false},
        //leg 4  foot -> ground
        {&footSegment, false, 28, true, -1, false},
        //leg 5  foot -> ground
        {&footSegment, false, 29, true, -1, false},
    };

const SEGMENT3D* segmentTemplates[NUM_SEGMENTS] =
    {  //Corresponds to SegmentTypes
        &coxaSegment,
        &femurSegment,
        &tibiaSegment,
        &footSegment,
    };

#endif //KRAKEN_PHYSICAL_CONFIG_H_H
