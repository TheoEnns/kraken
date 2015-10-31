//
// Created by Theo on 10/22/2015.
//

#ifndef KRAKEN_LEGIKENGINE_H
#define KRAKEN_LEGIKENGINE_H

#include "Physical_Config.h"
#include "Physical_Types_And_Conversions.h"

//#define DEBUG_IK 1
#ifdef DEBUG_IK  
  #include "SerialComm.h"
  #define IK_DEBUG_START                serialComms.startJsonMsg(true);
  #define IK_DEBUG_PRINT_VAL(name,val)  serialComms.printVar(name,val,true);
  #define IK_DEBUG_END                  serialComms.endJsonMsg(true);
#else
  #define IK_DEBUG_START 
  #define IK_DEBUG_PRINT_VAL(name,val) 
  #define IK_DEBUG_END 
#endif

#define vX      0
#define vY      1
#define vZ      2

typedef struct _LEG_POSE {
    COORD3D effector;
    float liftHeight;
    float groundHeight;

    //Derived Coords and Angles
    COORD3D centerOfMovement;
    float boundaryRadius;

    COORD3D foot;
    COORD3D ankle;
    COORD3D knee;
    COORD3D hipV;
    COORD3D hipH;

    float angle_ankle;
    float angle_knee;
    float angle_hipV;
    float angle_hipH;

    bool isValidEffector;
    bool isValidForeward;
} LEG_POSE;

enum IK_Error_T{
    ik_Success = 0,
    ik_hipH_MinViolation,
    ik_hipH_MaxViolation,
    ik_extension_Violation,
    ik_knee_NaN,//4
    ik_knee_MinViolation,
    ik_knee_MaxViolation,
    ik_hipV_LegTilt_NaN,
    ik_hipV_MinViolation,
    ik_hipV_MaxViolation,//9
    ik_ankle_MaxViolation,
    ik_ankle_MinViolation,
    IK_ERROR_COUNT,
};

class LegIKEngine {
public:
    LegIKEngine(void);
    ~LegIKEngine();

    //Core Functions
//    bool leg3DOF_Foreward(LegIndex legIndex);
    IK_Error_T leg3DOF_Inverse(LegIndex legIndex);

    //Setter
    void effector(LegIndex legIndex, COORD3D effector);

    //Coordinate getters
    COORD3D effector(LegIndex legIndex);
    COORD3D ankle(LegIndex legIndex); //unused, not impelemented
    COORD3D knee(LegIndex legIndex);  //unused, not impelemented
    COORD3D hipV(LegIndex legIndex);  //unused, not impelemented
    COORD3D hipH(LegIndex legIndex);  //unused, not impelemented

    //Angle getters
    float angle_ankle(LegIndex legIndex);
    float angle_knee(LegIndex legIndex);
    float angle_hipV(LegIndex legIndex);
    float angle_hipH(LegIndex legIndex);

    //IK validity
    bool isValidForeward(LegIndex legIndex);
    bool isValidEffector(LegIndex legIndex);

    //Step Parameters
    void setStepParameters(LegIndex legIndex, float liftHeight, float groundHeight);

private:
    LEG_POSE ik_legs[CNT_LEGS];
    float segLengths[NUM_JOINTS];
    float ik_offsetAngles[NUM_JOINTS];
};

LegIKEngine::LegIKEngine(void) {
    memset(ik_legs, 0, CNT_LEGS * sizeof(LEG_POSE));
  
//    for(int leg_Index = 0; leg_Index < CNT_LEGS; leg_Index++){
//        ik_legs[leg_Index].isValidEffector = false;
//        ik_legs[leg_Index].isValidForeward = false;
//    }

    for(int segIndex = 0; segIndex < NUM_SEGMENTS; segIndex++){
        segLengths[segIndex] = vLength(segmentTemplates[segIndex]->segVect);
    }

    ik_offsetAngles[coxa_Type]  = asin(coxaSegment.segVect.x/coxaSegment.segVect.y);
    ik_offsetAngles[femur_Type] = asin(femurSegment.segVect.x/femurSegment.segVect.y);
    ik_offsetAngles[tibia_Type] = asin(tibiaSegment.segVect.x/tibiaSegment.segVect.y);
    ik_offsetAngles[foot_Type]  = asin(footSegment.segVect.x/footSegment.segVect.y);
}

LegIKEngine::~LegIKEngine() {

}

//--------------//
//Core Functions

//bool LegIKEngine::leg3DOF_Foreward(LegIndex lgIndx) {
//    LEG_POSE * thisLeg = &(ik_legs[lgIndx]);
//    COORD3D legPlaneNormal = {0,-1,0};
//
//    thisLeg->hipV = {coxaSegment.segVect.x, coxaSegment.segVect.z, coxaSegment.segVect.y};
//    thisLeg->knee = {femurSegment.segVect.x, femurSegment.segVect.z, femurSegment.segVect.y};
//    thisLeg->ankle = {tibiaSegment.segVect.x, tibiaSegment.segVect.z, tibiaSegment.segVect.y};
//    thisLeg->foot = {footSegment.segVect.x, footSegment.segVect.z, footSegment.segVect.y};
//
//    thisLeg->hipH = hipSegment[lgIndx].segVect;
//    float angle = thisLeg->angle_hipH + hipSegment[lgIndx].effectorAngle;
//    thisLeg->hipV = vRotate(thisLeg->hipV, angle, {0,0,1});
//    thisLeg->knee = vRotate(thisLeg->knee, angle, {0,0,1});
//    thisLeg->ankle = vRotate(thisLeg->ankle, angle, {0,0,1});
//    legPlaneNormal = vRotate(legPlaneNormal, angle, {0,0,1});
//    thisLeg->hipV = vAdd(thisLeg->hipH, thisLeg->hipV);
//
//    angle = thisLeg->angle_hipV;
//    thisLeg->knee = vRotate(thisLeg->knee, angle, legPlaneNormal);
//    thisLeg->knee = vAdd(thisLeg->hipV, thisLeg->knee);
//
//    angle = thisLeg->angle_knee + angle;
//    thisLeg->ankle = vRotate(thisLeg->ankle, angle, legPlaneNormal);
//    thisLeg->ankle = vAdd(thisLeg->knee, thisLeg->ankle);
//
//    angle = thisLeg->angle_ankle + angle;
//    thisLeg->foot = vRotate(thisLeg->foot, angle, legPlaneNormal);
//    thisLeg->foot = vAdd(thisLeg->foot, thisLeg->ankle);
//
//    return thisLeg->isValidForeward = true;
//}

IK_Error_T LegIKEngine::leg3DOF_Inverse(LegIndex lgIndx) {
    IK_Error_T error = ik_Success;
    LEG_POSE * thisLeg = (ik_legs+lgIndx);
    float raw_angle;
    thisLeg->isValidForeward = false;
    IK_DEBUG_START
    IK_DEBUG_PRINT_VAL("IK For Leg",lgIndx)

    //Get HipH
    raw_angle = atan2(-thisLeg->effector.x*sin(hipSegment[lgIndx].effectorAngle)
                        + thisLeg->effector.y*cos(hipSegment[lgIndx].effectorAngle),
                      thisLeg->effector.x*cos(hipSegment[lgIndx].effectorAngle)
                        + thisLeg->effector.y*sin(hipSegment[lgIndx].effectorAngle));
    thisLeg->angle_hipH = raw_angle;
//    raw_angle = atan2(thisLeg->effector.y, thisLeg->effector.x);
//    if(raw_angle>=0){
//        thisLeg->angle_hipH = raw_angle - hipSegment[lgIndx].effectorAngle;
//    } else {
//        thisLeg->angle_hipH = raw_angle + 2*M_PI - hipSegment[lgIndx].effectorAngle;
//    }
    IK_DEBUG_PRINT_VAL("hipH",thisLeg->angle_hipH)
    if (thisLeg->angle_hipH > hipSegment[lgIndx].maxAngle) {
        thisLeg->isValidEffector = false;
        error = ik_hipH_MaxViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    } else if (thisLeg->angle_hipH < hipSegment[lgIndx].minAngle) {
        thisLeg->isValidEffector = false;
        error = ik_hipH_MinViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }
        
    //Rotate to leg plane, get extension
    COORD3D v_hip2target = vSubract(thisLeg->effector, hipSegment[lgIndx].segVect);
    v_hip2target.x = sqrt( cSquare(thisLeg->effector.x - hipSegment[lgIndx].segVect.x)
                         + cSquare(thisLeg->effector.y - hipSegment[lgIndx].segVect.y))
                         - segLengths[coxa_Type];
    v_hip2target.y = thisLeg->effector.z + footSegment.segVect.x;

    //Get leg extension
    float extension = sqrt( cSquare(v_hip2target.x) + cSquare(v_hip2target.y));
//    IK_DEBUG_PRINT_VAL("extension",extension)
    if (extension <= 0.0001){ //Allow for float precision error
        thisLeg->isValidEffector = false;
        error = ik_extension_Violation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }

    //Find knee angle
    raw_angle = getOpposingAngle(extension, segLengths[femur_Type], segLengths[tibia_Type]);
    if ( raw_angle == -1){ 
        //opposing returns NaN = -1
        //  The openCM compiler uses a not really
        //  float type for float which does not have a NaN.
        thisLeg->isValidEffector = false;
        error = ik_knee_NaN;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }
    thisLeg->angle_knee = raw_angle  - M_PI_2 - tibiaSegment.effectorAngle;
    IK_DEBUG_PRINT_VAL("knee",thisLeg->angle_knee)
    if (thisLeg->angle_knee > tibiaSegment.maxAngle) {
        thisLeg->isValidEffector = false;
        error = ik_knee_MaxViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    } else if (thisLeg->angle_knee < tibiaSegment.minAngle) {
        thisLeg->isValidEffector = false;
        error = ik_knee_MinViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }

    
    //Use law of cosines to get leg tilt
    float sN = cSquare(segLengths[femur_Type]) - cSquare(segLengths[tibia_Type]) + cSquare(extension); //Use law of cosines to get leg tilt
    float sD = 2*segLengths[femur_Type]*extension;
    if ( fabs(sN) > fabs(sD) ){
        thisLeg->isValidEffector = false;
        error = ik_hipV_LegTilt_NaN;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }
    float legAngleTilt = acos(sN/(sD)); //Use law of cosines to get leg tilt

    //Get hipV
    raw_angle = legAngleTilt + atan2(v_hip2target.y,v_hip2target.x);
    IK_DEBUG_PRINT_VAL("hipV",raw_angle)
    if (raw_angle > femurSegment.maxAngle) {
        thisLeg->isValidEffector = false;
        error = ik_hipV_MaxViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    } else if (raw_angle < femurSegment.minAngle) {
        thisLeg->isValidEffector = false;
        error = ik_hipV_MinViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }
    thisLeg->angle_hipV = raw_angle;

    //Get ankle
    raw_angle = - thisLeg->angle_hipV - thisLeg->angle_knee;
    IK_DEBUG_PRINT_VAL("ankle",raw_angle)
    if (raw_angle > footSegment.maxAngle) {
        thisLeg->isValidEffector = false;
        error = ik_ankle_MaxViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    } else if (raw_angle < footSegment.minAngle) {
        thisLeg->isValidEffector = false;
        error = ik_ankle_MinViolation;
        IK_DEBUG_PRINT_VAL("ERROR",error)
        IK_DEBUG_END
        return error;
    }
    thisLeg->angle_ankle = raw_angle;

    //Record Success
    thisLeg->isValidEffector = true;
    IK_DEBUG_PRINT_VAL("ERROR",error)
    IK_DEBUG_END
    return error;
}

//--------------//
//Setter
void LegIKEngine::effector(LegIndex legIndex, COORD3D effector) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    thisLeg->effector = effector;
    thisLeg->isValidEffector = false; //leg3DOF_Inverse validates, this effector will not push until validated
    thisLeg->isValidForeward = false; //leg3DOF_Foreward validates, this effector will not push until validated
}


//--------------//
//Coordinate getters
COORD3D LegIKEngine::effector(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->effector;
}

COORD3D LegIKEngine::ankle(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->ankle;
}

COORD3D LegIKEngine::knee(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->knee;
}

COORD3D LegIKEngine::hipV(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->hipV;
}

COORD3D LegIKEngine::hipH(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->hipH;
}

//--------------//
//Angle getters
float LegIKEngine::angle_ankle(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->angle_ankle;
}

float LegIKEngine::angle_knee(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->angle_knee;
}

float LegIKEngine::angle_hipV(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->angle_hipV;
}

float LegIKEngine::angle_hipH(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->angle_hipH;
}

//--------------//
//IK validity
bool LegIKEngine::isValidForeward(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->isValidForeward;
}

bool LegIKEngine::isValidEffector(LegIndex legIndex) {
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    return thisLeg->isValidEffector;
}

//--------------//
//Step Parameters
void LegIKEngine::setStepParameters(LegIndex legIndex, float liftHeight, float groundHeight){
    LEG_POSE * thisLeg = &(ik_legs[legIndex]);
    thisLeg->liftHeight = liftHeight;
    thisLeg->groundHeight = groundHeight;
}

extern LegIKEngine legIK;

#endif //KRAKEN_LEGIKENGINE_H

