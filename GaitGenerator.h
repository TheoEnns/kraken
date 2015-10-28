//
// Created by Theo on 10/27/2015.
//

#ifndef KRAKEN_GAITGENERATOR_H
#define KRAKEN_GAITGENERATOR_H

#include "Physical_Config.h"
#include "Physical_Types_And_Conversions.h"
#include "LegIKEngine.h"
#include "SerialComm.h"

#define REPORT_IK_FAILURE       1
//#define USE_GAIT_OVERSHOOT      1

#define DEF_STEP_UP_PERIOD      1.5f
#define DEF_STEP_HEIGHT         90.0f
#define DEF_GROUND_HEIGHT       -155.01f
#define DEF_CENTER_EXTENSION    175.05f
#define DEF_MAX_VEL_TRANS       35.0f
#define DEF_MAX_VEL_ROTATE      0.349066f

#define SERVO_INTERPOLATION_RATE 10
#define GAIT_INTERPOLATION_RATE 100

#ifdef USE_GAIT_OVERSHOOT
#define GAIT_INTERPOLATION_TARGET_TIME      (GAIT_INTERPOLATION_RATE+SERVO_INTERPOLATION_RATE)
#else
#define GAIT_INTERPOLATION_TARGET_TIME      (GAIT_INTERPOLATION_RATE)
#endif

enum WALK_STATE_T{
    walk_idle = 0,
    walk_starting,
    walk_stopping,
    walk_active,
    NUM_WALK_STATES,
};

typedef struct _Trajectory_Vector {
    float dr; //radians per sec (+ is counterclock)
    float dx; //units (I use mm) per sec, + is to the right
    float dy; //units (I use mm) per sec, + is foreword
} TRAJECTORY_2D;

typedef struct _Gait_Descriptor {
    int legModulus;
    float upStepPeriod;
    float stepHeight;
    float groundHeight;
    float maxTransVelocity;
    float maxRotVelocity;

    public:
        _Gait_Descriptor(int mod) {
            legModulus = mod;
            upStepPeriod = DEF_STEP_UP_PERIOD;
            stepHeight = DEF_STEP_HEIGHT;
            groundHeight = DEF_GROUND_HEIGHT;
            maxTransVelocity = DEF_MAX_VEL_TRANS;
            maxRotVelocity = DEF_MAX_VEL_ROTATE;
        }
} GAIT_DESCRIPTOR;

class GaitManager {
public:
    GaitManager(void);
    ~GaitManager();

//--------------//
//  Gait interpolations
    void updateTargetTime(unsigned long newTime);
    void interpolateNextWalk(int legIndx);
    int getInterpolationTime();
    void pushIKtoTarget();

//--------------//
//  Gait transitions
    void switchGaitModulus(int newModulus);
    bool setNextTrajectory(dx,dy,dr);

private:
    WALK_STATE_T cWalkState;
    WALK_STATE_T fWalkState;
    GAIT_DESCRIPTOR cGait;
    GAIT_DESCRIPTOR fGait;
    TRAJECTORY_2D cTrajectory;
    TRAJECTORY_2D fTrajectory;

    unsigned long targetTime;
    unsigned long timeOffset;
    int phase;
    float fraction;
    bool midStepToggle;

    COORD3D legCenters[CNT_LEGS];

//--------------//
//  Internal Gait mechanisms
    void setTarget(int legIndx);
    void updatePhase(unsigned long timer);
    void setTarget_Start(int legIndx);
    void setTarget_MidStep(int legIndx);
    void setTarget_End(int legIndx);
    void setTarget_Idle(int legIndx);

//--------------//
//  Leg Placement
    COORD_3D deltaFromCenter(float tdx, float tdy, float tdr, float tdz);
    void generateCenters();

};


GaitManager::GaitManager(void){
    cWalkState = walk_idle;
    fWalkState = walk_idle;
    cGait = GAIT_DESCRIPTOR(2);
    fGait = GAIT_DESCRIPTOR(2);
    cTrajectory = TRAJECTORY_2D{0f,0f,0f};
    fTrajectory = TRAJECTORY_2D{0f,0f,0f};

    targetTime = 0;
    timeOffset = 0;
    phase = 0;
    fraction = 0;
    midStepToggle = false;

    generateCenters();
}

GaitManager::~GaitManager(){

}

//--------------//
//  Gait interpolations
void GaitManager::updateTargetTime(unsigned long newTime){
    targetTime = newTime + GAIT_INTERPOLATION_TARGET_TIME;
    updatePhase();
}

IK_Error_T GaitManager::interpolateNextWalk(int legIndx){
    setTarget(legIndx);
    IK_Error_T error = legIK.leg3DOF_Inverse(legIndx);
#ifdef REPORT_IK_FAILURE
    if (error > ik_Success) {
        serialComms.startJsonMsg();
        serialComms.printVar("IK_Error", (int)(error));
        serialComms.printVar("Leg", legIndx);
        serialComms.endJsonMsg();
    }
#endif
    return error;
}

int GaitManager::getInterpolationTime(){
    return GAIT_INTERPOLATION_TARGET_TIME;
}

void GaitManager::pushIKtoTarget() {
    for(int legIndx = 0; legIndx < CNT_LEGS; legIndx++) {
        if (legIK.isValidEffector()) {
            servoTable_Target[hipH_Index + legIndx] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipH(legIndx), true), 0, 1023);
            servoTable_Target[hipV_cw_Index + legIndx] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndx), true), 0, 1023);
            servoTable_Target[hipV_ccw_Index + legIndx] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndx), false), 0, 1023);
            servoTable_Target[knee_Index + legIndx] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_knee(legIndx), false), 0, 1023);
            servoTable_Target[foot_Index + legIndx] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_ankle(legIndx), false), 0, 1023);
        }
    }
}

//--------------//
//  Gait transitions
void GaitManager::switchGaitModulus(int newModulus){
    fGait = GAIT_DESCRIPTOR(newModulus);
    if(cWalkState=walk_idle) {
        cGait = fGait;
    }
}

bool GaitManager::setNextTrajectory(dx,dy,dr){
    float vel = sqrt(dx*dx+dy*dy);
    if (vel > cGait.maxTransVelocity)
        return false;
    if (fabs(dr) > cGait.maxRotVelocity)
        return false;
    fTrajectory.dx = dx;
    fTrajectory.dy = dy;
    fTrajectory.dr = dr;

    if(vel == 0 && dr == 0) {
        if(cWalkState == walk_idle) {
            fWalkState = walk_idle;
        } else {
            fWalkState = walk_end;
        }
    } else {
        if(cWalkState == walk_idle) {
            cGait = fGait;
            cTrajectory = fTrajectory;
            cWalkState = walk_starting;
            timeOffset = targetTime;
            fraction = 0;
            phase = 0;
        }
    }
}

//--------------//
//  Internal Gait mechanisms
void GaitManager::setTarget(int legIndx){
    switch(cWalkState){
        case walk_starting:
            setTarget_Start(legIndx);
            break;
        case walk_active:
            setTarget_MidStep(legIndx);
            break;
        case walk_stopping:
            setTarget_End(legIndx);
            break;
        case walk_idle:
            setTarget_idle(legIndx);
            break;
    }
}

void GaitManager::updatePhase(){
    fraction = ((float)(timer - targetTime))/(cGait.upStepPeriod*cGait.legModulus);

    //phase rollover - increment the offset, time for switching walk state
    if(fraction > cGait.legModulus) {
        timeOffset = timer;
        cWalkState = fWalkState;
        fraction = 0;
        phase = 0;
    } else {
        phase = (int) floor(fraction);
        fraction = fraction - phase;
    }

    if(cWalkState == walk_active) {
        if ((fraction > 0.5) && (!midStepToggle)) {
            midStepToggle = true;
            if (cGait.legModulus == 2) {
                //Switch to new trajectory is safe in legMod == 2 when at half step
                cTrajectory = fTrajectory;
                memset(fTrajectory, 0, sizeof(_LEG_POSE));)
            }
        } else {
            midStepToggle = false;
            if (cGait.legModulus == 3) {
                //Switch to new trajectory is safe in legMod == 3 when at full step
                cTrajectory = fTrajectory;
            }
        }
    }
}

void GaitManager::setTarget_Start(int legIndx) {
    int mod = (legIndx % cGait.legModulus);
    int legModDelta = (mod - phase)%cGait.legModulus;
    if (legModDelta==0) {
        if(mod == 0) {
            legIK.effector(legIndx, deltaFromCenter(
                    cTrajectory.dx / 4 - cos(M_PI * fraction) * cTrajectory.dx / 4,
                    cTrajectory.dy / 4 - cos(M_PI * fraction) * cTrajectory.dy / 4,
                    cTrajectory.dr / 4 - cos(M_PI * fraction) * cTrajectory.dr / 4,
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else if (mod == (cGait.legModulus-1)){
            legIK.effector(legIndx, deltaFromCenter(
                    -(cTrajectory.dx / 4 - cos(M_PI * fraction) * cTrajectory.dx / 4),
                    -(cTrajectory.dy / 4 - cos(M_PI * fraction) * cTrajectory.dy / 4),
                    -(cTrajectory.dr / 4 - cos(M_PI * fraction) * cTrajectory.dr / 4),
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else{
            legIK.effector(legIndx, legCenters[legIndx]);
        }
    }
}

void GaitManager::setTarget_MidStep(int legIndx) {
    int mod = (legIndx % cGait.legModulus);
    int legModDelta = (mod - phase)%cGait.legModulus;
    if (legModDelta==0) {
        legIK.effector(legIndx, deltaFromCenter(
                (cos(M_PI * fraction) * cTrajectory.dx / 2),
                (cos(M_PI * fraction) * cTrajectory.dy / 2),
                (cos(M_PI * fraction) * cTrajectory.dr / 2),
                cGait.stepHeight * fabs(sin(M_PI * fraction))
        ));
    } else {// if (mod == (cGait.legModulus-1)){
        float dualFraction = fraction/(legModulus-1);
        legIK.effector(legIndx, deltaFromCenter(
                -(cos(M_PI * dualFraction) * cTrajectory.dx / 2),
                -(cos(M_PI * dualFraction) * cTrajectory.dy / 2),
                -(cos(M_PI * dualFraction) * cTrajectory.dr / 2),
                0
        ));
    }
}

void GaitManager::setTarget_End(int legIndx) {
    int mod = (legIndx % cGait.legModulus);
    int legModDelta = (mod - phase)%cGait.legModulus;
    if (legModDelta==0) {
        if(mod == 0) {
            legIK.effector(legIndx, deltaFromCenter(
                    cTrajectory.dx / 4 + cos(M_PI * fraction) * cTrajectory.dx / 4,
                    cTrajectory.dy / 4 + cos(M_PI * fraction) * cTrajectory.dy / 4,
                    cTrajectory.dr / 4 + cos(M_PI * fraction) * cTrajectory.dr / 4,
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else if (mod == (cGait.legModulus-1)){
            legIK.effector(legIndx, deltaFromCenter(
                    -(cTrajectory.dx / 4 + cos(M_PI * fraction) * cTrajectory.dx / 4),
                    -(cTrajectory.dy / 4 + cos(M_PI * fraction) * cTrajectory.dy / 4),
                    -(cTrajectory.dr / 4 + cos(M_PI * fraction) * cTrajectory.dr / 4),
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else{
            legIK.effector(legIndx, legCenters[legIndx]);
        }
    }
}

void GaitManager::setTarget_Idle(int legIndx) {
    legIK.effector(indx, legCenters[legIndx]);
}

//--------------//
//  Leg Placement
COORD_3D GaitManager::deltaFromCenter(float tdx, float tdy, float tdr, float tdz){
    COORD3D newPoint;
    newPoint.x = cos(tdr)*legCenters[legIndx].x - sin(tdr)*legCenters[legIndx].y + tdx;
    newPoint.y = sin(tdr)*legCenters[legIndx].x + cos(tdr)*legCenters[legIndx].y + tdy;
    newPoint.y = legCenters[legIndx].z + tdz;
    return newPoint;
}

COORD_3D GaitManager::generateCenters() {
    for(int legIndx = 0; legIndx<CNT_LEGS; legIndx++) {
        legCenters[legIndx] = COORD_3D{
                hipSegment[legIndx].segVect.x + DEF_CENTER_EXTENSION * cos(hipSegment[legIndx].effectorAngle),
                hipSegment[legIndx].segVect.y + DEF_CENTER_EXTENSION * sin(hipSegment[legIndx].effectorAngle),
                cGait.groundHeight
            };
    }
}

GaitManager gaitGen;

#endif //KRAKEN_GAITGENERATOR_H
