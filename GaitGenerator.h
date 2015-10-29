//
// Created by Theo on 10/27/2015.
//

#ifndef KRAKEN_GAITGENERATOR_H
#define KRAKEN_GAITGENERATOR_H

#include "Physical_Types_And_Conversions.h"
#include "Physical_Config.h"
#include "LegIKEngine.h"
#include "AX_Utilities.h"
#include "SerialComm.h"

#define REPORT_IK_FAILURE       1
//#define USE_GAIT_OVERSHOOT      1

#define DEF_STEP_UP_PERIOD      500.0f
#define DEF_STEP_HEIGHT         90.0f
#define DEF_GROUND_HEIGHT       -140.0f //80 min, 180 max
#define DEF_CENTER_EXTENSION    189.05f
#define DEF_MAX_VEL_TRANS       200.0//35.0f
#define DEF_MAX_VEL_ROTATE      1.0472f

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
    float dx; //units (I use mm) per sec, + is to the right
    float dy; //units (I use mm) per sec, + is foreword
    float dr; //radians per sec (+ is counterclock)
} TRAJECTORY_2D;

typedef struct _Gait_Descriptor {
    int legModulus;
    float upStepPeriod;
    float stepHeight;
    float groundHeight;
    float maxTransVelocity;
    float maxRotVelocity;
} GAIT_DESCRIPTOR;

GAIT_DESCRIPTOR gaitDescriptor(int newModulus){
    GAIT_DESCRIPTOR newGait;
    newGait.legModulus = newModulus;
    newGait.upStepPeriod = DEF_STEP_UP_PERIOD;
    newGait.stepHeight = DEF_STEP_HEIGHT;
    newGait.groundHeight = DEF_GROUND_HEIGHT;
    newGait.maxTransVelocity = DEF_MAX_VEL_TRANS;
    newGait.maxRotVelocity = DEF_MAX_VEL_ROTATE;
    return newGait;
}

class GaitManager {
public:
    GaitManager(void);
    ~GaitManager();
    void generateCenters();

//--------------//
//  Gait interpolations
    void updateTargetTime(unsigned long newTime);
    IK_Error_T interpolateNextWalk(int legIndx);
    int getInterpolationTime();
    void pushIKtoTarget();

//--------------//
//  Gait transitions
    void switchGaitModulus(int newModulus);
    bool setNextTrajectory(float dx, float dy, float dr);

private:
    WALK_STATE_T cWalkState;
    WALK_STATE_T fWalkState;
    GAIT_DESCRIPTOR cGait;
    GAIT_DESCRIPTOR fGait;
    TRAJECTORY_2D cTrajectory;
    TRAJECTORY_2D fTrajectory;
    unsigned long idx_setMask_up;
    unsigned long idx_setMask_down;

    unsigned long targetTime;
    unsigned long timeOffset;
    int phase;
    float fraction;
    bool midStepToggle;

    COORD3D legCenters[CNT_LEGS];

//--------------//
//  Internal Gait mechanisms
    void setTarget(int legIndx);
    void updatePhase();
    void setTarget_Start(int legIndx);
    void setTarget_MidStep(int legIndx);
    void setTarget_End(int legIndx);
    void setTarget_Idle(int legIndx);

//--------------//
//  Leg Placement
    COORD3D deltaFromCenter(int legIndx, float tdx, float tdy, float tdr, float tdz);

};


GaitManager::GaitManager(void){
    cWalkState = walk_idle;
    fWalkState = walk_idle;
    cGait = gaitDescriptor(2);
    cGait.legModulus = 2;
    fGait = gaitDescriptor(2);
    fGait.legModulus = 2;
    
    cTrajectory = TRAJECTORY_2D{0.0f,0.0f,0.0f};
    fTrajectory = TRAJECTORY_2D{0.0f,0.0f,0.0f};

    targetTime = 0;
    timeOffset = 0;
    phase = 0;
    fraction = 0;
    midStepToggle = false;
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
    IK_Error_T error = legIK.leg3DOF_Inverse((LegIndex)legIndx);
#ifdef REPORT_IK_FAILURE
    if (error > ik_Success) {
        serialComms.startJsonMsg();
        serialComms.printVar("IK_Error", (int) error);
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
    for(int i = 0; i < CNT_LEGS; i++) {
        LegIndex legIndx = (LegIndex)i;
        if (legIK.isValidEffector(legIndx)) {
            servoTable_Target[hipH_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipH(legIndx), true), 0, 1023);
            servoTable_Target[hipV_cw_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndx), true), 0, 1023);
            servoTable_Target[hipV_ccw_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndx), false), 0, 1023);
            servoTable_Target[knee_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_knee(legIndx), false), 0, 1023);
            servoTable_Target[foot_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_ankle(legIndx), false), 0, 1023);
        }
    }
}

//--------------//
//  Gait transitions
void GaitManager::switchGaitModulus(int newModulus){
    fGait = gaitDescriptor(newModulus);
    if(cWalkState=walk_idle) {
        cGait = fGait;
    }
}

bool GaitManager::setNextTrajectory(float dx, float dy, float dr){
    float vel = sqrt(dx*dx+dy*dy);
    if (vel > cGait.maxTransVelocity)
        return false;
    if (fabs(dr) > cGait.maxRotVelocity)
        return false;
    fTrajectory.dx = dx;
    fTrajectory.dy = dy;
    fTrajectory.dr = dr;

    if((vel == 0) && (dr == 0)) {
        if(cWalkState == walk_idle) {
            fWalkState = walk_idle;
        } else {
            fWalkState = walk_stopping;
        }
    } else {
        if(cWalkState == walk_idle) {
            cGait = fGait;
            cTrajectory = fTrajectory;
            cWalkState = walk_starting;
            fWalkState = walk_active;
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
            setTarget_Idle(legIndx);
            break;
    }
//    SerialUSB.print("legIndx: ");
//    SerialUSB.println(legIndx,DEC);
//    SerialUSB.print("x: ");
//    SerialUSB.println(legIK.effector((LegIndex)legIndx).x, 2);
//    SerialUSB.print("y: ");
//    SerialUSB.println(legIK.effector((LegIndex)legIndx).y, 2);
//    SerialUSB.print("z: ");
//    SerialUSB.println(legIK.effector((LegIndex)legIndx).z, 2);
}

void GaitManager::updatePhase(){
    fraction = ((float)(targetTime - timeOffset))/(cGait.upStepPeriod*cGait.legModulus);
    int oldPhase = phase;
    
    //phase rollover - increment the offset, time for switching walk state
    if(fraction > cGait.legModulus) {
        timeOffset = targetTime;
        cWalkState = fWalkState;
        if(cWalkState == walk_starting)
          fWalkState = walk_active;
        if(cWalkState == walk_stopping)
          fWalkState = walk_idle;
        fraction = 0;
        phase = 0;
    } else {
        phase = (int) floor(fraction);
        fraction = fraction - phase;
    }
    
    //When switching phase, update the up/down masks
    // phase == up leg always!
    if(oldPhase != phase){
        idx_setMask_up = 0;
        for(int servoIndx = 0; servoIndx < foot_Index; servoIndx++){
            if( (servoIndx%legCount) == phase ){
                idx_setMask_up = idx_setMask_up | (1<<servoIndx);
            }
        }
        idx_setMask_down = !(idx_setMask_up);
        axm.holdingMode(idx_setMask_down);
        axm.freeMoveMode(idx_setMask_up);
    }

    if(cWalkState == walk_active) {
        if ((fraction > 0.5) && (!midStepToggle)) {
            midStepToggle = true;
            if (cGait.legModulus == 2) {
                //Switch to new trajectory is safe in legMod == 2 when at half step
                cTrajectory = fTrajectory;
                fTrajectory = TRAJECTORY_2D{0,0,0}; //Timeout defacto of trajectory
            }
        } else {
            midStepToggle = false;
            if (cGait.legModulus == 3) {
                //Switch to new trajectory is safe in legMod == 3 when at full step
                cTrajectory = fTrajectory;
            }
        }
    }
//    SerialUSB.print("cWalkState: ");
//    SerialUSB.println(cWalkState);
//    SerialUSB.print("fraction: ");
//    SerialUSB.println(fraction);
//    SerialUSB.print("phase: ");
//    SerialUSB.println(phase);
//    SerialUSB.println("\n\n");
}

void GaitManager::setTarget_Start(int legIndx) {
    int mod = (legIndx % cGait.legModulus);
    int legModDelta = (mod - phase + cGait.legModulus)%cGait.legModulus;
    if (legModDelta==0) {
        if(mod == 0) {
            legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                    cTrajectory.dx*(0.25 - cos(M_PI * fraction) * 0.25),
                    cTrajectory.dy*(0.25 - cos(M_PI * fraction) * 0.25),
                    cTrajectory.dr*(0.25 - cos(M_PI * fraction) * 0.25),
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else if (mod == (cGait.legModulus-1)){
            legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                    -cTrajectory.dx*(0.25 - cos(M_PI * fraction) * 0.25),
                    -cTrajectory.dy*(0.25 - cos(M_PI * fraction) * 0.25),
                    -cTrajectory.dr*(0.25 - cos(M_PI * fraction) * 0.25),
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else{
            legIK.effector((LegIndex)legIndx, legCenters[legIndx]);
        }
    }
}

void GaitManager::setTarget_MidStep(int legIndx) {
    int mod = (legIndx % cGait.legModulus);
    int legModDelta = (mod - phase + cGait.legModulus)%cGait.legModulus;
//    SerialUSB.print("legIndx: ");
//    SerialUSB.print(legIndx);
//    SerialUSB.print("    smFract: ");
//    SerialUSB.print((.5+.5*cos(M_PI * fraction)));
//    SerialUSB.print("    cTrajectory.dy: ");
//    SerialUSB.println(cTrajectory.dy);
//    SerialUSB.print("   legModDelta: ");
//    SerialUSB.println(legModDelta);
    if (legModDelta==0) {
        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                (.5*cos(M_PI * fraction)) * cTrajectory.dx,
                (.5*cos(M_PI * fraction)) * cTrajectory.dy,
                (.5*cos(M_PI * fraction)) * cTrajectory.dr,
                cGait.stepHeight * fabs(sin(M_PI * fraction))
        ));
    } else if (legModDelta == (cGait.legModulus-1)){
        float dualFraction = fraction/(cGait.legModulus-1);
        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                -(.5*cos(M_PI * dualFraction)) * cTrajectory.dx,
                -(.5*cos(M_PI * dualFraction)) * cTrajectory.dy,
                -(.5*cos(M_PI * dualFraction)) * cTrajectory.dr,
                0
        ));
    } else {
        float dualFraction = .5 + fraction/(cGait.legModulus-1);
        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                -(.5*cos(M_PI * dualFraction)) * cTrajectory.dx,
                -(.5*cos(M_PI * dualFraction)) * cTrajectory.dy,
                -(.5*cos(M_PI * dualFraction)) * cTrajectory.dr,
                0
        ));
    }
}

void GaitManager::setTarget_End(int legIndx) {
    int mod = (legIndx % cGait.legModulus);
    int legModDelta = (mod - phase + cGait.legModulus)%cGait.legModulus;
    if (legModDelta==0) {
        if(mod == 0) {
            legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                    cTrajectory.dx*(0.25 + cos(M_PI * fraction) * 0.25),
                    cTrajectory.dy*(0.25 + cos(M_PI * fraction) * 0.25),
                    cTrajectory.dr*(0.25 + cos(M_PI * fraction) * 0.25),
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else if (mod == (cGait.legModulus-1)){
            legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx,
                    -cTrajectory.dx*(0.25 + cos(M_PI * fraction) * 0.25),
                    -cTrajectory.dy*(0.25 + cos(M_PI * fraction) * 0.25),
                    -cTrajectory.dr*(0.25 + cos(M_PI * fraction) * 0.25), 
                    cGait.stepHeight * fabs(sin(M_PI * fraction))
            ));
        }else{
            legIK.effector((LegIndex)legIndx, legCenters[legIndx]);
        }
    }
}

void GaitManager::setTarget_Idle(int legIndx) {
    legIK.effector((LegIndex)legIndx, legCenters[legIndx]);
}

//--------------//
//  Leg Placement
COORD3D GaitManager::deltaFromCenter(int legIndx, float tdx, float tdy, float tdr, float tdz){
    COORD3D newPoint;
    newPoint.x = cos(tdr)*legCenters[legIndx].x - sin(tdr)*legCenters[legIndx].y + tdx;
    newPoint.y = sin(tdr)*legCenters[legIndx].x + cos(tdr)*legCenters[legIndx].y + tdy;
    newPoint.z = legCenters[legIndx].z + tdz;
//        SerialUSB.print("legIndx: ");
//        SerialUSB.println(legIndx,DEC);
//        SerialUSB.print("x: ");
//        SerialUSB.println(newPoint.x, 2);
//        SerialUSB.print("y: ");
//        SerialUSB.println(newPoint.y, 2);    
//        SerialUSB.print("z: ");
//        SerialUSB.println(newPoint.z, 2);   
//        SerialUSB.print("\n\n\n");  
    return newPoint;
}

void GaitManager::generateCenters() {
    for(int legIndx = 0; legIndx<CNT_LEGS; legIndx++) {
          legCenters[legIndx] = COORD3D{
                hipSegment[legIndx].segVect.x + DEF_CENTER_EXTENSION * cos(hipSegment[legIndx].effectorAngle),
                hipSegment[legIndx].segVect.y + DEF_CENTER_EXTENSION * sin(hipSegment[legIndx].effectorAngle),
                cGait.groundHeight
          };
//        SerialUSB.print("legIndx: ");
//        SerialUSB.println(legIndx,DEC);
//        SerialUSB.print("x: ");
//        SerialUSB.println(legCenters[legIndx].x, 2);
//        SerialUSB.print("y: ");
//        SerialUSB.println(legCenters[legIndx].y, 2);    
//        SerialUSB.print("z: ");
//        SerialUSB.println(legCenters[legIndx].z, 2);   
//        SerialUSB.print("\n\n\n");     
    }
}

extern GaitManager gaitGen;

#endif //KRAKEN_GAITGENERATOR_H
