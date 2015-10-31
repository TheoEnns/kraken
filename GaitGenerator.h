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

//#define REPORT_IK_FAILURE       1
//#define USE_GAIT_OVERSHOOT      1

#define DEF_STEP_HEIGHT         100.0f//120
#define DEF_GROUND_HEIGHT       -190.0f // safe range: -220, -160
#define MIN_CROUCH              -30.0
#define MAX_CROUCH              60.0
#define DEF_CENTER_EXTENSION    140.0f//safe range: 120, 160
#define DEF_MAX_VEL_TRANS       90.0f//90.0f
#define DEF_MAX_VEL_ROTATE      0.628f //.2*pi

#define SERVO_INTERPOLATION_RATE 10
#define GAIT_INTERPOLATION_RATE 80

#define PRE_STEP_DELAY .01
#define POST_STEP_DELAY .01
#define DEF_STEP_DOWN_PERIOD_STARTING      1000//ms
#define DEF_STEP_DOWN_PERIOD_ACTIVE        800//ms
#define DEF_STEP_DOWN_PERIOD_STOPPING      1000//ms

#ifdef USE_GAIT_OVERSHOOT
#define GAIT_INTERPOLATION_TARGET_TIME      (GAIT_INTERPOLATION_RATE+SERVO_INTERPOLATION_RATE)
#else
#define GAIT_INTERPOLATION_TARGET_TIME      (GAIT_INTERPOLATION_RATE)
#endif
#define START_INTERPOLATION_TARGET_TIME      2000

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
    unsigned long upStepPeriod;
    float stepHeight;
    float groundHeight;
    float maxTransVelocity;
    float maxRotVelocity;
} GAIT_DESCRIPTOR;

GAIT_DESCRIPTOR gaitDescriptor(int newModulus){
    GAIT_DESCRIPTOR newGait;
    newGait.legModulus = newModulus;
    newGait.upStepPeriod = DEF_STEP_DOWN_PERIOD_STARTING;
    newGait.stepHeight = DEF_STEP_HEIGHT;
    newGait.groundHeight = DEF_GROUND_HEIGHT;
    newGait.maxTransVelocity = DEF_MAX_VEL_TRANS;
    newGait.maxRotVelocity = DEF_MAX_VEL_ROTATE;
    return newGait;
}

float drawTransition(float fraction, float startV, float stopV, bool slowStart, bool slowStop, float startDelay, float stopDelay){
    if (fraction < startDelay) {
        return startV;
    } else if (fraction >= (1-stopDelay)) {
        return stopV;
    } else {
        if (slowStart && slowStop) {
            float delayOffset = M_PI*(startDelay);
            float transitionFactor = M_PI/(1.0 - startDelay - stopDelay);
            return .5*(startV+stopV) + .5*(startV-stopV)*cos(transitionFactor*fraction-delayOffset);
        } else if (slowStart) {
            float delayOffset = M_PI_2*(startDelay);
            float transitionFactor = M_PI_2/(1.0 - startDelay - stopDelay);
            return (stopV) + (startV-stopV)*cos(transitionFactor*fraction-delayOffset);
        } else if (slowStop) {
            float delayOffset = M_PI_2*(startDelay);
            float transitionFactor = M_PI_2/(1.0 - startDelay - stopDelay);
            return (startV) + (stopV-startV)*sin(transitionFactor*fraction-delayOffset);
        } else {
            float delayOffset = startDelay;
            float transitionFactor = 1.0/(1.0 - startDelay - stopDelay);
            return (startV) + (stopV-startV)*transitionFactor*(fraction-delayOffset);
        }
    }
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
    void setCrouch(float newCrouch);
    void modifyCrouch(float delta);

private:
    float crouch;
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
    int stateEndPhase;
    float fraction;
    bool midStepToggle;
    
    //Phase params; internally use by leg targeting
        int legMod; //leg index % gait modulus
        int legModDelta; // (legMod - phase) % gait modulus
        float legDeltaModuloFactor;// = 2/(cGait.legModulus-1.0);
        float legStartDelta;// = legDeltaModuloFactor*(legModDelta - 1.0) - 1.0;
        float legStopDelta;// = legDeltaModuloFactor*(legModDelta) - 1.0;

    float legCenter_Angles[CNT_LEGS];
    float legCenter_Radii[CNT_LEGS];
    float legCenter_Heights[CNT_LEGS];
    
    float legCurrentDelta_x[CNT_LEGS];
    float legCurrentDelta_y[CNT_LEGS];
    float legCurrentDelta_r[CNT_LEGS];
    float legCurrentDelta_z[CNT_LEGS];
    
//    float legTargetDelta_x[CNT_LEGS];
//    float legTargetDelta_y[CNT_LEGS];
//    float legTargetDelta_r[CNT_LEGS];
//    float legTargetDelta_z[CNT_LEGS];

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
    crouch = 0.0;
    cWalkState = walk_idle;
    fWalkState = walk_idle;
    cGait = gaitDescriptor(2);
    cGait.legModulus = 2;
    fGait = gaitDescriptor(2);
    fGait.legModulus = 2;
    
    legDeltaModuloFactor = 0;
    legStartDelta = 0;
    legStopDelta = 0;
        
    cTrajectory = TRAJECTORY_2D{0.0f,0.0f,0.0f};
    fTrajectory = TRAJECTORY_2D{0.0f,0.0f,0.0f};

    targetTime = 0;
    timeOffset = 0;
    phase = 0;
    stateEndPhase = 1;
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
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipH(legIndx), false), 0, 1023);
            servoTable_Target[hipV_cw_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndx), true), 0, 1023);
            servoTable_Target[hipV_ccw_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_hipV(legIndx), false), 0, 1023);
            servoTable_Target[knee_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_knee(legIndx), false), 0, 1023);
            servoTable_Target[foot_Index + i] =
                    LIMIT(ANGLE_TO_SERVO(legIK.angle_ankle(legIndx), false), 0, 1023);
//            legCurrentDelta_x[i] = legTargetDelta_x[i];
//            legCurrentDelta_y[i] = legTargetDelta_y[i];
//            legCurrentDelta_r[i] = legTargetDelta_r[i];
//            legCurrentDelta_z[i] = legTargetDelta_z[i];
        }
//          if( (fraction == 1.0)){
//              SerialUSB.print("legIndx: ");
//              SerialUSB.print(legIndx);
//              SerialUSB.print("hipH: ");
//              SerialUSB.print(servoTable_Target[hipH_Index + i]);
//              SerialUSB.print("    hipV cw: ");
//              SerialUSB.print(servoTable_Target[hipV_cw_Index + i]);
//              SerialUSB.print("    hipV ccw: ");
//              SerialUSB.print(servoTable_Target[hipV_ccw_Index + i]);
//              SerialUSB.print("    knee ");
//              SerialUSB.println(servoTable_Target[knee_Index + i]);
//          }
    }
}

//--------------//
//  Gait transitions
void GaitManager::switchGaitModulus(int newModulus){
    fGait = gaitDescriptor(newModulus);
    if(cWalkState==walk_idle) {
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
        if( (cWalkState == walk_idle) || (cWalkState == walk_stopping) ){
            fWalkState = walk_idle;
        } else {
            fWalkState = walk_stopping;
        }
    } else {
        if(cWalkState == walk_idle) {
            cGait = fGait;
            cTrajectory = fTrajectory;
            if(fWalkState != walk_starting){
                stateEndPhase = 0;
                fWalkState = walk_starting;
                fraction = 1.0;
                phase = 0;
                timeOffset = targetTime;
            }
        }
    }
}

void GaitManager::setCrouch(float newCrouch){
    crouch = LIMIT(newCrouch,MIN_CROUCH,MAX_CROUCH);
    for(int legIndx=0; legIndx<CNT_LEGS; legIndx++){
      legCenter_Heights[legIndx] = cGait.groundHeight + crouch;
    }
}

void GaitManager::modifyCrouch(float delta){
    setCrouch(crouch+delta);
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
}

void GaitManager::updatePhase(){
    float oldFraction = fraction;
    int oldPhase = phase;
    fraction = ((float)(targetTime - timeOffset))/((float)cGait.upStepPeriod);
    if(fraction < oldFraction){
        phase = (phase + 1 + cGait.legModulus%2)%cGait.legModulus;
        fraction = 0;
        timeOffset = targetTime;
    }else if(fraction > 1.0){
        fraction = 1.0;
        timeOffset = targetTime;
        stateEndPhase --;
    }
    
    if((fraction == 0.0) && (stateEndPhase <= 0)) {
        if(fWalkState == walk_starting){
          cWalkState = fWalkState;
          cGait.upStepPeriod = DEF_STEP_DOWN_PERIOD_STARTING/(cGait.legModulus-1);
          fWalkState = walk_active;
          stateEndPhase = cGait.legModulus; //2 for mod2
          phase = 0;
        }else if(fWalkState == walk_stopping){
          cWalkState = fWalkState;
          cGait.upStepPeriod = DEF_STEP_DOWN_PERIOD_STOPPING/(cGait.legModulus-1);
          fWalkState = walk_idle;
          stateEndPhase = cGait.legModulus;//2;
        }else if(fWalkState == walk_idle){
          cWalkState = fWalkState;
          cGait.upStepPeriod = DEF_STEP_DOWN_PERIOD_STOPPING/(cGait.legModulus-1);
          stateEndPhase = 0;
          phase = 0;
        }else if (fWalkState == walk_active){
          if(cWalkState != walk_active){
            phase = 0;
            cWalkState = walk_active;  
            cGait.upStepPeriod = DEF_STEP_DOWN_PERIOD_ACTIVE/(cGait.legModulus-1); 
          } else {
                //Switch to new trajectory is safe only after a midstep!
                cTrajectory = fTrajectory;
          }
          stateEndPhase = 0;
        }
    }
    
//    if(fraction == 0.0){
//     //When starting phase, update the up/down masks
//     // phase == up leg always!
//        idx_setMask_up = 0;
//        for(int servoIndx = 0; servoIndx < foot_Index; servoIndx++){
//            if( (servoIndx%legCount) == phase ){
//                idx_setMask_up = idx_setMask_up | (1<<servoIndx);
//            }
//        }
//        idx_setMask_down = ~(idx_setMask_up);
//        axm.holdingMode(idx_setMask_down);
//        axm.freeMoveMode(idx_setMask_up);
//    }

    if(cWalkState == walk_active) {
        if (fraction == 0.0) {
            midStepToggle = false;
        } else if((fraction >= 0.5) && (!midStepToggle)) {
            midStepToggle = true;
        }
    }
    
//    SerialUSB.print("cWalkState: ");
//    SerialUSB.print(cWalkState);
////    SerialUSB.print("   fWalkState: ");
////    SerialUSB.println(fWalkState);
//    SerialUSB.print("fraction: ");
//    SerialUSB.print(fraction);
//    SerialUSB.print("   phase: ");
//    SerialUSB.println(phase);
//    SerialUSB.println("\n\n");

    legDeltaModuloFactor = 2/(cGait.legModulus-1.0);
}

void GaitManager::setTarget_Start(int legIndx) {
    legMod = (legIndx % cGait.legModulus);
    legModDelta = (legMod + phase + cGait.legModulus)%cGait.legModulus;
    if (legModDelta==0) {
        if (legMod == 0) {
            //The robot always starts midsteps with lifting the leg set that includes leg index 0 (it is foreright footed!)
            legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                    drawTransition(fraction,0,-cTrajectory.dx,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                    drawTransition(fraction,0,-cTrajectory.dy,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                    drawTransition(fraction,0,-cTrajectory.dr,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                    cGait.stepHeight * 0.5* fabs(1-cos(2*M_PI * fraction ))
            ));
        } else {
            //The robot always starts midsteps with lifting the leg set that includes leg index 0 (it is foreright footed!)
            legStartDelta = legDeltaModuloFactor*(legMod) - 1.0;
            legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx,
                    drawTransition(fraction,0,legStartDelta*cTrajectory.dx,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                    drawTransition(fraction,0,legStartDelta*cTrajectory.dy,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                    drawTransition(fraction,0,legStartDelta*cTrajectory.dr,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                    cGait.stepHeight * 0.5* fabs(1-cos(2*M_PI * fraction ))
            ));
        }
    }
}

void GaitManager::setTarget_MidStep(int legIndx) {
    legMod = (legIndx % cGait.legModulus);
    legModDelta = (legMod + phase + cGait.legModulus)%cGait.legModulus;
    if (legModDelta==0) {
        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                drawTransition(fraction,-cTrajectory.dx,cTrajectory.dx,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                drawTransition(fraction,-cTrajectory.dy,cTrajectory.dy,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                drawTransition(fraction,-cTrajectory.dr,cTrajectory.dr,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                cGait.stepHeight * 0.5* fabs(1-cos(2*M_PI * fraction ))
        ));
    }  else {
        legStartDelta = legDeltaModuloFactor*(legModDelta) - 1.0;
        legStopDelta = legDeltaModuloFactor*(legModDelta - 1.0) - 1.0;
        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx,
                drawTransition(fraction,legStartDelta*cTrajectory.dx,legStopDelta*fTrajectory.dx,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                drawTransition(fraction,legStartDelta*cTrajectory.dy,legStopDelta*fTrajectory.dy,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                drawTransition(fraction,legStartDelta*cTrajectory.dr,legStopDelta*fTrajectory.dr,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                0
        ));
    }
    
//    if( (fraction == 1.0)){
//        SerialUSB.print("legIndx: ");
//        SerialUSB.print(legIndx);
////        SerialUSB.print("fraction: ");
////        SerialUSB.println(fraction);
////        SerialUSB.print("phase: ");
////        SerialUSB.println(phase);
//        COORD3D aLeg  = legIK.effector((LegIndex)legIndx);
//        SerialUSB.print("x: ");
//        SerialUSB.print(aLeg.x,2);
//        SerialUSB.print(",  y: ");
//        SerialUSB.print(aLeg.y,2);
//        SerialUSB.print(",  z: ");
//        SerialUSB.println(aLeg.z,2);
////        SerialUSB.println("\n\n");
//    }
}

void GaitManager::setTarget_End(int legIndx) {
    legMod = (legIndx % cGait.legModulus);
    legModDelta = (legMod + phase + cGait.legModulus)%cGait.legModulus;
//    if (legModDelta==0) {
//        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
//                drawTransition(fraction,-cTrajectory.dx,cTrajectory.dx,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
//                drawTransition(fraction,-cTrajectory.dy,cTrajectory.dy,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
//                drawTransition(fraction,-cTrajectory.dr,cTrajectory.dr,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
//                cGait.stepHeight * 0.5* fabs(1-cos(2*M_PI * fraction ))
//        ));
//    }  else {
//        legStartDelta = legDeltaModuloFactor*(legModDelta) - 1.0;
//        legStopDelta = legDeltaModuloFactor*(legModDelta - 1.0) - 1.0;
//        legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx,
//                drawTransition(fraction,legStartDelta*cTrajectory.dx,legStopDelta*cTrajectory.dx,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
//                drawTransition(fraction,legStartDelta*cTrajectory.dy,legStopDelta*cTrajectory.dy,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
//                drawTransition(fraction,legStartDelta*cTrajectory.dr,legStopDelta*cTrajectory.dr,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
//                0
//        ));
//    }
    if (legModDelta==0) {
          //The robot always starts endsteps with lifting the leg set that includes leg index 0 (it is foreright footed!)
          legStopDelta = legDeltaModuloFactor*(legModDelta - 1.0) - 1.0;
          legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx,
                  drawTransition(fraction,legCurrentDelta_x[legIndx],0,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                  drawTransition(fraction,legCurrentDelta_y[legIndx],0,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                  drawTransition(fraction,legCurrentDelta_r[legIndx],0,true,true,PRE_STEP_DELAY,POST_STEP_DELAY),
                  cGait.stepHeight * 0.5* fabs(1-cos(2*M_PI * fraction ))
          ));
    }else{
          legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx,
                  legCurrentDelta_x[legIndx],
                  legCurrentDelta_y[legIndx],
                  legCurrentDelta_r[legIndx],
                  legCurrentDelta_z[legIndx]
          ));
    }
}

void GaitManager::setTarget_Idle(int legIndx) {
    legIK.effector((LegIndex)legIndx, deltaFromCenter( legIndx, 
                    0,
                    0,
                    0,
                    0
            ));
}

//--------------//
//  Leg Placement
COORD3D GaitManager::deltaFromCenter(int legIndx, float tdx, float tdy, float tdr, float tdz){
    COORD3D newPoint;
    newPoint.x =  .5*tdx + legCenter_Radii[legIndx]*cos(-.5*tdr + legCenter_Angles[legIndx]);
    newPoint.y =  .5*tdy + legCenter_Radii[legIndx]*sin(-.5*tdr + legCenter_Angles[legIndx]);
    newPoint.z = legCenter_Heights[legIndx] + tdz;
//    legTargetDelta_x[legIndx] = tdx;
//    legTargetDelta_y[legIndx] = tdy;
//    legTargetDelta_r[legIndx] = tdr;
//    legTargetDelta_z[legIndx] = tdz;
    legCurrentDelta_x[legIndx] = tdx;
    legCurrentDelta_y[legIndx] = tdy;
    legCurrentDelta_r[legIndx] = tdr;
    legCurrentDelta_z[legIndx] = tdz;
//    if( (newPoint.z != -200)){
//        SerialUSB.print("legIndx: ");
////        SerialUSB.println(legIndx,DEC);
////        SerialUSB.print("x: ");
////        SerialUSB.println(newPoint.x, 2);
////        SerialUSB.print("y: ");
////        SerialUSB.println(newPoint.y, 2);    
//        SerialUSB.print("z: ");
//        SerialUSB.println(newPoint.z, 2); 
//        SerialUSB.print("tdz: ");
//        SerialUSB.println(tdz, 2);  
//        SerialUSB.print("phase: ");
//        SerialUSB.println(phase, 2);    
//        SerialUSB.print("fraction: ");
//        SerialUSB.println(fraction, 2);  
//        SerialUSB.print("\n\n\n");  
//    }
    return newPoint;
}

void GaitManager::generateCenters() {
    for(int legIndx = 0; legIndx<CNT_LEGS; legIndx++) {
          legCenter_Angles[legIndx] = hipSegment[legIndx].effectorAngle;
          legCenter_Radii[legIndx] = sqrt( cSquare(hipSegment[legIndx].segVect.x) + cSquare(hipSegment[legIndx].segVect.y) )
                  + DEF_CENTER_EXTENSION;
          legCenter_Heights[legIndx] = cGait.groundHeight + crouch;
    }
}

extern GaitManager gaitGen;

#endif //KRAKEN_GAITGENERATOR_H

