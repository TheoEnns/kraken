//
// Created by Theo on 10/22/2015.
//

#ifndef KRAKEN_PHYSICAL_TYPES_AND_CONVERSIONS_H
#define KRAKEN_PHYSICAL_TYPES_AND_CONVERSIONS_H

float cSquare(float x){
  return x*x;
}

//--------------//
//  Angle tools
#define ANGLE_TO_SERVO_POS_FACTOR  195.569585f  //rescales an angle to a servo position value
#define ANGLE_TO_SERVO_OFFSET      512        //Zero degrees on a centered joint corresponds to this
                                                // servo position
#define DEGREES_TO_RADS(x)          ( ((float)x)*M_PI/180.0f )
#define RADS_TO_DEGREES(x)          ( ((float)x)*180.0f/M_PI )
#define LIMIT(x,a,b)                (x<a? a: (x>b? b: x))
#define RADS_TO_SERVO(x)            ((word)round(ANGLE_TO_SERVO_OFFSET + x*ANGLE_TO_SERVO_POS_FACTOR))
#define ANGLE_TO_SERVO(x,isCCW)     (isCCW? RADS_TO_SERVO(x): RADS_TO_SERVO(-x))

//--------------//
//  Structural Types
typedef struct _Coord3D {
    float      x;
    float      y;
    float      z;
} COORD3D;

typedef struct _Coord2D {
    float      x;
    float      y;
} COORD2D;

//typedef struct _Arc2D {
//    float      cX2;
//    float      cY2;
//    float      cX;
//    float      cY;
//    float      c0;
////Start point
//    float      x0;
//    float      y0;
////End point
//    float      x1;
//    float      y1;
//} ARC2D;


//This specialized arc describes a movement (dx/dt,dy/dt,dr/dt) about the robots origin
typedef struct _Parametized_Arc2D {
    float dx;
    float dy;
    float dr;
    float r;
    float cTheta;
//Start and end point params
    float startT;
    float endT;
} P_ARC2D;

typedef struct _Segment3D {
    COORD3D segVect; //vector of segment when servo is centered
    float maxAngle; //degrees, counterclockwise from center orientation
    float minAngle; //degrees, counterclockwise from center orientation
    float effectorAngle;
//    float collisionRadius;  //A cylinder around the joint segment for collision detection
} SEGMENT3D;

//typedef struct _ServoStructuralInfo {
//    int servo_idx;
//    bool isServoCounterClockwise;  //boolean for whether the servo turns the joint counterclockwise
//    float angleScaling;  //Conversion factor between joint angle (degrees) and servo position
//    float center_adjust;  //degrees counter clockwise that the servo is off true center;
//                            // not needed on dynamixels
//} SERVOLOGISTICS;

typedef struct _Joint3D {
    const SEGMENT3D * segments;   //describes the leg segment

    bool    hasDualServos; //Some joints may have servo doubling, in which two servos operate the same joint
                            //Servo doubling is a way of increasing joint torque
                            //  without sacrificing angular accuracy to sloppy 3d printed PLA gears or
                            //  buying very expensive MX servos. My experiments with PLA gears on AX-12 servos
                            //  found a very poor trade off in which the soft plastic is not only sloppy but
                            //  experiences a very high friction cost.
    int     primaryServo;
    bool    primaryOrientation; //boolean for direction the servo turns. true=counterclockwise, false=clockwise
    int     secondaryServo;
    bool    secondaryOrientation; //boolean for direction the servo turns. true=counterclockwise, false=clockwise
} JOINT3D;

//--------------//
//  Vector math
float vMultiply(COORD3D vectA, COORD3D vectB)
{
    return vectA.x*vectB.x + vectA.y*vectB.y + vectA.z*vectB.z;
}

COORD3D vAdd(COORD3D vectA, COORD3D vectB)
{
    COORD3D result;
    result.x = vectA.x + vectB.x;
    result.y = vectA.y + vectB.y;
    result.z = vectA.z + vectB.z;
    return result;
}

COORD3D vSubract(COORD3D vectA, COORD3D vectB)
{
    COORD3D result;
    result.x = vectA.x - vectB.x;
    result.y = vectA.y - vectB.y;
    result.z = vectA.z - vectB.z;
    return result;
}

COORD3D vScale(COORD3D vectA, float scale)
{
    COORD3D result;
    result.x = vectA.x*scale;
    result.y = vectA.y*scale;
    result.z = vectA.z*scale;
    return result;
}

float vLength(COORD3D vectA)
{
    return sqrt(cSquare(vectA.x) + cSquare(vectA.y) + cSquare(vectA.z));
}

COORD3D vRotate(COORD3D originalVector, float rotationAngle, COORD3D rotationAxis)
{
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    COORD3D result;
    COORD3D rotationMat_X = {
         cSquare(rotationAxis.x)*(1-cosA)+cosA,
         rotationAxis.x*rotationAxis.y*(1-cosA)-rotationAxis.z*sinA,
         rotationAxis.x*rotationAxis.z*(1-cosA)+rotationAxis.y*sinA,
        };
    COORD3D rotationMat_Y = {
         rotationAxis.x*rotationAxis.y*(1-cosA)+rotationAxis.z*sinA,
         cSquare(rotationAxis.y)*(1-cosA)+cosA,
         rotationAxis.y*rotationAxis.z*(1-cosA)-rotationAxis.x*sinA,
        };
    COORD3D rotationMat_Z = {
         rotationAxis.x*rotationAxis.z*(1-cosA)-rotationAxis.y*sinA,
         rotationAxis.y*rotationAxis.z*(1-cosA)+rotationAxis.x*sinA,
         cSquare(rotationAxis.z)*(1-cosA)+cosA,
        };
    result.x = vMultiply( originalVector, rotationMat_X);
    result.y = vMultiply( originalVector, rotationMat_Y);
    result.z = vMultiply( originalVector, rotationMat_Z);
    return result;
}

//--------------//
//  2D Trig
//Returns -1 when no valid solution exists
//  The openCM compiler uses a not really
//  float type for float which does not have a NaN.

float getOpposingAngle(float cLength, float aLength, float bLength)
{
    float rNumerator = cSquare(aLength) + cSquare(bLength) - cSquare(cLength);
    float rDenominator = 2*aLength*bLength;
    if (rDenominator == 0) {
        return -1;
    }
    float cos_cAngle = rNumerator/rDenominator;
    if (fabs(cos_cAngle) > 1) {
        return -1;
    }
    float result = acos(cos_cAngle);
    return result;
}

//// The ordering of points 'a,'b,'c does not effect the equation, but
////  I take point 'a as my start and 'c as the end implicitly.
//ARC2D makeArc(float aX, float aY, float bX, float bY, float cX, float cY){
//    ARC2D myARC;
//    float aX2 = cSquare(aX^2);
//    float bX2 = cSquare(bX^2);
//    float cX2 = cSquare(cX^2);
//    float aY2 = cSquare(aY^2);
//    float bY2 = cSquare(bY^2);
//    float cY2 = cSquare(cY^2);
//    myARC.cX2 = myARC.cY2 =  aX*bY - aY*bX - aX*cY + aY*cX + bX*cY - bY*cX;
//    myARC.cX = - aX2*bY + aX2*cY - aY2*bY + aY2*cY + aY*bX2 + aY*bY2 - aY*cX2 - aY*cY2 - bX2*cY - bY2*cY + bY*cX2 + bY*cY2;
//    myARC.cY = aX2*bX - aX2*cX - aX*bX2 - aX*bY2 + aX*cX2 + aX*cY2 + aY2*bX - aY2*cX + bX2*cX - bX*cX2 - bX*cY2 + bY2*cX;
//    myARC.c = - aX2*bX*cY + aX2*bY*cX + aX*bX2*cY + aX*bY2*cY - aX*bY*cX2 - aX*bY*cY2 - aY2*bX*cY + aY2*bY*cX - aY*bX2*cX + aY*bX*cX2 + aY*bX*cY2 - aY*bY2*cX;
//    myARC.x0 = aX;
//    myARC.x0 = aY;
//    myARC.x1 = cX;
//    myARC.x1 = cY;
//}

P_ARC2D makeArc(float r0, float theta0, float angularRate, float velX, float velY, float stepTime){
    P_ARC2D myARC;
    myARC.r = r0;
    myARC.dr = angularRate;
    myARC.dx = velX;
    myARC.dy = velY;
    myARC.cTheta = theta0;
    myARC.startT = -stepTime/2;
    myARC.endT = stepTime/2;
    return myARC;
}

COORD2D getArcPoint(P_ARC2D * mArc, float pathFraction){
    COORD2D newPt;
    float t = (1 - pathFraction)*mArc->startT + (pathFraction)*mArc->endT;
    newPt.x = mArc->dx*t + cos(t*mArc->dr + mArc->cTheta)*mArc->r;
    newPt.y = mArc->dy*t + sin(t*mArc->dr + mArc->cTheta)*mArc->r;
    return newPt;
}

bool constrictARC(P_ARC2D * mArc, float lowSwing, float highSwing, float lowReach, float highReach){
    bool changedArc = false;
//    startPt = getArcPoint(mArc, 0.0f);
//    endPt = getArcPoint(mArc, 1.0f);
//    float swingStart = atan2(startPt.y,startPt.x);
//    float swingEnd = atan2(endPt.y,endPt.x);
//    if(swingStart>highSwing){
//        ;
//        changedArc = true;
//    }
//    if(swingStart<lowSwing){
//        ;
//        changedArc = true;
//    }

    return changedArc;
}

#endif //KRAKEN_PHYSICAL_TYPES_AND_CONVERSIONS_H
