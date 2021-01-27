#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "Arduino.h"
#include "kinematic.h"

float calculateVx( float v1, float v2, float v3, float v4, float theta, float R_robot);

float calculateVy( float v1, float v2, float v3, float v4, float theta, float R_robot);

float calculateOmega( float v1, float v2, float v3, float v4, float theta, float R_robot);




float calculateV1( float vx, float vy, float Omega, float theta, float R_robot);

float calculateV2( float vx, float vy, float Omega, float theta, float R_robot);

float calculateV3( float vx, float vy, float Omega, float theta, float R_robot);

float calculateV4( float vx, float vy, float Omega, float theta, float R_robot);




#endif
