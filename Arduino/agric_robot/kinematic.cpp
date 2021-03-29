#include "kinematic.h"

float calculateVx( float v1, float v2, float v3, float v4, float a, float b) {
  return (1.0/4.0)*(v1+v2+v3+v4)   ;
}

//A-1 ligne 2
float calculateVy( float v1, float v2, float v3, float v4, float a, float b) {
  return (1.0/4.0)*(-v1+v2+v3-v4)   ;
}

//A-1 ligne 3
float calculateOmega( float v1, float v2, float v3, float v4, float a,  float b) {
  return  (1.0/((a+b)*4.0))*(-v1+v2-v3+v4)  ;
}
