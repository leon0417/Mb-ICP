#include "stdafx.h"
#include "utility.h"

//
double radToDeg(double rad) 
{
  return rad * 180.0 / 3.14159; 
}
//
double degToRad(double deg) 
{ 
  return deg * 3.14159 / 180.0; 
}
//
double NormalizarPI(double angle) 
{ 
	// normalise an angle to within +/- PI
	while (angle < -M_PI)
		angle += 2.*M_PI;
	while (angle > M_PI)
		angle -= 2.*M_PI;
	return angle;
}