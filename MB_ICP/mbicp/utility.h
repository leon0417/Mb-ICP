// nike [2/29/2008 Administrator]
#ifndef UTILITY_H
#define UTILITY_H

//#include "Aria.h"
/////////////////////////////////////////////////////////////////////////
//inline double NormalizarPI(double angle);
#include <math.h>
//////////////////////////////////////////////////////////////////////////
#define NULL 0
#define M_PI 3.141592
//////////////////////////////////////////////////////////////////////////
typedef struct tag_RPOINT
{
	double x;
	double y;
	double z;
} RPOINT,*PRPOINT;
//-----------------------------------------
typedef struct {
	float r;
	float t;
}Tpfp,*PTpfp;
//-----------------------------------------
typedef struct {
	int numPuntos;
	PRPOINT laserC;  // Cartesian coordinates
	PTpfp laserP; // Polar coordinates
}Tscan,PTscan;
//-----------------------------------------
// Associations information
typedef struct{
	float rx,ry,nx,ny,dist;				// Point (nx,ny), static corr (rx,ry), dist 
	int numDyn;							// Number of dynamic associations
	float unknown;						// Unknown weight
	int index;							// Index within the original scan
	int L,R;
}TAsoc,*PTAsoc;
//////////////////////////////////////////////////////////////////////////
//-----------------------------------------
void car2pol(PRPOINT in, PTpfp out);
//-----------------------------------------
void pol2car(PTpfp in, PRPOINT out);
//-----------------------------------------
//////////////////////////////////////////////////////////////////////////
void swapItem(TAsoc *a, TAsoc *b);
//-----------------------------------------
void perc_down(TAsoc a[], int i, int n); 
//-----------------------------------------
void heapsort(TAsoc a[], int n); 
//-----------------------------------------
double radToDeg(double rad) ;
//-----------------------------------------
double degToRad(double deg) ;
//-----------------------------------------
double NormalizarPI(double angle) ;
/////////////////////////////////////////////////////////////////////////
class ArPose
{
public:
  /// Constructor, with optional initial values
  /** 
      Sets the position with the given values, can be used with no variables,
      with just x and y, or with x, y, and th
      @param x the position to set the x position to, default of 0
      @param y the position to set the y position to, default of 0
      @param th the position to set the th position to, default of 0
  */
  ArPose(double x = 0, double y = 0, double th = 0)
    { myX = x; myY = y; myTh = th; }
  /// Copy Constructor
  ArPose(const ArPose &pose) : 
    myX(pose.myX), myY(pose.myY), myTh(pose.myTh) {}

  /// Destructor
  virtual ~ArPose() {}
  /// Sets the position to the given values
  /** 
      Sets the position with the given three values, but the theta does not
      need to be given as it defaults to 0.
      @param x the position to set the x position to
      @param y the position to set the y position to
      @param th the position to set the th position to, default of 0
  */
  virtual void setPose(double x, double y, double th = 0) 
    { setX(x); setY(y); setTh(th); }
  /// Sets the position equal to the given position
  /** @param position the position value this instance should be set to */
  virtual void setPose(ArPose position)
    {
      setX(position.getX());
      setY(position.getY());
      setTh(position.getTh());
    }
  /// Sets the x position
  void setX(double x) { myX = x; }
  /// Sets the y position
  void setY(double y) { myY = y; }
  /// Sets the heading
  void setTh(double th) { 
	  myTh = fixAngle(th); 
  }
  /// Sets the heading, using radians
  void setThRad(double th) { myTh = fixAngle(radToDeg(th)); }
  //static double radToDeg(double rad) { return rad * 180.0 / 3.14159; }

  /// Gets the x position
  double getX(void) const { return myX; }
  /// Gets the y position
  double getY(void) const { return myY; }
  /// Gets the heading
  double getTh(void) const { return myTh; }
  /// Gets the heading, in radians
  double getThRad(void) const { 
	  return degToRad(myTh); 
  }
 // static double degToRad(double deg) { return deg * 3.14159 / 180.0; }

  /// Gets the whole position in one function call
  /**
     Gets the whole position at once, by giving it 2 or 3 pointers to 
     doubles.  If you give the function a null pointer for a value it won't
     try to use the null pointer, so you can pass in a NULL if you don't 
     care about that value.  Also note that th defaults to NULL so you can 
     use this with just x and y.
     @param x a pointer to a double to set the x position to
     @param y a pointer to a double to set the y position to
     @param th a pointer to a double to set the heading to, defaults to NULL
   */
  void getPose(double *x, double *y, double *th = NULL) const
    { 
      if (x != NULL) 
	*x = myX;
      if (y != NULL) 
	*y = myY; 
      if (th != NULL) 
	*th = myTh; 
    }
  /// Finds the distance from this position to the given position
  /**
     @param position the position to find the distance to
     @return the distance to the position from this instance
  */
  //virtual double findDistanceTo(ArPose position) const
  //  {
  //    return ArMath::distanceBetween(getX(), getY(), 
		//		     position.getX(), 
		//		     position.getY());
  //  }

  /// Finds the square distance from this position to the given position
  /**
     This is only here for speed, if you aren't doing this thousands
     of times a second don't use this one use findDistanceTo

     @param position the position to find the distance to
     @return the distance to the position from this instance 
  **/
  //virtual double squaredFindDistanceTo(ArPose position) const
  //  {
  //    return ArMath::squaredDistanceBetween(getX(), getY(), 
		//			    position.getX(), 
		//			    position.getY());
  //  }
  /// Finds the angle between this position and the given position
  /** 
      @param position the position to find the angle to
      @return the angle to the given position from this instance, in degrees
  */
  //virtual double findAngleTo(ArPose position) const
  //  {
  //    return ArMath::radToDeg(atan2(position.getY() - getY(),
		//		    position.getX() - getX()));
  //  }
  /// Logs the coordinates using ArLog
  //virtual void log(void) const
  //  { ArLog::log(ArLog::Terse, "%.0f %.0f %.1f", myX, myY, myTh); }

  /// Add the other pose's X, Y and theta to this pose's X, Y, and theta (sum in theta will be normalized to (-180,180)), and return the result
  virtual ArPose operator+(const ArPose& other)
  {
    return ArPose( myX + other.getX(), myY + other.getY(), fixAngle(myTh + other.getTh()) );
  }

  /// Substract the other pose's X, Y, and theta from this pose's X, Y, and theta (difference in theta will be normalized to (-180,180)), and return the result
  virtual ArPose operator-(const ArPose& other)
  {
    return ArPose( myX - other.getX(), myY - other.getY(), fixAngle(myTh - other.getTh()) );
  }
  double fixAngle(double angle)
  {
	   if (angle >= 360)
         angle = angle - 360.0 * (double)((int)angle / 360);
       if (angle < -360)
         angle = angle + 360.0 * (double)((int)angle / -360);
       if (angle <= -180)
		 angle = + 180.0 + (angle + 180.0);
       if (angle > 180)
		 angle = - 180.0 + (angle - 180.0);
       return angle;
  }

protected:
  double myX;
  double myY;
  double myTh;
};

#endif