#include "COREDrive.h"
#include "WPILib.h"
#include <algorithm>

using namespace CORE;

double inline etherL(double fwd, double rcw, double a, double b){
	return fwd + b*rcw*(1-fwd);
}

double inline etherR(double fwd, double rcw, double a, double b){
	return fwd-b*rcw + fwd*rcw*(b-a-1);
}

/*
 Ether:
 
 Arcade algorithm to compute left and right wheel commands 
 from forward and rotate-clockwise joystick commands.

  "a" and "b" are tuning parameters:
  a is the amount to turn at 100% fwd in the range 0 to 1;
  b is the amount to turn at   0% fwd in the range 0 to 1;
  when a=0 and b=1 this gives you the WPILib arcade behavior;
*/
void COREDrive::EtherArcade(double mag, double rotate, double a, double b){
	
	double left;
	double right;
	
	if (mag>=0){
		if (rotate>=0){
			left = etherL(mag, rotate, a, b);
			right = etherR(mag, rotate, a, b);
		} else{
			left = etherR(mag, -rotate, a, b);
			right = etherL(mag, -rotate, a, b);
		}
	} else{
		if (rotate>=0){
			
			left = -etherR(-mag, rotate, a, b);
			right = -etherL(-mag, rotate, a, b);
		} else{
			left = -etherL(-mag, -rotate, a, b);
			right = -etherR(-mag, -rotate, a, b);
		}
	}

	SetLeftRightMotorOutputs(left, right);	
}

/*
 * Arcade:
 * The CORE version of Arcade drive
 */
void COREDrive::ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs)
{
	// local variables to hold the computed PWM values for the motors
	float leftMotorOutput;
	float rightMotorOutput;

	moveValue = Limit(moveValue);
	rotateValue = Limit(rotateValue);

	if (squaredInputs)
	{
		// square the inputs (while preserving the sign) to increase fine control while permitting full power
		if (moveValue >= 0.0){			
			moveValue = (moveValue * moveValue);
		}else{
			moveValue = -(moveValue * moveValue);
		}
		
		if (rotateValue >= 0.0){
			rotateValue = (rotateValue * rotateValue);
		}else{
			rotateValue = -(rotateValue * rotateValue);
		}
	}

	
	if (moveValue > 0.0)
	{
		if (rotateValue > 0.0)
		{
			leftMotorOutput = moveValue - rotateValue;
			rightMotorOutput = std::max(moveValue, rotateValue);
		}
		else
		{
			leftMotorOutput = std::max(moveValue, -rotateValue);
			rightMotorOutput = moveValue + rotateValue;
		}
	}
	else
	{
		if (rotateValue > 0.0)
		{
			leftMotorOutput = - std::max(-moveValue, rotateValue);
			rightMotorOutput = moveValue + rotateValue;
		}
		else
		{
			leftMotorOutput = moveValue - rotateValue;
			rightMotorOutput = - std::max(-moveValue, -rotateValue);
		}
	}
	SetLeftRightMotorOutputs(leftMotorOutput, rightMotorOutput);
}

#include <cmath>

#undef __AlternateRawImplementation__

inline double sgn(double x)
{
	return (x>=0)?1:-1;
}

const double PI=3.14159265358979323846;
const double PI2=PI*2.0;
const double PI_2=PI/2.0;

inline double DEG_2_RAD(double x) 
{	
	return	((x)*PI/180.0); 
}
inline double RAD_2_DEG(double x) 
{
	return ((x)*180.0/PI); 
}

//INTERPOLATION CURVES (CONST DATA)

//Since these go past 90 I'm assuming it intended to steer past 90 on the wheel and filter to 115 from 1 to 0 -- USED for linear interp
//const double c_radius_theta_x[] = {0, 90, 115, 180};
//const double c_radius_theta_z[] = {1,  1,   0,   0};

const double c_half_pi_reciprocal=1.0 / (PI_2);  //its more efficient to multiply than to divide... this is around 0.6366...
const double c_taper_limit=DEG_2_RAD(115.0);
const double c_taper_length_recip=1.0 / (c_taper_limit-PI_2);

void COREDrive::CulverDrive(float throttle, float steer_x, float steer_y, bool quickturn,
		double gain_radius, double gain_raw ) {
	
		//Find arctan of wheel stick relative to vertical... note relative to vertical suggests that we swap the x and y components!
	double theta = atan2(steer_x, -steer_y);

	//Find the magnitude of the wheel stick
	double r = sqrt(((steer_y * steer_y) + (steer_x * steer_x)));

	//taper off past 90 - 115 using simple linear interpolation 
	double radius_filter = 1.0;
//	cout << "abstheta " << std::abs(theta) << endl;
	if (std::abs(theta) > PI_2) {
		const double abs_theta = std::abs(theta);
		if (abs_theta < c_taper_limit)
			radius_filter = (1.0 - ((abs_theta - PI_2) * c_taper_length_recip));
		else
			radius_filter = 0;
	}
//	cout << "radiusfilter "<< radius_filter <<endl;
	
//	SmartDashboard::PutNumber("culver-mag", throttle);
//	SmartDashboard::PutNumber("s-x", steer_x);
//	SmartDashboard::PutNumber("s-y", -steer_y);
//	SmartDashboard::PutNumber("s-r", r);
//	SmartDashboard::PutNumber("theta", RAD_2_DEG(theta));

	double left = throttle;
	double right = throttle;
//	cout << left<< " : "<<right;
	if (quickturn) {
		const double raw = r * theta * c_half_pi_reciprocal * radius_filter * gain_raw * -1;
		//		const double raw = r * interp_2d(theta, c_raw_theta_x, c_raw_theta_z, _countof(c_raw_theta_x)) * c_gain_raw);
//		SmartDashboard::PutNumber("culver-raw", raw);

		#if 1
			left += raw;
			right -= raw;
		#else
			const double radius = r * theta*c_half_pi_reciprocal * radius_filter * gain_radius * throttle;
			//Find the left and right powers as the sums - Use for Alternate Raw
			left += radius + raw;
			right -= radius + raw;
		#endif
	} else {
		//Find the radius component based on r, theta, filter curve, gain, throttle, sign
		const double radius = r * theta * c_half_pi_reciprocal * radius_filter
				* gain_radius * throttle;
		
//		SmartDashboard::PutNumber("culver-radius", radius);
		
		left += radius;
		right -= radius;
	}

	//clamp value range to +-1
//	SmartDashboard::PutNumber("c-left", left);
//	SmartDashboard::PutNumber("c-right", right);
	SetLeftRightMotorOutputs(left, right);
}
