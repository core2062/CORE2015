#ifndef COREDRIVE_H
#define COREDRIVE_H

#include "WPILib.h"

namespace CORE {

class DoubleSpeed : public SpeedController {
public:
	SpeedController& one;
	SpeedController& two;
	DoubleSpeed(SpeedController& one, SpeedController& two):
		one(one),
		two(two)
	{}
	
	virtual void Set(float speed, uint8_t syncGroup=0){
		one.Set(speed, syncGroup);
		two.Set(speed, syncGroup);
	}
	
	virtual float Get(){
		float n = 0;
		n += one.Get();
		n += two.Get();
		return n/2;
	}
	
	virtual void Disable(){
		one.Disable();
		two.Disable();
	}
	
	virtual void PIDWrite(float output){
		Set(output);
	}

};

class COREDrive : public RobotDrive {
public:
	COREDrive(SpeedController &leftMotor, SpeedController &rightMotor):
			RobotDrive(leftMotor, rightMotor){
			
			}
	
	void EtherArcade(double mag, double rotate, double a, double b);
	void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs = false);
	void CulverDrive(float throttle, float steer_x, float steer_y, bool quickturn,
		double gain_radius, double gain_raw );
};

class CORERateLimiter{
	float increment;
	float old;
public:

	CORERateLimiter(float incremen){
		old = 0;
		increment = incremen;
	}
	
	float limit(float input){
		float diff = input - old;
		
		if(diff < 0){
			if(diff < -increment){
				old -= increment;	
			}else{
				old = input;
			}
		} else if( diff > 0){
			if(diff > increment){
				old += increment;
			} else {
				old = input;
			}
		}
		return old;
	}
};

}
#endif
