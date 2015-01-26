/*
 * LiftSubSystem.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */
#include "LiftSubsystem.h"



void LiftSubsystem::robotInit(void){
	liftEncoder.Reset();
}
void LiftSubsystem::teleopInit(void){


	robot.joystick.register_button("liftUpButton", 1, 1);
	robot.joystick.register_button("liftDownButton", 1, 2);
	robot.joystick.register_button("twoToteHeightButton", 1, 3);
	robot.joystick.register_button("toteHeightButton", 1, 4);
	SmartDashboard::PutBoolean("liftStart", true);


}
void LiftSubsystem::teleop(void){

	liftValue = liftEncoder.Get();
	liftUpButton = robot.joystick.button("liftUpButton");
	liftDownButton = robot.joystick.button("liftDownButton");
	double toteHeight = SmartDashboard::GetNumber("toteHeight");
	double twoToteHeight = SmartDashboard::GetNumber("twoToteHeight");
	double buffer = SmartDashboard::GetNumber("buffer");
	SmartDashboard::PutNumber("liftEncoderValue", liftValue);

	if(bottomLimit.Get()){
		liftEncoder.Reset();
	}
	if (liftUpButton == true && topLimit.Get() != true){
		liftMotor.Set(1.0);
	}else if (liftDownButton == true && bottomLimit.Get() != true){
		liftMotor.Set(-1.0);
	}else if (twoToteHeightButton == true){
			if(liftValue > twoToteHeight + buffer){
				liftMotor.Set(-1.0);
			}else if(liftValue < twoToteHeight - buffer){
				liftMotor.Set(1.0);
			}else{
				liftMotor.Set(0.0);
			}
	}else if (toteHeightButton == true){
			if(liftValue > toteHeight + buffer){
				liftMotor.Set(-1.0);
			}else if(liftValue < toteHeight - buffer){
				liftMotor.Set(1.0);
			}else{
				liftMotor.Set(0.0);
			}
	}else{
		liftMotor.Set(0.0);
	}

}
double LiftSubsystem::getLiftHeight(void)
{
	return liftEncoder.Get();
}
double LiftSubsystem::getBufferValue(void){
	return buffer;
}
void LiftSubsystem::setLiftSpeed(double speed){
	liftMotor.Set(speed);
}

