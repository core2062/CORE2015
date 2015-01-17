/*
 * LiftSubSystem.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */
#include "LiftSubsystem.h"



void LiftSubsystem::robotInit(void){


}
void LiftSubsystem::teleopInit(void){


	robot.joystick.register_button("liftUpButton", 1, 1);
	robot.joystick.register_button("liftDownButton", 1, 2);
	SmartDashboard::PutBoolean("liftStart", true);

}
void LiftSubsystem::teleop(void){

	liftUpButton = robot.joystick.button("liftUpButton");
	liftDownButton = robot.joystick.button("liftDownButton");
	SmartDashboard::PutBoolean("luB", liftUpButton);
	if (liftUpButton == true /*&& !topLimit.Get()*/){
		rightMotor.Set(1.0);
		leftMotor.Set(1.0);
		SmartDashboard::PutNumber("testmotor", 1);
	}
	else if (liftDownButton == true/* && !bottomLimit.Get()*/){
		rightMotor.Set(-1.0);
		leftMotor.Set(-1.0);
		SmartDashboard::PutNumber("testmotor", -1);
		}
	else{
		rightMotor.Set(0.0);
		leftMotor.Set(0.0);
		SmartDashboard::PutNumber("testmotor", 0);
	}

}



