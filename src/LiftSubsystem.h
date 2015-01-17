/*
 * LiftSubsystem.h
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */

#ifndef SRC_LIFTSUBSYSTEM_H_
#define SRC_LIFTSUBSYSTEM_H_

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
using namespace CORE;

class LiftSubsystem : public CORESubsystem{

	CANJaguar leftMotor;
	CANJaguar rightMotor;
//	DigitalInput topLimit;
//	DigitalInput bottomLimit;
	bool liftUpButton;
	bool liftDownButton;
public:
	std::string name(void){
		return "lift";

}
	LiftSubsystem(CORERobot& robot):
			CORESubsystem(robot),
			leftMotor(21),
			rightMotor(20)//,
			//topLimit(-1),
			//bottomLimit(-1)
{
		leftMotor.SetSafetyEnabled(true);
		leftMotor.SetSafetyEnabled(false);
		leftMotor.SetExpiration(0.1);
		rightMotor.SetSafetyEnabled(true);
		rightMotor.SetSafetyEnabled(false);
		rightMotor.SetExpiration(0.1);
}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);













};



#endif /* SRC_LIFTSUBSYSTEM_H_ */
