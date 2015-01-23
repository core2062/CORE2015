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
	Encoder liftEncoder;
	DigitalInput bottomLimit;
	DigitalInput topLimit;


	bool liftUpButton;
	bool liftDownButton;
	bool toteHeightButton;
	bool twoToteHeightButton;
	double liftValue;
	double topHeight = -1.0;
	double bottomHeight = 0.0;
	double toteHeight;
	double twoToteHeight;
	double buffer;
public:
	std::string name(void){
		return "lift";

}
	LiftSubsystem(CORERobot& robot):
			CORESubsystem(robot),
			leftMotor(21),
			rightMotor(20),
			bottomLimit(-1),
			topLimit(-1),
			toteHeightButton(-1),
			twoToteHeightButton(-1),
			liftEncoder(-1,-1)
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
