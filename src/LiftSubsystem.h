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

	Talon leftMotor;
	Talon rightMotor;
	DigitalInput topLimit;
	DigitalInput bottomLimit;
public:
	std::string name(void){
		return "lift";

}
	LiftSubsystem(CORERobot& robot):
			CORESubsystem(robot),
			leftMotor(-1),
			rightMotor(-1),
			topLimit(-1),
			bottomLimit(-1)
{}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);













};



#endif /* SRC_LIFTSUBSYSTEM_H_ */
