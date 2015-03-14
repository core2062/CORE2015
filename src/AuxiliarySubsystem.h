/*
 * AuxiliarySubsystem.h
 *
 *  Created on: Mar 10, 2015
 *      Author: core
 */

#ifndef SRC_AuxiliarySUBSYSTEM_H_
#define SRC_AuxiliarySUBSYSTEM_H_

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
#include "Subsystems.h"

using namespace CORE;

class AuxiliarySubsystem: public CORESubsystem {
// varriabbles and other privites

public:
	std::string name(void) {
		return "Auxiliary";

	}
	AuxiliarySubsystem(CORERobot& robot) :
		CORESubsystem(robot)

			{
//		liftMotor.Set(0.0);
//		liftMotor.SetSafetyEnabled(false);
//		liftMotor.SetExpiration(0.125);
//		liftMotor.SetSensorDirection(true);
	}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);
	void teleopEnd(void);
	void giveLog(std::string stringVar);

};

#endif /* SRC_AuxiliarySUBSYSTEM_H_ */
