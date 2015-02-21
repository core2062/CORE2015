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

class LiftSubsystem: public CORESubsystem {


	Encoder encoder;
	DigitalInput bottomLimit;
	DigitalInput middleLimit;
	DigitalInput topLimit;

	bool bottomHeightButton = false;
	bool toteHeightButton = false;
	bool twoToteHeightButton = false;
	bool topLatch = false;
	bool bottomLatch = false;
	double liftAxis = 0.0;
	double liftValue = 0.0;
	double buffer = 0.0;
	double ticksPerRotation = 1024;
	double bottomHeight = 1200;
	double toteHeight = 0.0;
	double twoToteHeight = 0.0;
	double location = 0.0;
	double P = 0.0;
	double I = 0.0;
	double D = 0.0;
	double Pu = 0.0;
	double Iu = 0.0;
	double Du = 0.0;
	double Pd = 0.0;
	double Id = 0.0;
	double Dd = 0.0;
	bool beenSet = false;


	struct{
		double P = 0.1;
		double I = 0.001;
		double D = 0.0;
		double mistake;
		double actualPosition;
		double lastError;
		double integral;
		double derivative;
		double setPoint = 0.0;
		bool enabled = false;
		}encoderPID;

public:
	std::string name(void) {
		return "lift";

	}
	CANTalon liftMotor;
	LiftSubsystem(CORERobot& robot) :
			CORESubsystem(robot),
			encoder(9, 10),
			bottomLimit(0),
			middleLimit(-1),
			topLimit(1),
			liftMotor(14)
			{
		liftMotor.Set(0.0);
		liftMotor.SetSafetyEnabled(false);
		liftMotor.SetExpiration(0.1);
		liftMotor.SetSensorDirection(true);
	}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);
	void teleopEnd(void);
	double getLiftHeight(void);
	double getBufferValue(void);
	void setLift(double speed);
	void setPositionModeEnc(void);
	void setVoltageMode(void);
	void giveLog(std::string stringVar);
	double liftPID(void);
	void setPID(double setPoint);

};

class LiftAction: public Action {
	LiftSubsystem* lift;
	double targetHeight;
	double currentHeight = 0.0;
	bool background = false;
public:
	LiftAction(LiftSubsystem& lift, double targetHeight, bool background = false) :
			lift(&lift), targetHeight(targetHeight){
	}
	void init(void) {
		currentHeight = lift->getLiftHeight();
		lift->setPositionModeEnc();
	}
	ControlFlow call(void) {
		if (background){
			lift->giveLog("ENCLiftAction Completed");
			lift->setLift(targetHeight);
			return BACKGROUND;
		} else {
			lift->giveLog("ENCLiftAction Completed");
			lift->setLift(targetHeight);
			return END;
		}

	}
};


#endif /* SRC_LIFTSUBSYSTEM_H_ */
;
