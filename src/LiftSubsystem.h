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

	CANTalon liftMotor;
	Encoder encoder;
	DigitalInput bottomLimit;
	DigitalInput middleLimit;
	DigitalInput topLimit;
	AnalogInput IRSensor;
	SendableChooser liftChoose;

	bool liftUpButton = false;
	bool liftDownButton = false;
	bool toteHeightButton = false;
	double liftAxis = 0.0;
	double liftValue = 0.0;
	bool twoToteHeightButton = false;
	bool topLatch = false;
	bool bottomLatch = false;
	bool logIR = false;
	bool logENC = false;
	bool logLLLE = false;
	bool logLLLL = false;
	double IRliftValue = 0.0;
	double buffer = 0.0;
	double ticksPerRotation = 200;
	struct {
		double P = 0.0;
		double I = 0.0;
		double D = 0.0;
		double toteHeight = 0.0;
		double twoToteHeight = 0.0;
		double location = 0.0;
	}encoderLift, IRLift;

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
	LiftSubsystem(CORERobot& robot) :
			CORESubsystem(robot),
			liftMotor(13),
			encoder(9, 10),
			bottomLimit(0),
			middleLimit(-1),
			topLimit(1),
			IRSensor(1),
			liftChoose()

	{
		liftMotor.SetSafetyEnabled(false);
		liftMotor.SetExpiration(0.1);
		liftMotor.Set(0.0);
	}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);
	void teleopEnd(void);
	double getLiftHeight(void);
	double getBufferValue(void);
	void setLift(double speed);
	void setPositionModeEnc(void);
	void setPositionModeIR(void);
	void setVoltageMode(void);
	double getIRLiftHeight(void);
	void giveLog(std::string stringVar);
	double liftPID(void);

};

class LiftAction: public Action {
	LiftSubsystem* lift;
	double targetHeight;
	double currentHeight = 0.0;
public:
	LiftAction(LiftSubsystem& lift, double targetHeight) :
			lift(&lift), targetHeight(targetHeight){

	}
	void init(void) {
		currentHeight = lift->getLiftHeight();
		lift->setPositionModeEnc();
	}
	ControlFlow call(void) {
		lift->giveLog("ENCLiftAction Completed");
		lift->setLift(targetHeight);
		return END;

	}
};

class IRLiftAction: public Action {
	LiftSubsystem* lift;
	double targetHeight;
	double currentHeight = 0.0;
public:
	IRLiftAction(LiftSubsystem& lift, double targetHeight) :
			lift(&lift), targetHeight(targetHeight) {

	}
	void init(void) {
		lift->setPositionModeIR();
		currentHeight = lift->getIRLiftHeight();

	}
	ControlFlow call(void) {
		lift->giveLog("IRLiftAction Completed");
		lift->setLift(targetHeight);
		return END;
	}
};

#endif /* SRC_LIFTSUBSYSTEM_H_ */
;
