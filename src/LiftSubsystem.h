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

	CANTalon liftMotor;
	Encoder liftEncoder;
	DigitalInput bottomLimit;
	DigitalInput topLimit;
	AnalogInput IRSensor;
	SendableChooser autoChoose;

	bool liftUpButton = false;
	bool liftDownButton = false;
	bool toteHeightButton;
	bool twoToteHeightButton;
	double liftValue = 0.0;
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
	}

	encoderLift, IRLift;

public:
	std::string name(void){
		return "lift";

}
	LiftSubsystem(CORERobot& robot):
			CORESubsystem(robot),
			liftMotor(21),
			liftEncoder(-1,-1),
			bottomLimit(-1),
			topLimit(-1),
			IRSensor(-1),
			autoChoose(),
			toteHeightButton(-1),
			twoToteHeightButton(-1)

{
		liftMotor.SetSafetyEnabled(true);
		liftMotor.SetSafetyEnabled(false);
		liftMotor.SetExpiration(0.1);
}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);
	double getLiftHeight(void);
	double getBufferValue(void);
	void setLift(double speed);
	void setPositionModeEnc(void);
	void setPositionModeIR(void);
	void setVoltageMode(void);
	double getIRLiftHeight(void);

};

	class LiftAction : public Action{
		LiftSubsystem* lift;
		double targetHeight;
		double currentHeight =0.0;
	public:
		LiftAction(LiftSubsystem& lift, double targetHeight):
			lift(&lift),
			targetHeight(targetHeight){

		}
		void init(void){
			currentHeight = lift->getLiftHeight();
			lift->setPositionModeEnc();
		}
		ControlFlow call(void){
			lift->setLift(targetHeight);
			return END;

		}
	};

	class IRLiftAction : public Action{
		LiftSubsystem* lift;
		double targetHeight;
		double currentHeight = 0.0;
	public:
		IRLiftAction(LiftSubsystem& lift, double targetHeight):
			lift(&lift),
			targetHeight(targetHeight){

		}
		void init(void){
			lift->setPositionModeIR();
			currentHeight = lift->getIRLiftHeight();

		}
		ControlFlow call(void){
				lift->setLift(targetHeight);
				return END;
		}
	};

#endif /* SRC_LIFTSUBSYSTEM_H_ */
;
