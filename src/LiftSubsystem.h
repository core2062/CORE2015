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

	CANJaguar liftMotor;
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
	double topHeight = -1.0;
	double bottomHeight = 0.0;
	double toteHeight = 0.0;
	double twoToteHeight = 0.0;
	double buffer = 0.0;
	double ticksPerRotation = 200;
	double IRtoteHeight = 0.0;
	double IRtwoToteHeight = 0.0;
	double liftPValue = 0.0;
	double liftIValue = 0.0;
	double liftDValue = 0.0;
	double IRliftPValue = 0.0;
	double IRliftIValue = 0.0;
	double IRliftDValue= 0.0;
	double topHeight = -1.0;
	double bottomHeight = 0.0;
	double buffer;
	double ticksPerRotation = 200;


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
	void setLiftSpeed(double speed);
};

	class LiftAction : public Action{
		LiftSubsystem* lift;
		double speed;
		double targetHeight;
		double currentHeight =0.0;
	public:
		LiftAction(LiftSubsystem& lift, double speed, double targetHeight):
			lift(&lift),
			speed(speed),
			targetHeight(targetHeight){

		}
		void init(void){
			currentHeight = lift->getLiftHeight();
		}
		ControlFlow call(void){
			currentHeight = lift->getLiftHeight();
			if(currentHeight<targetHeight- lift->getBufferValue()){
				lift->setLiftSpeed(speed);
				return CONTINUE;
			}else if(currentHeight>targetHeight + lift->getBufferValue()){
				lift->setLiftSpeed(-speed);
				return CONTINUE;
			}else{
				lift->setLiftSpeed(0.0);
				return END;
			}
		}
	};
















#endif /* SRC_LIFTSUBSYSTEM_H_ */
