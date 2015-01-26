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
			liftMotor(21),
			liftEncoder(-1,-1),
			bottomLimit(-1),
			topLimit(-1),
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
		double currentHeight;
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
