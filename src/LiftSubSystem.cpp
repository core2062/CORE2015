/*
 * LiftSubSystem.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */
#include "LiftSubsystem.h"



void LiftSubsystem::robotInit(void){
	liftEncoder.Reset();
	autoChoose.AddDefault("EncoderLift", new std::string ("use-Encoder-Lift"));
	autoChoose.AddDefault("IRLift", new std::string ("use-IR-Lift"));
	SmartDashboard::PutData("auto-chooser", &autoChoose);
}
void LiftSubsystem::teleopInit(void){


	robot.joystick.register_button("liftUpButton", 1, 1);
	robot.joystick.register_button("liftDownButton", 1, 2);
	robot.joystick.register_button("twoToteHeightButton", 1, 3);
	robot.joystick.register_button("toteHeightButton", 1, 4);
	SmartDashboard::PutBoolean("liftStart", true);


}
void LiftSubsystem::teleop(void){

	liftValue = liftEncoder.Get();
	IRliftValue = IRSensor.GetVoltage();
	liftUpButton = robot.joystick.button("liftUpButton");
	liftDownButton = robot.joystick.button("liftDownButton");

	std::string liftMode = *(std::string*)autoChoose.GetSelected();

	double toteHeight = SmartDashboard::GetNumber("toteHeight");
	double twoToteHeight = SmartDashboard::GetNumber("twoToteHeight");
	double IRtoteHeight = SmartDashboard::GetNumber("IRtoteHeight");
	double IRtwoToteHeight = SmartDashboard::GetNumber("IRtwoToteHeight");
	double liftPValue = SmartDashboard::GetNumber("Lift-P-Value");
	double liftIValue = SmartDashboard::GetNumber("Lift-I-Value");
	double liftDValue = SmartDashboard::GetNumber("Lift-D-Value");
	double IRliftPValue = SmartDashboard::GetNumber("IR-Lift-P-Value");
	double IRliftIValue = SmartDashboard::GetNumber("IR-Lift-I-Value");
	double IRliftDValue = SmartDashboard::GetNumber("IR-Lift-D-Value");
	SmartDashboard::PutNumber("liftEncoderValue", liftValue);
	SmartDashboard::PutNumber("IR-Sensor-Value", IRliftValue);

	if(liftMode == "use-Encoder-Lift"){
		if(bottomLimit.Get()){
			liftEncoder.Reset();
		}
		if (liftUpButton == true && topLimit.Get() != true){
			liftMotor.SetVoltageMode();
			liftMotor.Set(1.0);
		}else if (liftDownButton == true && bottomLimit.Get() != true){
			liftMotor.SetVoltageMode();
			liftMotor.Set(-1.0);
		}else if (twoToteHeightButton == true){
			liftMotor.SetPositionMode(CANJaguar::QuadEncoder,ticksPerRotation, liftPValue, liftIValue, liftDValue);
			liftMotor.Set(twoToteHeight);
		}else if (toteHeightButton == true){
			liftMotor.SetPositionMode(CANJaguar::QuadEncoder,ticksPerRotation, liftPValue, liftIValue, liftDValue);
			liftMotor.Set(toteHeight);
		}else{
			liftMotor.Set(0.0);
		}
	}else if (liftMode == "use-IR-Lift"){

			if (liftUpButton == true && topLimit.Get() != true){
				liftMotor.SetVoltageMode();
				liftMotor.Set(1.0);
			}else if (liftDownButton == true && bottomLimit.Get() != true){
				liftMotor.SetVoltageMode();
				liftMotor.Set(-1.0);
			}else if (twoToteHeightButton == true){
				liftMotor.SetPositionMode(CANJaguar::QuadEncoder,ticksPerRotation, IRliftPValue, IRliftIValue, IRliftDValue);
				liftMotor.Set(IRtwoToteHeight);
			}else if (toteHeightButton == true){
				liftMotor.SetPositionMode(CANJaguar::QuadEncoder,ticksPerRotation, IRliftPValue, IRliftIValue, IRliftDValue);
				liftMotor.Set(IRtoteHeight);
			}else{
				liftMotor.Set(0.0);
			}
	}
}
double LiftSubsystem::getLiftHeight(void)
{
	return liftEncoder.Get();
}
double LiftSubsystem::getBufferValue(void){
	return buffer;
}
void LiftSubsystem::setLiftSpeed(double speed){
	liftMotor.Set(speed);
}

