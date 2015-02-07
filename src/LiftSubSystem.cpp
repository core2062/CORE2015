/*
 * LiftSubSystem.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */
#include "LiftSubsystem.h"
const double MOTORUPDATEFREQUENCY = 0.005;


void LiftSubsystem::robotInit(void){
	robot.outLog.throwLog("LiftSubsystem: RobotInit Start");
	autoChoose.AddObject("EncoderLift", new std::string ("use-Encoder-Lift"));
	autoChoose.AddDefault("IRLift", new std::string ("use-IR-Lift"));
	SmartDashboard::PutData("auto-chooser", &autoChoose);
	robot.outLog.throwLog("LiftSubsystem: RobotInit Success");

}
void LiftSubsystem::teleopInit(void){
	robot.outLog.throwLog("LiftSubsystem: TeleopInit Success");

	robot.joystick.register_button("liftUpButton", 2, 1);
	robot.joystick.register_button("liftDownButton", 2, 2);
	robot.joystick.register_button("twoToteHeightButton", 1, 3);
	robot.joystick.register_button("toteHeightButton", 1, 4);

	SmartDashboard::PutBoolean("liftStart", true);


}
void LiftSubsystem::teleop(void){

	std::string liftMode = *(std::string*)autoChoose.GetSelected();

//	encoderLift.location = liftEncoder.Get();
//	IRLift.location = IRSensor.GetVoltage();
	liftUpButton = robot.joystick.button("liftUpButton");
	liftDownButton = robot.joystick.button("liftDownButton");
	encoderLift.toteHeight = SmartDashboard::GetNumber("toteHeight");
	encoderLift.twoToteHeight = SmartDashboard::GetNumber("twoToteHeight");
	IRLift.toteHeight = SmartDashboard::GetNumber("IRtoteHeight");
	IRLift.twoToteHeight = SmartDashboard::GetNumber("IRtwoToteHeight");
	encoderLift.P = SmartDashboard::GetNumber("Lift-P-Value");
	encoderLift.I = SmartDashboard::GetNumber("Lift-I-Value");
	encoderLift.D = SmartDashboard::GetNumber("Lift-D-Value");
	IRLift.P = SmartDashboard::GetNumber("IR-Lift-P-Value");
	IRLift.I = SmartDashboard::GetNumber("IR-Lift-I-Value");
	IRLift.D = SmartDashboard::GetNumber("IR-Lift-D-Value");
	SmartDashboard::PutBoolean("liftUpButton", liftUpButton);
	SmartDashboard::PutBoolean("liftDownButton", liftDownButton);
	SmartDashboard::PutBoolean("botLimit", bottomLimit.Get());
	SmartDashboard::PutBoolean("topLimit", topLimit.Get());
//	SmartDashboard::PutNumber("liftEncoderValue", encoderLift.location);
//	SmartDashboard::PutNumber("IR-Sensor-Value", IRLift.location);

	if(liftMode == "use-Encoder-Lift"){
		if(logENC == false){
			logENC = true;
			logIR = false;
		}
		if(bottomLimit.Get()){
			liftEncoder.Reset();
		}
		if (liftUpButton == true && topLimit.Get() == true){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			liftMotor.Set(0.5);
		}else if (liftDownButton == true && bottomLimit.Get() == true){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			liftMotor.Set(-0.5);
		}else if (twoToteHeightButton == true){
//			liftMotor.SetControlMode(CANSpeedController::kPosition);
//			liftMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
			liftMotor.Set(encoderLift.twoToteHeight);
		}else if (toteHeightButton == true){
//			//liftMotor.SetControlMode(CANSpeedController::kPosition);
//			liftMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
			liftMotor.Set(encoderLift.toteHeight);
		}else{
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			liftMotor.Set(0.0);
		}
	}else if (liftMode == "use-IR-Lift"){
		if(logIR == false){
			logENC = false;
			logIR = true;
		}
		if (liftUpButton == true && topLimit.Get() == true){

			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			liftMotor.Set(0.5);
		}else if (liftDownButton == true && bottomLimit.Get() == true){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			liftMotor.Set(-0.5);
		}else if (twoToteHeightButton == true){
//			//liftMotor.SetControlMode(CANSpeedController::kPosition);
//			liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
			liftMotor.Set(IRLift.twoToteHeight);
		}else if (toteHeightButton == true){
			if(liftMotor.GetControlMode() == CANSpeedController::kPosition){

			}
			//			//liftMotor.SetControlMode(CANSpeedController::kPosition);
//			liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
			liftMotor.Set(IRLift.toteHeight);
		}else{
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			liftMotor.Set(0.0);
		}
	}

}
double LiftSubsystem::getLiftHeight(void)
{
	return liftMotor.GetPosition();
}
double LiftSubsystem::getBufferValue(void){
	return buffer;
}
void LiftSubsystem::setLift(double speed){
	liftMotor.Set(speed);
}
void LiftSubsystem::setPositionModeEnc(void){
	//liftMotor.SetControlMode(CANSpeedController::kPosition);
	liftMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
	liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);

}
void LiftSubsystem::setPositionModeIR(void){
	//liftMotor.SetControlMode(CANSpeedController::kPosition);
	liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
	liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
}
void LiftSubsystem::setVoltageMode(void){
	//liftMotor.SetControlMode(CANSpeedController::kPercentVbus);

}
double LiftSubsystem::getIRLiftHeight(void){
	return IRSensor.GetVoltage();
}
void LiftSubsystem::giveLog(std::string stringVar){
	robot.outLog.throwLog(stringVar);
}
