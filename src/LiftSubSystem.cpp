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
	robot.outLog.throwLog("LiftSubsystem: RobotInit Success");
}
void LiftSubsystem::teleopInit(void){
	liftMotor.SetSafetyEnabled(true);
	robot.outLog.throwLog("LiftSubsystem: TeleopInit Success");
	robot.joystick.register_button("twoToteHeightButton", 1, 3);
	robot.joystick.register_button("toteHeightButton", 1, 4);
	robot.joystick.register_button("FlagButton", 2 , 1);
	robot.joystick.register_axis("liftAxis", 2, 1);

	SmartDashboard::PutBoolean("liftStart", true);

}
void LiftSubsystem::teleop(void){
	toteHeight = SmartDashboard::GetNumber("toteHeight");
	twoToteHeight = SmartDashboard::GetNumber("twoToteHeight");
	P = SmartDashboard::GetNumber("Lift-P-Value");
	I = SmartDashboard::GetNumber("Lift-I-Value");
	D = SmartDashboard::GetNumber("Lift-D-Value");
	SmartDashboard::PutBoolean("botLimit", bottomLimit.Get());
	SmartDashboard::PutBoolean("topLimit", topLimit.Get());
	SmartDashboard::PutBoolean("Top Latch", topLatch);
	SmartDashboard::PutBoolean("Bot Latch", bottomLatch);
	liftValue = robot.joystick.axis("liftAxis");

	if (liftMotor.GetStickyFaults() != 0){
		robot.outLog.throwLog("Lift Flag");
		robot.outLog.throwLog(liftMotor.GetStickyFaults());
		liftMotor.Get();
	}
	if (!bottomLimit.Get()){
		bottomLatch = true;
	}
	if (!topLimit.Get()){
		topLatch = true;
	}
	if (liftValue < 0.1 && liftValue > -.1){
		liftValue = 0;
	}


	if (liftValue > 0.0 && topLatch == false){

		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		bottomLatch = false;
		liftMotor.Set(liftValue * SmartDashboard::GetNumber("Lift-Speed"));
	}else if (liftValue < 0.0 && bottomLatch == false){
		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		topLatch = false;
		liftMotor.Set(liftValue * SmartDashboard::GetNumber("Lift-Speed"));
	}else if (twoToteHeightButton == true){
//			if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
//			liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
//			liftMotor.SetPID(IRLift.P, IRLift.I, IRLift.D);
//			liftMotor.SetControlMode(CANSpeedController::kPosition);
//		}
		liftMotor.Set(twoToteHeight);
	}else if (toteHeightButton == true){
//		if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
//			liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
//			liftMotor.SetPID(IRLift.P, IRLift.I, IRLift.D);
//			liftMotor.SetControlMode(CANSpeedController::kPosition);
//		}
		liftMotor.Set(toteHeight);
	}else{
		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		liftMotor.Set(0.0);
	}
}
void LiftSubsystem::teleopEnd(){
	liftMotor.SetSafetyEnabled(false);
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
	liftMotor.SetPID(P, I, D);

}
void LiftSubsystem::setVoltageMode(void){
	//liftMotor.SetControlMode(CANSpeedController::kPercentVbus);

}
void LiftSubsystem::giveLog(std::string stringVar){
	robot.outLog.throwLog(stringVar);
}
double LiftSubsystem::liftPID(void){
		//Disable Brake
		encoderPID.mistake = encoderPID.setPoint - encoder.Get();
		encoderPID.integral = encoderPID.integral + (encoderPID.mistake * .05);
		encoderPID.derivative = (encoderPID.mistake - encoderPID.lastError) * (1/.05);
		double output = (encoderPID.P*encoderPID.mistake) + (encoderPID.I*encoderPID.integral) + (encoderPID.D*encoderPID.derivative);
		output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
		encoderPID.lastError = encoderPID.mistake;
		return output;
}
