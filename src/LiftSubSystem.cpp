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
	liftChoose.AddDefault("IRLift", new std::string ("use-IR-Lift"));
	liftChoose.AddObject("EncoderLift", new std::string ("use-Encoder-Lift"));
	liftChoose.AddObject("LLLE", new std::string("use-three-limit-switches-one-encoder"));
	liftChoose.AddObject("LLLL", new std::string("use-four-limit-switches"));
	SmartDashboard::PutData("lift-chooser", &liftChoose);
	robot.outLog.throwLog("LiftSubsystem: RobotInit Success");
}
void LiftSubsystem::teleopInit(void){
	liftMotor.SetSafetyEnabled(true);
	robot.outLog.throwLog("LiftSubsystem: TeleopInit Success");

	robot.joystick.register_button("liftUpButton", 2, 1);
	robot.joystick.register_button("liftDownButton", 2, 2);
	robot.joystick.register_button("twoToteHeightButton", 1, 3);
	robot.joystick.register_button("toteHeightButton", 1, 4);

	SmartDashboard::PutBoolean("liftStart", true);


}
void LiftSubsystem::teleop(void){

	std::string liftMode = *(std::string*)liftChoose.GetSelected();

//	encoderLift.location = encoder.Get();
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
			logLLLE = false;
			logLLLL = false;
		}
		if(bottomLimit.Get()){
			encoder.Reset();
		}
		if (liftUpButton == true && topLimit.Get() == true){
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(0.5);
		}else if (liftDownButton == true && bottomLimit.Get() == true){
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(-0.5);
		}else if (twoToteHeightButton == true){
//			if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
	//			liftMotor.SetFeedbackDevice(CANTalon::EncRising);
	//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
	//			liftMotor.SetControlMode(CANSpeedController::kPosition);
	//		}
			liftMotor.Set(encoderLift.twoToteHeight);
		}else if (toteHeightButton == true){
//			if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
	//			liftMotor.SetFeedbackDevice(CANTalon::EncRising);
	//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
	//			liftMotor.SetControlMode(CANSpeedController::kPosition);
	//		}
			liftMotor.Set(0);
		}else{
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(0.0);
		}
	}else if (liftMode == "use-IR-Lift"){
		if(logIR == false){
			logENC = false;
			logLLLE = false;
			logLLLL = false;
			logIR = true;

		}
		if (liftUpButton == true && topLimit.Get() == true){

			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(0.5);
		}else if (liftDownButton == true && bottomLimit.Get() == true){
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(-0.5);
		}else if (twoToteHeightButton == true){
//			if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
	//			liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
	//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	//			liftMotor.SetPID(IRLift.P, IRLift.I, IRLift.D);
	//			liftMotor.SetControlMode(CANSpeedController::kPosition);
	//		}
			liftMotor.Set(IRLift.twoToteHeight);
		}else if (toteHeightButton == true){
	//		if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
	//			liftMotor.SetFeedbackDevice(CANTalon::AnalogPot);
	//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	//			liftMotor.SetPID(IRLift.P, IRLift.I, IRLift.D);
	//			liftMotor.SetControlMode(CANSpeedController::kPosition);
	//		}
			liftMotor.Set(IRLift.toteHeight);
		}else{
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(0.0);
		}

	}else if (liftMode == "use-three-limit-switches-one-encoder"){
		if(logLLLE == false){
			logENC = false;
			logIR = false;
			logLLLL = false;
			logLLLE = true;
		}
		if(middleLimit.Get()){
			encoder.Reset();
		}
		if (liftUpButton == true && topLimit.Get() == true){
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(0.5);
		}else if (liftDownButton == true && bottomLimit.Get() == true){
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
			}
			liftMotor.Set(-0.5);
		}else if (twoToteHeightButton == true){
//			if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
	//			liftMotor.SetFeedbackDevice(CANTalon::EncRising);
	//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
	//			liftMotor.SetControlMode(CANSpeedController::kPosition);
	//		}
			liftMotor.Set(encoderLift.twoToteHeight);
		}else if (toteHeightButton == true){
//			if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
	//			liftMotor.SetFeedbackDevice(CANTalon::EncRising);
	//			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
	//			liftMotor.SetPID(encoderLift.P, encoderLift.I, encoderLift.D);
	//			liftMotor.SetControlMode(CANSpeedController::kPosition);
	//		}
			liftMotor.Set(encoderLift.toteHeight);
		}else{
			if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
				liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
				liftMotor.Set(0.0);
			}
		}
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
