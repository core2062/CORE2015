/*
 * LiftSubSystem.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */
#include "LiftSubsystem.h"
const double MOTORUPDATEFREQUENCY = 0.005;
struct{
	const double P = 0.1;
	const double I = 0.001;
	const double D = 0.0;
	double location;
	double toteHeight = 0;
	double twoToteHeight = 0;
	double completionTolerance = 0; //Stop PID and enable brake if lift position is within +- completionTolerance
}encoderLift;

struct{
	const double P = 0.1;
	const double I = 0.001;
	const double D = 0.0;
	double mistake;
	double actualPosition;
	double lastError;
	double integral;
	double derivative;
	double setPoint;
	bool enabled = false;
	double location;
	double toteHeight = 0;
	double twoToteHeight = 0;
	double completionTolerance = 0; //Stop PID and enable brake if lift position is within +- completionTolerance
}IRLift;

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

	encoderLift.location = liftEncoder.Get();
	IRLift.location = IRSensor.GetVoltage();
	liftUpButton = robot.joystick.button("liftUpButton");
	liftDownButton = robot.joystick.button("liftDownButton");

	std::string liftMode = *(std::string*)autoChoose.GetSelected();

	double toteHeight = SmartDashboard::GetNumber("toteHeight");
	double twoToteHeight = SmartDashboard::GetNumber("twoToteHeight");
	double IRtoteHeight = SmartDashboard::GetNumber("IRtoteHeight");
	double IRtwoToteHeight = SmartDashboard::GetNumber("IRtwoToteHeight");
	encoderLift.P = SmartDashboard::GetNumber("Lift-P-Value");
	encoderLift.I = SmartDashboard::GetNumber("Lift-I-Value");
	encoderLift.D = SmartDashboard::GetNumber("Lift-D-Value");
	IRLift.P = SmartDashboard::GetNumber("IR-Lift-P-Value");
	IRLift.I = SmartDashboard::GetNumber("IR-Lift-I-Value");
	IRLift.D = SmartDashboard::GetNumber("IR-Lift-D-Value");
	SmartDashboard::PutNumber("liftEncoderValue", encoderLift.location);
	SmartDashboard::PutNumber("IR-Sensor-Value", IRLift.location);

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
			liftMotor.SetPositionMode(CANJaguar::QuadEncoder,ticksPerRotation, encoderLift.P, encoderLift.I, encoderLift.D);
			liftMotor.Set(encoderLift.twoToteHeight);
		}else if (toteHeightButton == true){
			liftMotor.SetPositionMode(CANJaguar::QuadEncoder,ticksPerRotation, encoderLift.P, encoderLift.I, encoderLift.D);
			liftMotor.Set(encoderLift.toteHeight);
		}else{
			liftMotor.Set(0.0);
		}
	}else if (liftMode == "use-IR-Lift"){

			if (liftUpButton == true && topLimit.Get() != true){
				IRLift.enabled = false;
				liftMotor.Set(1.0);
			}else if (liftDownButton == true && bottomLimit.Get() != true){
				IRLift.enabled = false;
				liftMotor.Set(-1.0);
			}else if (twoToteHeightButton == true){
				IRLift.enabled = true;
				IRLift.setPoint = IRLift.twoToteHeight;
			}else if (toteHeightButton == true){
				IRLift.enabled = true;
				IRLift.setPoint = IRLift.toteHeight;
			}else{
				liftMotor.Set(0.0);
			}
	}
	if(IRLift.enabled && !(twoToteHeightButton || toteHeightButton) && (IRLift.setPoint - IRLift.completionTolerance <= IRLift.location && IRLift.setPoint + IRLift.completionTolerance >= IRLift.location))
	{
	 IRLift.enabled = false;
	 liftMotor.Set(0.0);
	 //Enable Brake
	}
	else if(IRLift.enabled)
	{
		//Disable Brake
		IRLift.mistake = IRLift.setPoint - IRLift.location;
		IRLift.integral = IRLift.integral + (IRLift.mistake * MOTORUPDATEFREQUENCY);
		IRLift.derivative = (IRLift.mistake - IRLift.lastError) * (1/MOTORUPDATEFREQUENCY);
		double output = (IRLift.P*IRLift.mistake) + (IRLift.I*IRLift.integral) + (IRLift.D*IRLift.derivative);
		output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
		liftMotor.Set(output);
		IRLift.lastError = IRLift.mistake;
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

