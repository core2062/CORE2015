/*
 * LiftSubSystem.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: core
 */
#include "LiftSubsystem.h"
const double MOTORUPDATEFREQUENCY = 0.005;


void LiftSubsystem::robotInit(void){
	robot.outLog.throwLog("LiftSubsystem: RobotInit");
}
void LiftSubsystem::teleopInit(void){
	liftMotor.SetSafetyEnabled(true);
	liftMotor.Set(0);
	liftMotor.SetExpiration(0.125);
	robot.outLog.throwLog("LiftSubsystem: TeleopInit Success");
	robot.joystick.register_button("bottomHeightButton", 2, 2);
	robot.joystick.register_button("twoToteHeightButton", 2, 4);
	robot.joystick.register_button("toteHeightButton", 2, 3);
	robot.joystick.register_button("stack",2,5);
//	robot.joystick.register_button("FlagButton", 2 , 1);
	robot.joystick.register_axis("liftAxis", 2, 1);
	robot.joystick.register_button("toteIn",2,1);

//	SmartDashboard::PutBoolean("liftStart", true);

}

void LiftSubsystem::setPID(double setPoint)
{
		Pu = SmartDashboard::GetNumber("Lift-P-Up-Value",0.0);
		Iu = SmartDashboard::GetNumber("Lift-I-Up-Value",0.0);
		Du = SmartDashboard::GetNumber("Lift-D-Up-Value",0.0);
		Pd = SmartDashboard::GetNumber("Lift-P-Down-Value",0.0);
		Id = SmartDashboard::GetNumber("Lift-I-Down-Value",0.0);
		Dd = SmartDashboard::GetNumber("Lift-D-Down-Value",0.0);
		if(liftMotor.GetControlMode() != CANSpeedController::kPosition){
			liftMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
			liftMotor.ConfigEncoderCodesPerRev(ticksPerRotation);
			liftMotor.SelectProfileSlot(0);
			liftMotor.SetPID(Pu, Iu, Du);
			liftMotor.SelectProfileSlot(1);
			liftMotor.SetPID(Pd, Id, Dd);
			liftMotor.SetControlMode(CANSpeedController::kPosition);
			liftMotor.SetSensorDirection(true);
		}
		liftMotor.SelectProfileSlot((liftMotor.GetEncPosition() >= setPoint) ? 1 : 0);
		liftMotor.Set(setPoint);
		beenSet = true;
}

void LiftSubsystem::teleop(void){
	beenSet = false;
	toteHeight = SmartDashboard::GetNumber("toteHeight");
	twoToteHeight = SmartDashboard::GetNumber("twoToteHeight");
	bottomHeight = SmartDashboard::GetNumber("bottomHeight");
	magicToteHeight = SmartDashboard::GetNumber("magicToteHeight");
	SmartDashboard::PutBoolean("botLimit", bottomLimit.Get());
	SmartDashboard::PutBoolean("topLimit", topLimit.Get());
	SmartDashboard::PutBoolean("Top Latch", topLatch);
	SmartDashboard::PutBoolean("Bot Latch", bottomLatch);
	liftValue = robot.joystick.axis("liftAxis");
	bottomHeightButton = robot.joystick.button("bottomHeightButton");
	toteHeightButton = robot.joystick.button("toteHeightButton");
	twoToteHeightButton = robot.joystick.button("twoToteHeightButton");
	magicToteHeightButton = robot.joystick.button("toteIn");
	SmartDashboard::PutNumber("Lift Encoder", liftMotor.GetEncPosition());
	SmartDashboard::PutNumber("Lift Get", liftMotor.Get());

	if (liftMotor.GetStickyFaults() != 0){
		robot.outLog.throwLog("Lift Flag");
		robot.outLog.throwLog(liftMotor.GetStickyFaults());
		liftMotor.Get();
	}
	if (robot.joystick.button("stack")){
		switch (stack.state){
		case HUMAN:
			stack.count = 0;
			setPID(twoToteHeight);
			stack.state = WAITING;
			break;
		case WAITING:
			setPID(21000.0);
			if(!stack.old && robot.joystick.button("toteIn")){
				drive->alignTwo = true;
				stack.state = ALIGNTWO;
			}
			stack.old = robot.joystick.button("toteIn");
			break;
		case ALIGNTWO:
			setPID(21000.0);
			if (!drive->alignTwo){
				stack.state = LOWERTWO;
			}
			break;
		case LOWERTWO:
			setPID(toteHeight);
			if (liftMotor.GetEncPosition()<(-toteHeight)+100 && liftMotor.GetEncPosition()>(-toteHeight)-100){
//				if (stack.count<=stack.max){
					stack.old = true;
					drive->alignOne = true;
					stack.state = ALIGNONESTACK;
					stack.count++;
				/*}else{
					drive->alignOne = true;
					stack.state = ALIGNONE;
				}*/
			}
			break;
		case ALIGNONESTACK:
			setPID(twoToteHeight);
			if (!drive->alignOne){
				stack.state = WAITING;
			}
			break;
		case ALIGNONE:
			setPID(toteHeight);
			if (!drive->alignOne){
				stack.state = LOWERONE;
			}
			break;
		case LOWERONE:
			setPID(100);
			if (liftMotor.GetEncPosition()<-200 && liftMotor.GetEncPosition()>0){
				stack.state = CARRY;
			}
			break;
		case CARRY:
			setPID(bottomHeight);
			break;
			default:
				if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
					liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
				}
				liftMotor.Set(0.0);
			break;
		}

		SmartDashboard::PutNumber("Stack State", stack.state);









	}else{
	stack.state = HUMAN;
	if (!bottomLimit.Get()){
		bottomLatch = true;
		liftMotor.SetPosition(0);
	}
	if (!topLimit.Get()){
		topLatch = true;
	}
	if (liftValue < 0.1 && liftValue > -.1){
		liftValue = 0;
	}


	if (liftValue < 0.0 && topLatch == false){

		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		bottomLatch = false;
		liftMotor.Set(liftValue * SmartDashboard::GetNumber("Lift-Speed"));
		beenSet = true;
	}else if (liftValue > 0.0 && bottomLatch == false){
		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		topLatch = false;
		liftMotor.Set(liftValue * SmartDashboard::GetNumber("Lift-Speed"));
		beenSet = true;
	}else if (twoToteHeightButton == true){
		setPID(twoToteHeight);
	}else if (toteHeightButton == true){
		setPID(toteHeight);
	}else if (bottomHeightButton == true){
		setPID(bottomHeight);
	}else if (magicToteHeightButton == true){
		setPID(magicToteHeight);
	}else{
		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		liftMotor.Set(0.0);
		beenSet = true;
	}
	if (!beenSet){
		if(liftMotor.GetControlMode() != CANSpeedController::kPercentVbus){
			liftMotor.SetControlMode(CANSpeedController::kPercentVbus);
		}
		liftMotor.Set(0.0);
	}
	}
}

void LiftSubsystem::teleopEnd(){
	liftMotor.SetSafetyEnabled(false);
	liftMotor.Set(0.0);
}
double LiftSubsystem::getLiftHeight(void)
{
	return liftMotor.GetPosition();
}
double LiftSubsystem::getBufferValue(void){
	return buffer;
}
void LiftSubsystem::setLift(double speed){
	setPID(speed);
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
//		encoderPID.mistake = encoderPID.setPoint - encoder.Get();
		encoderPID.integral = encoderPID.integral + (encoderPID.mistake * .05);
		encoderPID.derivative = (encoderPID.mistake - encoderPID.lastError) * (1/.05);
		double output = (encoderPID.P*encoderPID.mistake) + (encoderPID.I*encoderPID.integral) + (encoderPID.D*encoderPID.derivative);
		output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
		encoderPID.lastError = encoderPID.mistake;
		return output;
}
