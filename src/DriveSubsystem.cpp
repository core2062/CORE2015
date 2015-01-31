#include "DriveSubsystem.h"
#include <iostream>


std::string DriveSubsystem::name(void){
	return "Drive";
}


void DriveSubsystem::robotInit(void){
	robot.outLog.throwLog("DriveSubsystem: RobotInit Success");
}
void DriveSubsystem::teleopInit(void){
	robot.outLog.throwLog("DriveSubsystem: TeleopInit Success");

	frontLeft.SetSafetyEnabled(true);
	backLeft.SetSafetyEnabled(true);
	frontRight.SetSafetyEnabled(true);
	backRight.SetSafetyEnabled(true);

	robot.joystick.register_axis("drive_x", 1, 1);
	robot.joystick.register_axis("drive_rotation", 1, 4);
	robot.joystick.register_axis("drive_y", 1, 2);

	frontLeft.SetControlMode(CANSpeedController::kSpeed);
	backLeft.SetControlMode(CANSpeedController::kSpeed);
	frontRight.SetControlMode(CANSpeedController::kSpeed);
	backRight.SetControlMode(CANSpeedController::kSpeed);
	oldFrontRight = frontRight.GetEncPosition();
	oldFrontLeft = frontLeft.GetEncPosition();
	oldBackRight = backRight.GetEncPosition();
	oldBackLeft = backLeft.GetEncPosition();
	timer.Start();
}
	
void DriveSubsystem::teleop(void){
	SmartDashboard::PutNumber("drive x", drive_x);
	SmartDashboard::PutNumber("drive y", drive_y);
	SmartDashboard::PutNumber("drive rot", drive_rotation);
	SmartDashboard::PutBoolean("Encoders-Status", isBroken);
	SmartDashboard::PutNumber("Front Left Set", frontLeftSet);
	SmartDashboard::PutNumber("Front Right Set", frontRightSet);
	SmartDashboard::PutNumber("Back Left Set", backLeftSet);
	SmartDashboard::PutNumber("Back Right Set", backRightSet);


	switchEncoderMode = SmartDashboard::GetBoolean("Swtich-Encoder-Status", false);

	frontLeft.SetPID(SmartDashboard::GetNumber("FrontLeftPValue"), SmartDashboard::GetNumber("FrontLeftIValue"), SmartDashboard::GetNumber("FrontLeftDValue"));
	backLeft.SetPID(SmartDashboard::GetNumber("BackLeftPValue"), SmartDashboard::GetNumber("BackLeftIValue"), SmartDashboard::GetNumber("BackLeftDValue"));
	frontRight.SetPID(SmartDashboard::GetNumber("FrontRightPValue"), SmartDashboard::GetNumber("FrontRightIValue"), SmartDashboard::GetNumber("FrontRightDValue"));
	backRight.SetPID(SmartDashboard::GetNumber("BackRightPValue"), SmartDashboard::GetNumber("BackRightIValue"), SmartDashboard::GetNumber("BackRightDValue"));
	gyroPID.P=(SmartDashboard::GetNumber("gyroPValue"));
	gyroPID.I=(SmartDashboard::GetNumber("gyroIValue"));
	gyroPID.D=(SmartDashboard::GetNumber("gyroDValue"));

//Simple Dead-banding
	drive_x = robot.joystick.axis("drive_x");
	if (drive_x < .2 && drive_x > -.2){
		drive_x = 0;
	}
	drive_rotation = robot.joystick.axis("drive_rotation");
	if (drive_rotation < .2 && drive_rotation > -.2){
		drive_rotation = 0;
	}
	drive_y = robot.joystick.axis("drive_y");
	if (drive_y < .2 && drive_y > -.2){
		drive_y = 0;		
	}

	//Gyro PID
	if(drive_rotation!=0){
		//Disable Brake
		gyroPID.mistake = gyroPID.setPoint - gyro.GetRate();
		gyroPID.integral = gyroPID.integral + (gyroPID.mistake * .05);
		gyroPID.derivative = (gyroPID.mistake - gyroPID.lastError) * (1/.05);
		double output = (gyroPID.P*gyroPID.mistake) + (gyroPID.I*gyroPID.integral) + (gyroPID.D*gyroPID.derivative);
		output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
		drive_rotation = output;
		gyroPID.lastError = gyroPID.mistake;
	}

//Testing for broken encoders
	if(isTested == false){
			if(5 < timer.Get() && timer.Get() < 5.5){
				if(oldFrontRight == frontRight.GetEncPosition()){
					robot.outLog.throwLog("[ERROR] FrontRight Encoder stopped working!");
				}
				if(oldFrontLeft == frontLeft.GetEncPosition()){
					robot.outLog.throwLog("[ERROR] FrontLeft Encoder stopped working!");
				}
				if(oldBackRight == backRight.GetEncPosition()){
					robot.outLog.throwLog("[ERROR] BackRight Encoder stopped working!");
				}
				if(oldBackLeft == backLeft.GetEncPosition()){
					robot.outLog.throwLog("[ERROR] BackLeft Encoder stopped working!");
				}
				isTested = true;
			}
			isBroken = true;
		}

//Deciding which drive mode to use
	if(isBroken == false || switchEncoderMode == false){
		frontLeftSet = ((drive_x+drive_y+drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
		frontRightSet = ((-drive_x+drive_y-drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
		backLeftSet = ((-drive_x+drive_y+drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
		backRightSet = ((drive_x+drive_y-drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
		frontLeft.Set(frontLeftSet);
		frontRight.Set(frontRightSet);
		backLeft.Set(backLeftSet);
		backRight.Set(backRightSet);
	}else{
		if(flag == false){
			frontRight.SetControlMode(CANSpeedController::kVoltage);
			frontLeft.SetControlMode(CANSpeedController::kVoltage);
			backRight.SetControlMode(CANSpeedController::kVoltage);
			backLeft.SetControlMode(CANSpeedController::kVoltage);
			flag = true;
		}
		frontLeftSet = (drive_x+drive_y+drive_rotation);
		frontRightSet = (-drive_x+drive_y-drive_rotation);
		backLeftSet = (-drive_x+drive_y+drive_rotation);
		backRightSet = (drive_x+drive_y-drive_rotation);
		robot.outLog.throwLog("[CHANGE] Encoders were switched to  voltageMode");
	}
}
void DriveSubsystem::teleopEnd(void){
	frontLeft.SetSafetyEnabled(false);
	frontRight.SetSafetyEnabled(false);
	backLeft.SetSafetyEnabled(false);
	backRight.SetSafetyEnabled(false);
}

double DriveSubsystem::getDistance(void)
{
	double distance = (fabs(frontLeftEnc.GetDistance())+fabs(frontRightEnc.GetDistance())+fabs(backRightEnc.GetDistance())+fabs(backLeftEnc.GetDistance()))/4;
	return distance;
}

void DriveSubsystem::mec_drive(double drive_x, double drive_y, double rotation)
{
	frontLeftSet = ((drive_x+drive_y+rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	frontRightSet = ((-drive_x+drive_y-rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	backLeftSet = ((-drive_x+drive_y+rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	backRightSet = ((drive_x+drive_y-rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
}
double DriveSubsystem::getRot(void)
{
	return gyro.GetAngle();
}
void DriveSubsystem::resetRot(void)
{
	gyro.Reset();
}
void DriveSubsystem::setPositionMode(void){
	frontLeft.SetControlMode(CANSpeedController::kPosition);
	backLeft.SetControlMode(CANSpeedController::kPosition);
	frontRight.SetControlMode(CANSpeedController::kPosition);
	backRight.SetControlMode(CANSpeedController::kPosition);
}
void DriveSubsystem::setVoltageMode(void){
	frontLeft.SetControlMode(CANSpeedController::kVoltage);
	backLeft.SetControlMode(CANSpeedController::kVoltage);
	frontRight.SetControlMode(CANSpeedController::kVoltage);
	backRight.SetControlMode(CANSpeedController::kVoltage);
}
void DriveSubsystem::setSpeedMode(void){
	frontLeft.SetControlMode(CANSpeedController::kSpeed);
	backLeft.SetControlMode(CANSpeedController::kSpeed);
	frontRight.SetControlMode(CANSpeedController::kSpeed);
	backRight.SetControlMode(CANSpeedController::kSpeed);
}
void DriveSubsystem::setFrontLeftMotor(double value){
	frontLeft.Set(value);
}
void DriveSubsystem::setFrontRightMotor(double value){
	frontRight.Set(value);
}
void DriveSubsystem::setBackLeftMotor(double value){
	backLeft.Set(value);
}
void DriveSubsystem::setBackRightMotor(double value){
	backRight.Set(value);
}
double DriveSubsystem::getJoystickMultiplier(void){
	return SmartDashboard::GetNumber("joystickMultiplier");
}
void DriveSubsystem::giveLog(std::string stringVar){
	robot.outLog.throwLog(stringVar);
}
