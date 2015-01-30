#include "DriveSubsystem.h"
#include <iostream>


std::string DriveSubsystem::name(void){
	return "Drive";
}


void DriveSubsystem::robotInit(void){
	std::cout << "                                                  drive robo init" << std::endl;
//	robot.outLog.throwLog("DriveRobotInit");
}
void DriveSubsystem::teleopInit(void){
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
//	robot.outLog.throwLog("DriveTeleInit");
}
	
void DriveSubsystem::teleop(void){
	frontLeft.SetPID(SmartDashboard::GetNumber("FrontLeftPValue"), SmartDashboard::GetNumber("FrontLeftIValue"), SmartDashboard::GetNumber("FrontLeftDValue"));
	backLeft.SetPID(SmartDashboard::GetNumber("BackLeftPValue"), SmartDashboard::GetNumber("BackLeftIValue"), SmartDashboard::GetNumber("BackLeftDValue"));
	frontRight.SetPID(SmartDashboard::GetNumber("FrontRightPValue"), SmartDashboard::GetNumber("FrontRightIValue"), SmartDashboard::GetNumber("FrontRightDValue"));
	backRight.SetPID(SmartDashboard::GetNumber("BackRightPValue"), SmartDashboard::GetNumber("BackRightIValue"), SmartDashboard::GetNumber("BackRightDValue"));
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


	SmartDashboard::PutNumber("drive x", drive_x);
	SmartDashboard::PutNumber("drive y", drive_y);
	SmartDashboard::PutNumber("drive rot", drive_rotation);
//	driveMotors.MecanumDrive_Cartesian(drive_x, drive_y, drive_rotation);
	frontLeftSet = ((drive_x+drive_y+drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	frontRightSet = ((-drive_x+drive_y-drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	backLeftSet = ((-drive_x+drive_y+drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	backRightSet = ((drive_x+drive_y-drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	SmartDashboard::PutNumber("Front Left Set", frontLeftSet);
	SmartDashboard::PutNumber("Front Right Set", frontRightSet);
	SmartDashboard::PutNumber("Back Left Set", backLeftSet);
	SmartDashboard::PutNumber("Back Right Set", backRightSet);
	frontLeft.Set(frontLeftSet);
	frontRight.Set(frontRightSet);
	backLeft.Set(backLeftSet);
	backRight.Set(backRightSet);
	}
void DriveSubsystem::teleopEnd(void){
//	driveMotors.SetSafetyEnabled(false);
}

double DriveSubsystem::getDistance(void)
{
	double distance = (fabs(frontLeftEnc.GetDistance())+fabs(frontRightEnc.GetDistance())+fabs(backRightEnc.GetDistance())+fabs(backLeftEnc.GetDistance()))/4;
	return distance;
}

void DriveSubsystem::mec_drive(double drive_x, double drive_y, double rotation)
{
//driveMotors.MecanumDrive_Cartesian(drive_x, drive_y, rotation);
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
