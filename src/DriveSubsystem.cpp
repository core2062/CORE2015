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
	driveMotors.SetSafetyEnabled(true);
	robot.joystick.register_axis("drive_x", 1, 0);
	robot.joystick.register_axis("drive_rotation", 1, 2);
	robot.joystick.register_axis("drive_y", 1, 1);
//	robot.outLog.throwLog("DriveTeleInit");
}
	
void DriveSubsystem::teleop(void)
{
	drive_x = robot.joystick.axis("drive_x");
	if (drive_x < .2 && drive_x > -.2){
		drive_x = 0;
	}
	drive_rotation =robot.joystick.axis("drive_rotation");
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
	driveMotors.MecanumDrive_Cartesian(drive_x, drive_y, drive_rotation);
	SmartDashboard::PutNumber("FL", frontLeft.Get());
	SmartDashboard::PutNumber("FR", frontRight.Get());
	SmartDashboard::PutNumber("BL", backLeft.Get());
	SmartDashboard::PutNumber("BR", backRight.Get());
}

void DriveSubsystem::teleopEnd(void){
	driveMotors.SetSafetyEnabled(false);
}

double DriveSubsystem::getDistance(void)
{
	double distance = (fabs(frontLeftEnc.GetDistance())+fabs(frontRightEnc.GetDistance())+fabs(backRightEnc.GetDistance())+fabs(backLeftEnc.GetDistance()))/4;
	return distance;
}

void DriveSubsystem::mec_drive(double drive_x, double drive_y, double rotation)
{
driveMotors.MecanumDrive_Cartesian(drive_x, drive_y, rotation);
}
double DriveSubsystem::getRot(void)
{
	return gyro.GetAngle();
}
void DriveSubsystem::resetRot(void)
{
	gyro.Reset();
}
