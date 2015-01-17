#ifndef DRIVESUBSYSTEM_H
#define DRIVESUBSYSTEM_H

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
using namespace CORE;

class DriveSubsystem : public CORESubsystem{
	// Drive Motors
	Jaguar frontLeft;
	Jaguar backLeft;
	Jaguar frontRight;
	Jaguar backRight;
	RobotDrive driveMotors;
	
	float drive_x;
	float drive_rotation;
	float drive_y;
	
public:
	
	std::string name(void);
	DriveSubsystem(CORERobot& robot):
		CORESubsystem(robot),
		frontLeft(6),
		backLeft(8),
		frontRight(9),
		backRight(7),
		driveMotors(frontLeft,backLeft,frontRight,backRight)
		{
			driveMotors.SetExpiration(0.1);
			// Motor Invertions
			//	driveMotors.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
			//	driveMotors.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
			driveMotors.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
			driveMotors.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
			driveMotors.SetSafetyEnabled(true);
		}
	void robotInit(void);
	// Called before loop at start of Teleop period
	void teleopInit(void);
	
	//Called sequentially during loop, interleaved with other subsystems
	void teleop(void);
};
#endif
