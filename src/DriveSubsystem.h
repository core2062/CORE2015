#ifndef DRIVESUBSYSTEM_H
#define DRIVESUBSYSTEM_H

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
using namespace CORE;

class DriveSubsystem : public CORESubsystem{
	// Drive Motors
	CANTalon frontLeft;
	CANTalon backLeft;
	CANTalon frontRight;
	CANTalon backRight;

	Encoder frontLeftEnc;
	Encoder backLeftEnc;
	Encoder frontRightEnc;
	Encoder backRightEnc;

	Gyro gyro;
	
	float drive_x = 0.0;
	float drive_rotation = 0.0;
	float drive_y = 0.0;
	
public:
	RobotDrive driveMotors;
	std::string name(void);
	DriveSubsystem(CORERobot& robot):
		CORESubsystem(robot),
		frontLeft(13),
		backLeft(12),
		frontRight(10),
		backRight(11),
		frontLeftEnc(-1, -1),
		backLeftEnc(-1, -1),
		frontRightEnc(-1, -1),
		backRightEnc(-1, -1),
		gyro(-1),
		driveMotors(frontLeft,backLeft,frontRight,backRight)
		{
			driveMotors.SetExpiration(0.1);
			// Motor Invertions
			//	driveMotors.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
			//	driveMotors.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
			driveMotors.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
			driveMotors.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
			driveMotors.SetSafetyEnabled(false);
		}
	void robotInit(void);
	// Called before loop at start of Teleop period
	void teleopInit(void);
	
	//Called sequentially during loop, interleaved with other subsystems
	void teleop(void);
	void teleopEnd(void);
	double getDistance(void);
	void mec_drive(double drive_x, double drive_y, double rotation);
	double getRot(void);
	void resetRot(void);
};

class DriveAction : public Action{
	DriveSubsystem* drive;
	double speed;
	double targetDistance;
	double currentDistance;
	double rotation = 0.0;
public:
	DriveAction(DriveSubsystem& drive, double speed, double targetDistance):
		drive(&drive),
		speed(speed),
		targetDistance(targetDistance){

	}
	void init(void){
		currentDistance = drive->getDistance();
		rotation = drive->getRot();
		rotation = rotation/-100.0;
	}
	ControlFlow call(void){
		currentDistance = drive->getDistance();
		if(currentDistance<targetDistance){
			drive->mec_drive(0,speed,rotation);
			return CONTINUE;
		}else{
			drive->mec_drive(0,0,0);
			return END;
		}
	}
};
	class StrafeAction : public Action{
		DriveSubsystem* drive;
		double speed;
		double targetDistance;
		double currentDistance = 0.0;
		double rotation = 0.0;
	public:
		StrafeAction(DriveSubsystem& drive, double speed, double targetDistance):
			drive(&drive),
			speed(speed),
			targetDistance(targetDistance)
		{

		}
		void init(void){
			currentDistance = drive->getDistance();
			rotation = drive->getRot();
			rotation = rotation/-100.0;
		}
		ControlFlow call(void){
			currentDistance = drive->getDistance();
			if(currentDistance<targetDistance){
				drive->mec_drive(speed,0,rotation);
				return CONTINUE;
			}else{
				drive->mec_drive(0,0,0);
				return END;
			}
		}
	};
class TurnAction : public Action{
	DriveSubsystem* drive;
	double speed;
	double degrees;
	double rotation;
public:
	TurnAction(DriveSubsystem& drive, double speed, double degrees):
		drive(&drive),
		speed(speed),
		degrees(degrees)
	{
		rotation = 0;
	}

	void init(void){
		drive->resetRot();
	}
	ControlFlow call(void){
		rotation = drive->getRot();
		if (degrees<0){
					if (rotation>degrees){
						drive->mec_drive(0, 0,-speed);
						return CONTINUE;
					}else{
						drive->mec_drive(0, 0, 0);
						return END;
					}
				}else{
					if (rotation<degrees){
						drive->mec_drive(0, 0, speed);
						return CONTINUE;
					}else{
						drive->mec_drive(0, 0, 0);
						return END;
					}
				}
	}
};
#endif
