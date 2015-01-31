#ifndef DRIVESUBSYSTEM_H
#define DRIVESUBSYSTEM_H

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
#include "CORERobot/SpeedPID.h"
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

	Timer timer;
	Gyro gyro;
	
	float drive_x = 0.0;
	float drive_rotation = 0.0;
	float drive_y = 0.0;
	float frontLeftSet = 0.0;
	float frontRightSet = 0.0;
	float backLeftSet = 0.0;
	float backRightSet = 0.0;
	int oldFrontRight = 0;
	int oldFrontLeft = 0;
	int oldBackRight = 0;
	int oldBackLeft = 0;
	bool isTested = false;
	bool isBroken = false;
	bool switchEncoderMode = false;

	struct{
		double P = 0.1;
		double I = 0.001;
		double D = 0.0;
		double mistake;
		double actualPosition;
		double lastError;
		double integral;
		double derivative;
		double setPoint = 0.0;
		bool enabled = false;
		}gyroPID;

public:
	
	std::string name(void);
	DriveSubsystem(CORERobot& robot):
		CORESubsystem(robot),
		frontLeft(6),
		backLeft(8),
		frontRight(9),
		backRight(7),
		frontLeftEnc(-1,-1), //TODO encoder channels
		backLeftEnc(-1,-1),
		frontRightEnc(-1,-1),
		backRightEnc(-1,-1),
		gyro(-1)
		{
			frontLeft.SetControlMode(CANSpeedController::kSpeed);
			backLeft.SetControlMode(CANSpeedController::kSpeed);
			frontRight.SetControlMode(CANSpeedController::kSpeed);
			backRight.SetControlMode(CANSpeedController::kSpeed);
			frontLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
			backLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
			frontRight.SetFeedbackDevice(CANTalon::QuadEncoder);
			backRight.SetFeedbackDevice(CANTalon::QuadEncoder);
			robot.outLog.throwLog("le (drive) constroctor has arrived");
		}
	void robotInit(void);
	// Called before loop at start of Teleop period
	void teleopInit(void);
	//Called sequentially during loop, interleaved with other subsystems
	void teleop(void);
	//Main teleop code

	void teleopEnd(void);
	double getDistance(void);
	void mec_drive(double drive_x, double drive_y, double rotation);
	double getRot(void);
	void resetRot(void);
	void setPositionMode(void);
	void setVoltageMode(void);
	void setSpeedMode(void);
	void setFrontLeftMotor(double value);
	void setFrontRightMotor(double value);
	void setBackLeftMotor(double value);
	void setBackRightMotor(double value);
	double getJoystickMultiplier(void);

};

class DriveAction : public Action{
	DriveSubsystem* drive;
	double speed;
	double targetDistance;
	double currentDistance = 0.0;
	double rotation = 0.0;
public:
	DriveAction(DriveSubsystem& drive, double speed, double targetDistance):
		drive(&drive),
		speed(speed),
		targetDistance(targetDistance){

	}
	void init(void){
		drive->setVoltageMode();
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
			drive->setVoltageMode();
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
		drive->setVoltageMode();
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

class PIDDriveAction : public Action{
	DriveSubsystem* drive;
	double targetDistance = 0.0;
public:
	PIDDriveAction(DriveSubsystem& drive, double targetDistance):
		drive(&drive),
		targetDistance(targetDistance){

	}
	void init(void){
		drive->setPositionMode();
	}
	ControlFlow call(void){
		drive->setBackLeftMotor(targetDistance);
		drive->setBackRightMotor(targetDistance);
		drive->setFrontLeftMotor(targetDistance);
		drive->setFrontRightMotor(targetDistance);
		return END;
	}
};

class PIDStrafeAction : public Action{
	DriveSubsystem* drive;
	double targetDistance = 0.0;
	double gyro = 0.0;
public:
	PIDStrafeAction(DriveSubsystem& drive, double targetDistance):
		drive(&drive),
		targetDistance(targetDistance){

	}
	void init(void){
		drive->setPositionMode();
		gyro = drive->getRot();
	}
	ControlFlow call(void){

		drive->setBackLeftMotor((-targetDistance+0+gyro)*drive->getJoystickMultiplier());
		drive->setBackRightMotor((targetDistance+0-gyro)*drive->getJoystickMultiplier());
		drive->setFrontLeftMotor((targetDistance+0+gyro)*drive->getJoystickMultiplier());
		drive->setFrontRightMotor((-targetDistance+0-gyro)*drive->getJoystickMultiplier());
		return END;
	}
};
#endif

