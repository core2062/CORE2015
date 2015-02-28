#ifndef DRIVESUBSYSTEM_H
#define DRIVESUBSYSTEM_H

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
#include "CORERobot/SpeedPID.h"
#include <iostream>
using namespace CORE;

const int frontLeftInvert = 1;
const int backLeftInvert = 1;
const int frontRightInvert = -1;
const int backRightInvert = -1;


class DriveSubsystem : public CORESubsystem{






	Timer timer;
	Timer gyroTimer;
	double gyroTime = 0.0;
	Gyro gyro;
	
	DigitalInput leftPhoto;
	DigitalInput middlePhoto;
	DigitalInput rightPhoto;
	DigitalInput topLeftPhoto;
	DigitalInput topMiddlePhoto;
	DigitalInput topRightPhoto;

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
	bool isBroken = true;
	bool switchEncoderMode = false;
	bool flag = false;
	bool shoulderSpeed = false;
	bool oldRot = 0.0;
	int resetQ = 0;
	bool leftPhotoVar = 0;
	bool middlePhotoVar = 0;
	bool rightPhotoVar = 0;
	bool tl = 0;
	bool tm = 0;
	bool tr = 0;
	bool alignError = 0;
	double alignPowerLeft = -.5;
	double alignPowerRight = .5;

	struct{
		double P = 0.1;
		double I = 0.001;
		double D = 0.0;
		double mistake;
		double actualPosition;
		double lastError;
		double integral =0.0;
		double derivative;
		double setPoint = 0.0;
		bool enabled = false;
		}gyroPID;

public:
		bool alignTwo = false;
		bool alignOne = false;
		// Drive Motors
		CANTalon frontLeft;
		CANTalon backLeft;
		CANTalon frontRight;
		CANTalon backRight;
	std::string name(void);
	DriveSubsystem(CORERobot& robot):
		CORESubsystem(robot),
		gyro(0),
		leftPhoto(2),
		middlePhoto(3),
		rightPhoto(4),
		topLeftPhoto(5),
		topMiddlePhoto(6),
		topRightPhoto(7),
		frontLeft(13),
		backLeft(12),
		frontRight(10),
		backRight(11)
		{
			//start false to avoid error
			frontLeft.SetSafetyEnabled(false);
			frontRight.SetSafetyEnabled(false);
			backLeft.SetSafetyEnabled(false);
			backRight.SetSafetyEnabled(false);
			frontLeft.Set(0.0);
			frontRight.Set(0.0);
			backLeft.Set(0.0);
			backRight.Set(0.0);
			gyro.SetDeadband(0.007); //TODO .005 on main bot
			frontLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
			backLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
			frontRight.SetFeedbackDevice(CANTalon::QuadEncoder);
			backRight.SetFeedbackDevice(CANTalon::QuadEncoder);
//			robot.outLog.throwLog("set to voltage");
//			frontRight.SetControlMode(CANSpeedController::kPercentVbus);
//			frontLeft.SetControlMode(CANSpeedController::kPercentVbus);
//			backRight.SetControlMode(CANSpeedController::kPercentVbus);
//			backLeft.SetControlMode(CANSpeedController::kPercentVbus);
			frontRight.SetSensorDirection(true);
			backRight.SetSensorDirection(true);
		}
	void robotInit(void);
	// Called before loop at start of Teleop period
	void teleopInit(void);
	//Called sequentially during loop, interleaved with other subsystems
	void teleop(void);
	//Main teleop code

	void teleopEnd(void);
	double getDistance(void);
	void resetDistance(void);
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
	void giveLog(std::string stringVar);
	double gyroPIDCalc(double set, double rot);
	bool getLeftPhoto();
	bool getMiddlePhoto();
	bool getRightPhoto();

};

class DriveAction : public Action{
	DriveSubsystem* drive;
	double speed;
	double targetDistance;
	double currentDistance = 0.0;
	double rotation = 0.0;
public:
	std::string name = "Drive Action";
	DriveAction(DriveSubsystem& drive, double speed, double targetDistance):
		drive(&drive),
		speed(speed),
		targetDistance(targetDistance){

	}
	void init(void){
		drive->giveLog("drive action init");
//		drive->setVoltageMode();
		drive->resetDistance();
		currentDistance = drive->getDistance();

		drive->resetRot();
		rotation = drive->getRot();
	}
	ControlFlow call(void){
//		drive->frontLeft.SetSafetyEnabled(true);
//		drive->frontLeft.Set(1.0);
//		drive->giveLog("drive action iter");
		rotation = drive->getRot();
//		drive->giveLog("rot gotten");
		rotation = drive->gyroPIDCalc(0, rotation);
//		drive->giveLog("pid calced");
		currentDistance = drive->getDistance();
//		drive->robot.outLog.throwLog(currentDistance);
//		drive->giveLog("dist got");
		if(targetDistance>=0){
			if(currentDistance<=targetDistance){
//				drive->giveLog("set1");
				drive->mec_drive(0,speed,rotation);
//				drive->robot.outLog.throwLog(speed);
//				drive->robot.outLog.throwLog(rotation);
//				drive->giveLog("cont");
				return CONTINUE;
			}else{
				drive->mec_drive(0,0,0);
				drive->giveLog("DriveAction Completed");
				return END;
			}
		}else{
			if(currentDistance>=targetDistance){
//				drive->giveLog("set2");
				drive->mec_drive(0,speed,rotation);
				drive->giveLog("cont");
				return CONTINUE;
			}else{
				drive->mec_drive(0,0,0);
				drive->giveLog("DriveAction Completed");
				return END;
			}
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
		std::string name = "Strafe Action";
		StrafeAction(DriveSubsystem& drive, double speed, double targetDistance):
			drive(&drive),
			speed(speed),
			targetDistance(targetDistance)
		{

		}
		void init(void){
			drive->resetDistance();
//			drive->setVoltageMode();
			currentDistance = drive->getDistance();
			drive->resetRot();

		}
		ControlFlow call(void){
			rotation = drive->getRot();
			rotation = drive->gyroPIDCalc(0, rotation);
			currentDistance = drive->getDistance();
			if(targetDistance>0){
				if(currentDistance<targetDistance){
					drive->mec_drive(speed,0,rotation);
					return CONTINUE;
				}else{
					drive->mec_drive(0,0,0);
					drive->giveLog("DriveAction Completed");
					return END;
				}
			}else{
				if(currentDistance>targetDistance){
					drive->mec_drive(speed,0,rotation);
					return CONTINUE;
				}else{
					drive->mec_drive(0,0,0);
					drive->giveLog("DriveAction Completed");
					return END;
				}
			}

		}
	};

class TurnAction : public Action{
	DriveSubsystem* drive;
	double degrees;
	double rotation;
public:
	std::string name = "Turn Action";
	TurnAction(DriveSubsystem& drive, double degrees):
		drive(&drive),
		degrees(degrees)
	{
		rotation = 0;
	}

	void init(void){
//		drive->setVoltageMode();
		drive->resetRot();
		rotation = drive->getRot();
	}
	ControlFlow call(void){
		rotation = drive->gyroPIDCalc(degrees,drive->getRot());
		drive->mec_drive(0,0,rotation);
		if (drive->getRot()>degrees-2.0 && drive->getRot()<degrees+2.0){
			drive->giveLog("TurnAction Completed");
			drive->mec_drive(0,0,0);
			return END;
		}else{
			return CONTINUE;
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
		drive->giveLog("PIDDriveAction Completed");
		drive->setBackLeftMotor(targetDistance);
		drive->setBackRightMotor(targetDistance);
		drive->setFrontLeftMotor(targetDistance);
		drive->setFrontRightMotor(targetDistance);
		return END;
	}
};
class PhotoDriveAction : public Action{
	DriveSubsystem* drive;
		bool oldValue = true;
		double rotation = 0.0;
public:
	PhotoDriveAction(DriveSubsystem& drive):
		drive(&drive){

	}
	void init(void){
		drive->giveLog("PotoDriveAction Start");
	}
	ControlFlow call(void){
		rotation = drive->gyroPIDCalc(0,drive->getRot());
		if(!oldValue && drive->getMiddlePhoto()){
			drive->mec_drive(0,0,0);
			drive->giveLog("PhotoDriveAction Completo");
			return END;
		}
		drive->mec_drive(0,0.9,rotation);
		oldValue = drive->getMiddlePhoto();
		return CONTINUE;
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
//		drive->setPercentMode();
		gyro = drive->getRot();
	}
	ControlFlow call(void){
		drive->giveLog("PIDStrafeAction Completed");
		drive->setBackLeftMotor((-targetDistance+0+gyro)*drive->getJoystickMultiplier());
		drive->setBackRightMotor((targetDistance+0-gyro)*drive->getJoystickMultiplier());
		drive->setFrontLeftMotor((targetDistance+0+gyro)*drive->getJoystickMultiplier());
		drive->setFrontRightMotor((-targetDistance+0-gyro)*drive->getJoystickMultiplier());
		return END;
	}
};

class AlignAction: public Action{
	DriveSubsystem* drive;
	bool rightPhotoVar = false;
	bool middlePhotoVar = false;
	bool leftPhotoVar = false;
	double rotation = 0;
	double strafe = 0;
public:
	std::string name = "Align Action";
	AlignAction(DriveSubsystem& drive):
		drive(&drive)
		{}
	void init(void){

	}
	ControlFlow call(void){
			rotation = drive->getRot();
			rotation = drive->gyroPIDCalc(0, rotation);
			//Tote Alignment
			//set vars
			leftPhotoVar = drive->getLeftPhoto();
			middlePhotoVar = drive->getMiddlePhoto();
			rightPhotoVar = drive->getRightPhoto();

			//different cases
			if (leftPhotoVar && !rightPhotoVar){
				strafe = -.6;
				drive->mec_drive(strafe,0,rotation);
				return CONTINUE;
			}else if (!leftPhotoVar && rightPhotoVar){
				strafe = .6;
				drive->mec_drive(strafe,0,rotation);
				return CONTINUE;
			}else if (!leftPhotoVar && middlePhotoVar && !rightPhotoVar){
				strafe = 0;
				drive->mec_drive(0,0,0);
				drive->giveLog("Tote Aligned");
				return END;
			}else /*if((!leftPhotoVar && !middlePhotoVar && !rightPhotoVar) || (leftPhotoVar && middlePhotoVar && rightPhotoVar))*/{
				drive->mec_drive(0,0,0);
				strafe = 0;
				drive->giveLog("ERROR IN PHOTO");
				return END;
			}



	}
};












#endif

