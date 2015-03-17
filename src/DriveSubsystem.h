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


class DriveSubsystem: public CORESubsystem{

	Gyro gyro;
	Timer timer;
	Timer gyroTimer;
	Timer ultraTimer;
	double ultraTime = 0.0;
	double gyroTime = 0.0;
	bool oldCenter = false;
	

	DigitalInput leftPhoto;
	DigitalInput middlePhoto;
	DigitalInput rightPhoto;
	DigitalInput topLeftPhoto;
	DigitalInput topMiddlePhoto;
	DigitalInput topRightPhoto;
	AnalogInput ultra;
	AnalogInput jumper;
	DigitalInput centerPhoto;

//	BuiltInAccelerometer accel;

	DoubleSolenoid binPunch;


	float drive_x = 0.0;
	float ultraVoltageScale = (1024.0 / 2.54); //403.1496
	float ultraValue = 0.0;
	float ultrasonicValue = 0.0;
	float drive_rotation = 0.0;
	float drive_y = 0.0;
	float centerDrivePower = 0.0;
	float frontLeftSet = 0.0;
	float frontRightSet = 0.0;
	float backLeftSet = 0.0;
	float backRightSet = 0.0;
	float initalUltraValue = 0.0;
	int oldFrontRight = 0;
	int oldFrontLeft = 0;
	int oldBackRight = 0;
	int oldBackLeft = 0;
	bool ultraCalculated = false;
	bool isTested = false;
	bool isBroken = true;
	bool switchEncoderMode = false;
	bool flag = false;
	bool shoulderSpeed = false;
	bool oldRot = 0.0;
	int resetQ = 0;
	int POV = -1;
	bool leftPhotoVar = 0;
	bool middlePhotoVar = 0;
	bool rightPhotoVar = 0;
	bool tl = 0;
	bool tm = 0;
	bool tr = 0;
	bool alignError = 0;
	bool targetSeen = false;
	bool simple = true;
	bool oldCenterPhoto = false;
	int centerDirection = 1;
	bool oldCenterButton = true;

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
		}gyroPID, ultraPID;

public:
		bool alignTwo = false;
		bool alignOne = false;
		bool ultraDistCorrect = false;
		bool ultraCenter = false;
		// Drive Motors
		CANTalon frontLeft;
		CANTalon backLeft;
		CANTalon frontRight;
		CANTalon backRight;
	std::string name(void);
	DriveSubsystem(CORERobot& robot):
		CORESubsystem(robot),
		gyro(1),
		leftPhoto(2),
		middlePhoto(3),
		rightPhoto(4),
		topLeftPhoto(5),
		topMiddlePhoto(6),
		topRightPhoto(7),
		ultra(2),
		jumper(3),
		centerPhoto(8),
//		accel(),
		binPunch(1,0),
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
			gyro.SetDeadband(0.007); //TODO .005 on main bot tried .007
			frontLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
			backLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
			frontRight.SetFeedbackDevice(CANTalon::QuadEncoder);
			backRight.SetFeedbackDevice(CANTalon::QuadEncoder);
			frontLeft.ConfigEncoderCodesPerRev(1024);
			backLeft.ConfigEncoderCodesPerRev(1024);
			frontRight.ConfigEncoderCodesPerRev(1024);
			backRight.ConfigEncoderCodesPerRev(1024);

			frontLeft.SetSensorDirection(true);
			backLeft.SetSensorDirection(true);
			frontRight.SetSensorDirection(true);
			backRight.SetSensorDirection(true);
		}
	void robotInit(void);
	// Called before loop at start of Teleop period
	void teleopInit(void);
	//Called sequentially during loop, interleaved with other subsystems
	void teleop(void);
	//Main teleop code

	float getUltra(void);
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
	double gyroPIDCalc(double set, double rot, int mult= 1);
	bool getLeftPhoto();
	bool getMiddlePhoto();
	bool getRightPhoto();
	void punchSet(DoubleSolenoid::Value v = DoubleSolenoid::kOff);
	void setMotorExpiration(bool position);
	double rateTest(void);
	void reconstructGyro(void);
};

class DriveAction : public Action{
	DriveSubsystem* drive;
	double speed;
	double targetDistance;
	double currentDistance = 0.0;
	double rotation = 0.0;
	int countTime = 0;
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

//		drive->resetRot();
		rotation = drive->getRot();
		countTime = 0;
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
		countTime++;
		if(targetDistance>=0){
			if(currentDistance<=targetDistance){
//				drive->giveLog("set1");
				drive->mec_drive(0,speed,rotation);
//				drive->robot.outLog.throwLog(speed);
//				drive->robot.outLog.throwLog(rotation);
//				drive->giveLog("cont");
				return CONTINUE;
			}else if (countTime >3){
				drive->mec_drive(0,0,0);
				drive->giveLog("DriveAction Completed");
				drive->robot.outLog.throwLog("Strafe end Enc:", drive->getDistance());
				return END;
			}else{
				return CONTINUE;
			}
		}else{
			if(currentDistance>=targetDistance){
//				drive->giveLog("set2");
				drive->mec_drive(0,speed,rotation);
				drive->giveLog("cont");
				return CONTINUE;
			}else if (countTime >3){
				drive->mec_drive(0,0,0);
				drive->giveLog("DriveAction Completed");
				return END;
			}else{
				return CONTINUE;
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
//			drive->resetRot();

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

					drive->giveLog("StrafeAction Completed");
					drive->robot.outLog.throwLog("Strafe end Enc:", drive->getDistance());
					drive->resetDistance();
					return END;
				}
			}else{
				if(currentDistance>targetDistance){
					drive->mec_drive(speed,0,rotation);
					return CONTINUE;
				}else{
					drive->mec_drive(0,0,0);
					drive->giveLog("DriveAction Completed");
					drive->resetDistance();
					return END;
				}
			}

		}
	};

class TurnAction : public Action{
	DriveSubsystem* drive;
	double degrees;
	double rotation;
	int mult;
	int seen = 0;
	int seenNeed;
public:
	std::string name = "Turn Action";
	TurnAction(DriveSubsystem& drive, double degrees, int mult = 1):
		drive(&drive),
		degrees(degrees),
		mult(mult)
	{
		rotation = 0;
		seenNeed = (mult == 2)?3:1;
	}

	void init(void){
//		drive->setVoltageMode();
		drive->resetRot();
		drive->giveLog("Turn Init");
		rotation = drive->getRot();
	}
	ControlFlow call(void){
		drive->resetDistance();
		rotation = drive->gyroPIDCalc(degrees,drive->getRot(),mult);
		drive->mec_drive(0,0,rotation);
		if (drive->getRot()>degrees-2.0 && drive->getRot()<degrees+2.0){
			drive->giveLog("Target Hit");
			seen++;
			if (seen ==seenNeed){
				drive->giveLog("TurnAction Completed");
				drive->mec_drive(0,0,0);
				drive->resetRot();
				drive->giveLog("Turn End");
				drive->resetDistance();
				return END;
			}else{
				return CONTINUE;
			}
		}else{
			return CONTINUE;
		}
	}
};

class TurnActionNoReset : public Action{
	DriveSubsystem* drive;
	double degrees;
	double rotation;
	int mult;
	int seen = 0;
	int seenNeed;
public:
	std::string name = "Turn Action";
	TurnActionNoReset(DriveSubsystem& drive, double degrees, int mult = 1):
		drive(&drive),
		degrees(degrees),
		mult(mult)
	{
		rotation = 0;
		seenNeed = (mult == 2)?3:1;
	}

	void init(void){
//		drive->setVoltageMode();
//		drive->resetRot();
		drive->giveLog("Turn Init");
		rotation = drive->getRot();
	}
	ControlFlow call(void){
		drive->resetDistance();
		rotation = drive->gyroPIDCalc(degrees,drive->getRot(),mult);
		drive->mec_drive(0,0,rotation);
		if (drive->getRot()>degrees-3.0 && drive->getRot()<degrees+3.0){
			drive->giveLog("Target Hit");
			seen++;
			if (seen ==seenNeed){
				drive->giveLog("TurnAction Completed");
				drive->mec_drive(0,0,0);
//				drive->resetRot();
				drive->giveLog("Turn End");
				drive->resetDistance();
				return END;
			}else{
				return CONTINUE;
			}
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
		bool oldMidValue = true;
		bool oldLeftValue = true;
		bool oldRightValue = true;
		int sideTicks = -1;
		double rotation = 0.0;
		double maxDist;
		int timeTicks = 0;
		double speed;
public:
	PhotoDriveAction(DriveSubsystem& drive, double maxDist = 10000.0, double speed = .9):
		drive(&drive),
		maxDist(maxDist),
		speed(speed)
		{

	}
	void init(void){
		drive->giveLog("PotoDriveAction Start");
		drive->resetDistance();
		timeTicks = 0;
	}
	ControlFlow call(void){
		rotation = drive->gyroPIDCalc(0,drive->getRot());
		if(!oldMidValue && drive->getMiddlePhoto()){
			drive->mec_drive(0,0,0);
			drive->giveLog("PhotoDriveAction Complete");
			return END;
		}else if((!oldRightValue && drive->getRightPhoto()) || (!oldLeftValue && drive->getLeftPhoto())){
			drive->robot.outLog.throwLog("PHOTO SIDE SEEN");
			sideTicks = 3;
			return CONTINUE;
		}else if (sideTicks == 0){
			drive->mec_drive(0,0,0);
			drive->robot.outLog.throwLog("PHOTO SIDE SEEN STOP", drive->getDistance());
//			drive->giveLog("PhotoDriveAction Complete, MAX DIST");
			return END;
		}else if (drive->getDistance()>maxDist && timeTicks >7){
			drive->mec_drive(0,0,0);
			drive->robot.outLog.throwLog("PHOTO MAX DIST", drive->getDistance());
//			drive->giveLog("PhotoDriveAction Complete, MAX DIST");
			return END;
		}
		drive->mec_drive(0,speed,rotation);
		oldMidValue = drive->getMiddlePhoto();
		oldRightValue = drive->getRightPhoto();
		oldLeftValue = drive->getLeftPhoto();
		sideTicks--;
		timeTicks++;
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
	double strafe = .6;
	int seen = 0;
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
				strafe += .05;
				drive->mec_drive(-strafe,0,rotation);
				return CONTINUE;
			}else if (!leftPhotoVar && rightPhotoVar){
				strafe += .05;
				drive->mec_drive(strafe,0,rotation);
				return CONTINUE;
			}else if (!leftPhotoVar && middlePhotoVar && !rightPhotoVar){
				strafe = .5;
				drive->mec_drive(0,0,rotation);
				drive->giveLog("Tote Align Seen");
				seen++;
//				return CONTINUE;
			}else /*if((!leftPhotoVar && !middlePhotoVar && !rightPhotoVar) || (leftPhotoVar && middlePhotoVar && rightPhotoVar))*/{
				drive->mec_drive(0,0,0);
				strafe = 0;
				drive->giveLog("ERROR IN PHOTO");
				return END;
			}
			if (seen == 2){
				drive->giveLog("Tote Aligned");
				return END;
			}else{
				return CONTINUE;
			}



	}
};

class PunchAction : public WaitAction{
	DriveSubsystem* drive;
	double rotation = 0;
	bool background;
public:
	PunchAction(DriveSubsystem& drive, double duration = .5, bool background = false):
		WaitAction(duration),
		drive(&drive),
		background(background)
	{	}

	void init(void){
		drive->giveLog("Bin Slap");
	}

	ControlFlow call(void){
		rotation = drive->getRot();
		rotation = drive->gyroPIDCalc(0, rotation);
		ControlFlow flow = WaitAction::call();
		if (flow == CONTINUE){
			drive->mec_drive(0,0,rotation);
			drive->punchSet(DoubleSolenoid::kForward);
			return background?BACKGROUND:CONTINUE;
		} else {
			drive->giveLog("Bin Slapped");
			drive->mec_drive(0,0,0);
			drive->punchSet(DoubleSolenoid::kReverse);
			return END;
		}
	}

};

class PuncherAction : public Action{
	DriveSubsystem* drive;
	DoubleSolenoid::Value val;
public:
	PuncherAction(DriveSubsystem& drive, DoubleSolenoid::Value v):
		drive(&drive),
		val(v)
	{	}

	void init(void){
		drive->giveLog("Bin Slap");
	}

	ControlFlow call(void){
		drive->punchSet(val);
		return END;
	}
};










#endif

