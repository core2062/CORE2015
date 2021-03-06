#ifndef DRIVESUBSYSTEM_H
#define DRIVESUBSYSTEM_H

#include "CORERobot/CORERobot.h"
#include "WPILib.h"
#include <cmath>
#include "CORERobot/SpeedPID.h"
#include <iostream>
#include <math.h>

//NavX stuff
#include <navx2.0/AHRS.h>

/* NOTE:  Comment in only ONE of the following definitions. */

//#define ENABLE_IMU
//#define ENABLE_IMU_ADVANCED
#define ENABLE_AHRS


#define PI 3.14159265

using namespace CORE;

const int frontLeftInvert = 1;
const int backLeftInvert = 1;
const int frontRightInvert = -1;
const int backRightInvert = -1;


class DriveSubsystem: public CORESubsystem{

#if defined(ENABLE_AHRS)
        AHRS *ahrs;
#elif defined(ENABLE_IMU_ADVANCED)
        IMUAdvanced *ahrs;
#else // ENABLE_IMU
        IMU *ahrs;
#endif
        SerialPort *serial_port;

//	Gyro gyro;
	Timer timer;
	Timer gyroTimer;
	Timer leftUltraTimer;
	Timer rightUltraTimer;
	Timer feederAlignTimer;
	double leftUltraTime = 0.0;
	double rightUltraTime = 0.0;
	double gyroTime = 0.0;
	double feederAlignTime = 0.0;
	bool oldCenter = false;
	bool polar = false;
	double polarMag = 0.0;
	double polarAngle = 0.0;
	bool polarLast = false;
	int polarState = 0;
	double leftUltraVal = 0;
	double rightUltraVal = 0;
	double feederUltraVal = 0;

	

	DigitalInput leftPhoto;
	DigitalInput middlePhoto;
	DigitalInput rightPhoto;
	DigitalInput topLeftPhoto;
	DigitalInput topMiddlePhoto;
	DigitalInput topRightPhoto;
	AnalogInput leftUltra;
	AnalogInput rightUltra;
	AnalogInput leftFeederAlignUltra;
	AnalogInput rightFeederAlignUltra;
	AnalogInput jumper;
	AnalogInput autoUltra;
	AnalogOutput ultraPulse;
	DigitalInput centerPhoto;
	DoubleSolenoid binPunch;

//	SendableChooser feederStationChooser;

	float drive_x = 0.0;
	float ultraVoltageScale = (1024.0 / 2.54); //403.1496
	float ultraValue = 0.0;
	float ultrasonicValue = 0.0;
	float drive_rotation = 0.0;
	float drive_y = 0.0;
	float drive_left_y = 0.0;
	float drive_right_y = 0.0;
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
	bool isTested = false;
	bool isBroken = true;
	bool switchEncoderMode = false;
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
	bool rightFeederStation = true;
	bool modeSet = false;
	bool dPad = false;
	double alignPowerLeft = -.5;
	double alignPowerRight = .5;

	struct{
		double P = 0.0;
		double I = 0.0;
		double D = 0.0;
		double mistake;
		double actualPosition;
		double lastError;
		double integral = 0.0;
		double derivative;
		double setPoint = 0.0;
		bool enabled = false;
		double lastValue = 25.0;
		}gyroPID, leftUltraPID, rightUltraPID, feederAlignPID;

public:
		CANSpeedController::ControlMode mode = CANSpeedController::kPercentVbus;
//		CANSpeedController::ControlMode mode = CANSpeedController::kVoltage;
		bool alignTwo = false;
		bool alignOne = false;
		bool leftUltraDistCorrect = false;
		bool rightUltraDistCorrect = false;
		bool feederAlignUltraDistCorrect = false;
		bool ultraCenter = false;

		// Drive Motors
		CANTalon frontLeft;
		CANTalon backLeft;
		CANTalon frontRight;
		CANTalon backRight;

	std::string name(void);
	DriveSubsystem(CORERobot& robot):
		CORESubsystem(robot),
//		gyro(1),
		leftPhoto(2),
		middlePhoto(3),
		rightPhoto(4),
		topLeftPhoto(5),
		topMiddlePhoto(6),
		topRightPhoto(7),
		leftUltra(5),
		rightUltra(6),
		leftFeederAlignUltra(4),
		rightFeederAlignUltra(7),
		jumper(3),
		autoUltra(1),
		ultraPulse(0),
		centerPhoto(8),
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
//			gyro.SetDeadband(0.007);
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
			ultraPulse.SetVoltage(5.0);
			Wait(.00002);
			ultraPulse.SetVoltage(0.0);
			binPunch.Set(DoubleSolenoid::kReverse);
		}

	void robotInit(void);
	void teleopInit(void);
	void teleop(void);

	void Pulse(void);
	float getJumper(void);
	float getLeftUltra(void);
	float getRightUltra(void);
	float getAutoUltra(void);
	float getFeederAlignUltra(void);
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
	double autoUltraPIDCalc(double set);
	bool getLeftPhoto();
	bool getMiddlePhoto();
	bool getRightPhoto();
	void punchSet(DoubleSolenoid::Value v = DoubleSolenoid::kOff);
	void setMotorExpiration(bool position);
	double rateTest(void);
	void reconstructGyro(void);
	void setMode(CANSpeedController::ControlMode m);
	void setHeading(double val);
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

class DriveRampAction : public Action{
	DriveSubsystem* drive;
	double speed = 0.15;
	double targetSpeed;
	double targetDistance;
	double currentDistance = 0.0;
	double rotation = 0.0;
	int countTime = 0;
	double rampTime;
	double setPoint;
	double strafe = 0;
public:
	std::string name = "Drive Action";
	DriveRampAction(DriveSubsystem& drive, double Tspeed, double targetDistance, double rampTime = 1.0, double ultraSetPoint = -1):
		drive(&drive),
		targetSpeed(Tspeed),
		targetDistance(targetDistance),
		rampTime(rampTime),
		setPoint(ultraSetPoint)
	{}
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
		if (speed<targetSpeed){
			speed+=((targetSpeed-.15)/(rampTime*10.0));
		}
		strafe = (setPoint == -1)?0.0:drive->autoUltraPIDCalc(setPoint);
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
				drive->mec_drive(strafe,speed,rotation);
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
				drive->mec_drive(strafe,speed,rotation);
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

class StrafeCorrectAction : public Action{
		DriveSubsystem* drive;
		double speed;
		double targetDistance;
		double currentDistance = 0.0;
		double rotation = 0.0;
		double driveY = 0.0;
	public:
		std::string name = "Strafe Action";
		StrafeCorrectAction(DriveSubsystem& drive, double speed, double targetDistance, double vert = 0):
			drive(&drive),
			speed(speed),
			targetDistance(targetDistance),
			driveY(vert)
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
					drive->mec_drive(speed,driveY,rotation);
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
					drive->mec_drive(speed,driveY,rotation);
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
		seenNeed = 1;
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

class SloppyTurnAction: public Action{
	DriveSubsystem* drive;
	double degrees;
	double rotation;
public:
	std::string name = "Turn Action";
	SloppyTurnAction(DriveSubsystem& drive, double degrees):
		drive(&drive),
		degrees(degrees)
	{
		rotation = 0;
	}

	void init(void){
//		drive->setVoltageMode();
//		drive->resetRot();
		drive->setHeading(degrees);
		drive->giveLog("Sloppy Turn Init");
		rotation = drive->getRot();
	}
	ControlFlow call(void){
		drive->resetDistance();
		rotation = drive->gyroPIDCalc(degrees,drive->getRot());
		drive->mec_drive(0,0,rotation);
		if ((degrees > 0) && drive->getRot() > degrees){
			drive->mec_drive(0,0,0);
			return END;
		}else if ((degrees < 0) && drive->getRot() < degrees){
			drive->mec_drive(0,0,0);
			return END;
		}else{

			return CONTINUE;
		}
	}
};


class TurnSettleAction : public WaitAction{
	DriveSubsystem* drive;
	double rotation = 0;
public:
	std::string name = "Turn Action";
	TurnSettleAction(DriveSubsystem& drive, double time):
		WaitAction(time),
		drive(&drive)
	{
	}

	void init(void){
//		drive->setVoltageMode();
//		drive->resetRot();
		drive->giveLog("Turn Init");
		rotation = drive->getRot();
	}
	ControlFlow call(void){
		drive->resetDistance();
		rotation = drive->gyroPIDCalc(0,drive->getRot());
		ControlFlow flow = WaitAction::call();
		if (flow == CONTINUE){
			drive->mec_drive(0,0,rotation);
			return CONTINUE;
		} else {
			drive->mec_drive(0,0,0);
			return END;
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
		int ticksNeeded = 4;
		bool waitExtra = false;
		double setpoint;
		double strafe = 0;
public:
	PhotoDriveAction(DriveSubsystem& drive, double maxDist = 10000.0, double speed = .9, int ticksNeed = 3, bool waitExtra = false, double ultraSetpoint = -1.0):
		drive(&drive),
		maxDist(maxDist),
		speed(speed),
		ticksNeeded(ticksNeed),
		waitExtra(waitExtra),
		setpoint(ultraSetpoint)
		{

	}
	void init(void){
		drive->giveLog("PotoDriveAction Start");
		drive->resetDistance();
		timeTicks = 0;
	}
	ControlFlow call(void){
		strafe = (setpoint == -1)?0.0:drive->autoUltraPIDCalc(setpoint);
		rotation = drive->gyroPIDCalc(0,drive->getRot());
		if(!oldMidValue && drive->getMiddlePhoto()){
			drive->robot.outLog.throwLog("PHOTO MIDDLE SEEN");
			if (sideTicks<0){
				sideTicks = ticksNeeded;
			}
//			return CONTINUE;
		}else if((!oldRightValue && drive->getRightPhoto()) || (!oldLeftValue && drive->getLeftPhoto())){
			drive->robot.outLog.throwLog("PHOTO SIDE SEEN");
			if (sideTicks<0){
				sideTicks = ticksNeeded+1;
			}
//			return CONTINUE;

		}
		drive->mec_drive(strafe,speed,rotation);
		oldMidValue = drive->getMiddlePhoto();
		oldRightValue = drive->getRightPhoto();
		oldLeftValue = drive->getLeftPhoto();
		timeTicks++;
		if (sideTicks == 0){
			if (waitExtra){
				Wait(.05);
			}
			drive->mec_drive(0,0,0);
			drive->robot.outLog.throwLog("PHOTO SEEN STOP", drive->getDistance());
//			drive->giveLog("PhotoDriveAction Complete, MAX DIST");
			return END;
			drive->robot.outLog.throwLog("You should never see this");
		}else if (drive->getDistance()>maxDist && timeTicks >7){
			drive->mec_drive(0,0,0);
			drive->robot.outLog.throwLog("PHOTO MAX DIST", drive->getDistance());
//			drive->giveLog("PhotoDriveAction Complete, MAX DIST");
			return END;
		}
		sideTicks--;

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
	bool volt = false;
	double oldDist = 0;

public:
	std::string name = "Align Action";
	AlignAction(DriveSubsystem& drive, bool volts = false):
		drive(&drive),
		volt(volts)
		{}
	void init(void){
		drive->robot.outLog.throwLog("Align Init");
		if (volt){
			drive->setMode(CANSpeedController::kVoltage);
			drive->robot.outLog.throwLog("Drive Set To VOLTAGE");
		}
		drive->resetDistance();
	}
	ControlFlow call(void){

			rotation = drive->getRot();
			rotation = drive->gyroPIDCalc(0, rotation);
			//Tote Alignment
			//set vars
			leftPhotoVar = drive->getLeftPhoto();
			middlePhotoVar = drive->getMiddlePhoto();
			rightPhotoVar = drive->getRightPhoto();
//			drive->robot.outLog.throwLog("dist", drive->getDistance());
//			drive->robot.outLog.throwLog("oldDist", oldDist);
//			drive->robot.outLog.throwLog("delta dist", fabs(drive->getDistance()-fabs(oldDist)));
			if (fabs(drive->getDistance()-fabs(oldDist))<1000){
				strafe+=.075;
				drive->robot.outLog.throwLog("Align Ramped Up with enc diff of",fabs(drive->getDistance()-oldDist));
			}
			oldDist = drive->getDistance();

			//different cases
			if (leftPhotoVar && !rightPhotoVar){
//				strafe += .05;
				drive->mec_drive(-strafe*(volt?12:1),0,rotation*(volt?12:1));
				return CONTINUE;
			}else if (!leftPhotoVar && rightPhotoVar){
//				strafe += .05;
				drive->mec_drive(strafe*(volt?12:1),0,rotation*(volt?12:1));
				return CONTINUE;
			}else if (!leftPhotoVar && middlePhotoVar && !rightPhotoVar){
				strafe = .5;
				drive->mec_drive(0,0,rotation*(volt?12:1));
				drive->giveLog("Tote Align Seen");
				seen++;
//				return CONTINUE;
			}else /*if((!leftPhotoVar && !middlePhotoVar && !rightPhotoVar) || (leftPhotoVar && middlePhotoVar && rightPhotoVar))*/{
				drive->mec_drive(0,0,0);
				strafe = 0;
				drive->giveLog("ERROR IN PHOTO");
				drive->setMode(CANSpeedController::kPercentVbus);
				return END;
			}
			if (seen == 2){
				drive->mec_drive(0,0,0);
				drive->giveLog("Tote Aligned");
				drive->setMode(CANSpeedController::kPercentVbus);
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

class ChangeHeadingAction : public Action{
	DriveSubsystem* drive;
	double value;
public:
	ChangeHeadingAction(DriveSubsystem& drive, double v):
		drive(&drive),
		value(v)
	{	}

	void init(void){
		drive->robot.outLog.throwLog("heading changed: ", value);
	}

	ControlFlow call(void){
		drive->setHeading(value);
		return END;
	}


};








#endif

