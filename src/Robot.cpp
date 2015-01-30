#include "WPILib.h"
#include "CORERobot/CORERobot.h"
#include "Subsystems.h"


using namespace CORE;

class CORE2015: public SampleRobot {

	CORERobot robot;
	DriveSubsystem drive;
	LiftSubsystem lift;
public:
	CORE2015() :
		robot(),
		drive(robot),
		lift(robot)
	{
		robot.add(drive);
		robot.add(lift);
	}

	void RobotInit() {
		robot.robotInit();
		SmartDashboard::PutNumber("FrontLeftPValue",0.0);
		SmartDashboard::PutNumber("FrontLeftIValue",0.0);
		SmartDashboard::PutNumber("FrontLeftDValue",0.0);
		SmartDashboard::PutNumber("BackLeftPValue",0.0);
		SmartDashboard::PutNumber("BackLeftIValue",0.0);
		SmartDashboard::PutNumber("BackLeftDValue",0.0);
		SmartDashboard::PutNumber("FrontRightPValue",0.0);
		SmartDashboard::PutNumber("FrontRightIValue",0.0);
		SmartDashboard::PutNumber("FrontRightDValue",0.0);
		SmartDashboard::PutNumber("BackRightPValue",0.0);
		SmartDashboard::PutNumber("BackRightIValue",0.0);
		SmartDashboard::PutNumber("BackRightDValue",0.0);
		SmartDashboard::PutNumber("JoystickMultipier",8);
	}
	void Autonomous() {
//		Watchdog& wd = GetWatchdog();



//			wd.Feed();
			while (IsAutonomous() and !IsDisabled()) {
//				wd.Feed();
				Wait(0.05); // wait for a motor update time
			}

	}

	void OperatorControl() {
		robot.teleopInit();
//		Watchdog& wd = GetWatchdog();
//		wd.SetExpiration(.5);
//		wd.SetEnabled(true);
//
		while (IsOperatorControl() && !IsDisabled()) {
//			wd.Feed();
			robot.teleop();
			Wait(0.05); // wait for a motor update time
		}
	}

	/**
	 * Runs during test mode
	 */
	void Test() {
//		Watchdog& wd = GetWatchdog();
//		wd.SetExpiration(.5);
//		wd.SetEnabled(true);
//		robot.requirePneumatics();
//		robot.compressor->Enabled();
		while(IsTest() && !IsDisabled()){
//			wd.Feed();
//			robot.compressor->Start();
			Wait(0.05);
		}
//		robot.compressor->Stop();
	}
};

START_ROBOT_CLASS(CORE2015)
;

