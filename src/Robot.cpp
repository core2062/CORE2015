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
		SmartDashboard::PutNumber("FLP",0.0);
		SmartDashboard::PutNumber("FLI",0.0);
		SmartDashboard::PutNumber("FLD",0.0);
		SmartDashboard::PutNumber("BLP",0.0);
		SmartDashboard::PutNumber("BLI",0.0);
		SmartDashboard::PutNumber("BLD",0.0);
		SmartDashboard::PutNumber("FRP",0.0);
		SmartDashboard::PutNumber("FRI",0.0);
		SmartDashboard::PutNumber("FRD",0.0);
		SmartDashboard::PutNumber("BRP",0.0);
		SmartDashboard::PutNumber("BRI",0.0);
		SmartDashboard::PutNumber("BRD",0.0);
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

