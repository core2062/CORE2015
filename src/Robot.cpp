#include "WPILib.h"
#include "CORERobot/CORERobot.h"
#include "Subsystems.h"
const double MOTORUPDATEFREQUENCY = 0.005;

using namespace CORE;

class CORE2015: public SampleRobot {
	CORERobot robot;
	DriveSubsystem drive;
//	LiftSubsystem lift;
public:
	CORE2015() :
		robot(),
		drive(robot)
//		lift(robot)
	{
		robot.add(drive);
//		robot.add(lift);
	}

	void RobotInit() {
		robot.robotInit();
	}
	void Autonomous() {
//		Watchdog& wd = GetWatchdog();



//			wd.Feed();
			while (IsAutonomous() and !IsDisabled()) {
//				wd.Feed();
				Wait(MOTORUPDATEFREQUENCY); // wait for a motor update time
			}

	}

	void OperatorControl() {
		SmartDashboard::PutBoolean("safety",drive.driveMotors.IsSafetyEnabled());
		robot.teleopInit();
//		Watchdog& wd = GetWatchdog();
//		wd.SetExpiration(.5);
//		wd.SetEnabled(true);
//
		while (IsOperatorControl() && !IsDisabled()) {
//			wd.Feed();
			robot.teleop();
			SmartDashboard::PutBoolean("safety",drive.driveMotors.IsSafetyEnabled());
			Wait(MOTORUPDATEFREQUENCY); // wait for a motor update time

		}
		drive.driveMotors.SetSafetyEnabled(false);
		SmartDashboard::PutBoolean("safety",drive.driveMotors.IsSafetyEnabled());
		robot.teleopEnd();
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
			Wait(MOTORUPDATEFREQUENCY);
		}
//		robot.compressor->Stop();
	}
};

START_ROBOT_CLASS(CORE2015)
;

