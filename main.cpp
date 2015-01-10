#include "WPILib.h" 
#include "CORERobot/CORERobot.h"
#include "Subsystems.h"

using namespace CORE;

class CORE2014: public SampleRobot {

	CORERobot robot;

	DriveSubsystem drive;

	AutoSequencer autoSeq;
	bool firstRun;
public:
	CORE2014() :
		robot(),
		drive(robot),
		autoSeq(),
		firstRun(false)
	{
			robot.add(drive);
	}

	void RobotInit() {
		robot.robotInit();
	}
	void Autonomous() {
		
	}
	void OperatorControl() {
		robot.teleopInit();
//		Watchdog& wd = GetWatchdog();
//		wd.SetExpiration(.5);
//		wd.SetEnabled(true);
		
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
		while(IsTest() && !IsDisabled()){
//			wd.Feed();
			Wait(0.05);
		}
	}
};

START_ROBOT_CLASS(CORE2014)
;

