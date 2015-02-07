#include "WPILib.h"
#include "CORERobot/CORERobot.h"
#include "Subsystems.h"
const double MOTORUPDATEFREQUENCY = 0.005;

using namespace CORE;

class CORE2015: public SampleRobot {
	CORERobot robot;
	DriveSubsystem drive;
	LiftSubsystem lift;

	AutoSequencer autoSeq;
	SendableChooser autoChoose;
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
		SmartDashboard::PutNumber("JoystickMultipier",1);
		SmartDashboard::PutNumber("gyroPValue",0.0);
		SmartDashboard::PutNumber("gyroIValue",0.0);
		SmartDashboard::PutNumber("gyroDValue",0.0);

		SmartDashboard::PutNumber("toteHeight", 0.0);
		SmartDashboard::PutNumber("twoToteHeight", 0.0);
		SmartDashboard::PutNumber("IRtoteHeight", 0.0);
		SmartDashboard::PutNumber("IRtwoToteHeight", 0.0);
		SmartDashboard::PutNumber("Lift-P-Value", 0.0);
		SmartDashboard::PutNumber("Lift-I-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Value", 0.0);
		SmartDashboard::PutNumber("IR-Lift-P-Value", 0.0);
		SmartDashboard::PutNumber("IR-Lift-I-Value", 0.0);
		SmartDashboard::PutNumber("IR-Lift-D-Value", 0.0);

		autoChoose.AddDefault("Drive to Zone", new std::string("Drive-to-Zone"));
		autoChoose.AddObject("Push to Zone", new std::string("Push-to-Zone"));
		autoChoose.AddObject("Push All to Zone", new std::string("Push-All-to-Zone"));

		SmartDashboard::PutData("auto-chooser", &autoChoose);
	}
	void Autonomous() {
//		Watchdog& wd = GetWatchdog();
//		std::string choice = *(std::string*) autoChoose.GetSelected();
//		autoSeq.clear();
//		std::cout<<"Auto mode:" <<choice<<std::endl;
//
//
//			if(choice=="Drive-to-Zone"){
//
//
//				autoSeq.init();
//				while (IsAutonomous() and !IsDisabled()) {
//					autoSeq.iter();
//					Wait(0.05);
//				}
//			}else if(choice == "Push-to-Zone"){
//
//
//				autoSeq.init();
//				while (IsAutonomous() and !IsDisabled()) {
//					autoSeq.iter();
//					Wait(0.05);
//				}
//			}else if (choice=="Push-All-to-Zone"){
//
//
//				autoSeq.init();
//				while (IsAutonomous() and !IsDisabled()) {
//					autoSeq.iter();
//					Wait(0.05);
//				}
//			}else{
//				std::cout<<"Bad auto type"<<std::endl;
//			}
	}
//
	void OperatorControl() {
		robot.teleopInit();
////		Watchdog& wd = GetWatchdog();
////		wd.SetExpiration(.5);
////		wd.SetEnabled(true);
////
		while (IsOperatorControl() && !IsDisabled()) {
////			wd.Feed();
			robot.teleop();
			Wait(0.05); // wait for a motor update time
////			SmartDashboard::PutBoolean("safety",drive.driveMotors.IsSafetyEnabled());
		};
	};

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

