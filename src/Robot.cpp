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
		SmartDashboard::PutNumber("JoystickMultipier", 0.5);
		SmartDashboard::PutNumber("gyroPValue",0.0);
		SmartDashboard::PutNumber("gyroIValue",0.0);
		SmartDashboard::PutNumber("gyroDValue",0.0);

		SmartDashboard::PutNumber("toteHeight", 0.0);
		SmartDashboard::PutNumber("twoToteHeight", 0.0);
		SmartDashboard::PutNumber("bottomEndHeight",0.0);
		SmartDashboard::PutNumber("IRtoteHeight", 0.0);
		SmartDashboard::PutNumber("IRtwoToteHeight", 0.0);
		SmartDashboard::PutNumber("Lift-P-Value", 0.0);
		SmartDashboard::PutNumber("Lift-I-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Value", 0.0);
		SmartDashboard::PutNumber("IR-Lift-P-Value", 0.0);
		SmartDashboard::PutNumber("IR-Lift-I-Value", 0.0);
		SmartDashboard::PutNumber("IR-Lift-D-Value", 0.0);
		SmartDashboard::PutNumber("Lift-Speed",1.0);
		SmartDashboard::PutNumber("DriveVoltageRampRate",6.0);

		//SmartDashboard::PutNumber("voltage ramp rate", 6.0);
		//Auto
		//Drive To Zone
		SmartDashboard::PutNumber("DriveToZoneDist",0.0);
		//Move Set
		SmartDashboard::PutNumber("SetLiftHeightAboveCan",0.0);
		SmartDashboard::PutNumber("SetControlCanDist",0.0);
		SmartDashboard::PutNumber("SetStrafeCanControlDist",0.0);
		SmartDashboard::PutNumber("SetGoFarDist",0.0);
		SmartDashboard::PutNumber("SetBackupDist",0.0);
		SmartDashboard::PutNumber("SetBackupToteDist",0.0);
		SmartDashboard::PutNumber("LiftLowerWait",2.0);
		SmartDashboard::PutNumber("LiftRaiseWait", 2.0);
		autoChoose.AddDefault("Drive to Zone", new std::string("Drive-to-Zone"));
		autoChoose.AddObject("Move Set", new std::string("Move-Set"));
		autoChoose.AddObject("Push All to Zone", new std::string("Push-All-to-Zone"));

		SmartDashboard::PutData("auto-chooser", &autoChoose);
	}
	void Autonomous() {
		robot.teleopEnd();
//		Watchdog& wd = GetWatchdog();
		std::string choice = *(std::string*) autoChoose.GetSelected();
		autoSeq.clear();
		std::cout<<"Auto mode:" <<choice<<std::endl;


			if(choice=="Drive-to-Zone"){
				DriveAction driveToZoneAction(drive, 1.0, SmartDashboard::GetNumber("DriveToZoneDist"));
				autoSeq.add_action(driveToZoneAction);

				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if(choice == "Move-Set"){
				LiftAction liftAboveCan(lift,SmartDashboard::GetNumber("SetLiftHeightAboveCan"));
				autoSeq.add_action(liftAboveCan);
				WaitAction waitToLift(SmartDashboard::GetNumber("LiftRaiseWait"));
				DriveAction driveToControl(drive, 1.0,SmartDashboard::GetNumber("SetControlCanDist"));
				autoSeq.add_action(driveToControl);
				TurnAction turnToControl(drive,.75,90);
				autoSeq.add_action(turnToControl);
				StrafeAction strafeCanControlDist(drive, -.9, SmartDashboard::GetNumber("SetStrafeCanControlDist"));
				autoSeq.add_action(strafeCanControlDist);
				DriveAction endOfAZone(drive, 1.0, SmartDashboard::GetNumber("SetGoFarDist"));
				autoSeq.add_action(endOfAZone);
				DriveAction backupDistance(drive, -1.0, SmartDashboard::GetNumber("SetBackupDist"));
				autoSeq.add_action(backupDistance);
				LiftAction lowerToBottom(lift, SmartDashboard::GetNumber("bottomEndHeight"));
				autoSeq.add_action(lowerToBottom);
				WaitAction waitToLower(SmartDashboard::GetNumber("LiftLowerWait"));
				autoSeq.add_action(waitToLower);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if (choice=="Push-All-to-Zone"){
// move all the totes..... WIN AT LIFE

				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else{
				std::cout<<"Bad auto type"<<std::endl;
			}
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

