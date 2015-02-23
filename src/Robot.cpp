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
	Timer timeVal;
public:
	CORE2015() :
		robot(),
		drive(robot),
		lift(robot),
		autoSeq(robot.outLog)
	{
		robot.add(drive);
		robot.add(lift);
	}

	void RobotInit() {
		robot.robotInit();
//		SmartDashboard::PutNumber("FrontLeftPValue",0.0);
//		SmartDashboard::PutNumber("FrontLeftIValue",0.0);
//		SmartDashboard::PutNumber("FrontLeftDValue",0.0);
//		SmartDashboard::PutNumber("BackLeftPValue",0.0);
//		SmartDashboard::PutNumber("BackLeftIValue",0.0);
//		SmartDashboard::PutNumber("BackLeftDValue",0.0);
//		SmartDashboard::PutNumber("FrontRightPValue",0.0);
//		SmartDashboard::PutNumber("FrontRightIValue",0.0);
//		SmartDashboard::PutNumber("FrontRightDValue",0.0);
//		SmartDashboard::PutNumber("BackRightPValue",0.0);
//		SmartDashboard::PutNumber("BackRightIValue",0.0);
//		SmartDashboard::PutNumber("BackRightDValue",0.0);
		SmartDashboard::PutNumber("JoystickMultipier", 0.5);
		SmartDashboard::PutNumber("gyroPValue",0.05);
		SmartDashboard::PutNumber("gyroIValue",0.0);
		SmartDashboard::PutNumber("gyroDValue",0.0);

		SmartDashboard::PutNumber("toteHeight", 7200.0);
		SmartDashboard::PutNumber("twoToteHeight", 21000.0);
		SmartDashboard::PutNumber("bottomHeight",1200.0);
//		SmartDashboard::PutNumber("IRtoteHeight", 0.0);
//		SmartDashboard::PutNumber("IRtwoToteHeight", 0.0);
		SmartDashboard::PutNumber("Lift-P-Up-Value", 2.5);
		SmartDashboard::PutNumber("Lift-I-Up-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Up-Value", 0.0);
		SmartDashboard::PutNumber("Lift-P-Down-Value", 1.0);
		SmartDashboard::PutNumber("Lift-I-Down-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Down-Value", 0.0);
//		SmartDashboard::PutNumber("IR-Lift-P-Value", 0.0);
//		SmartDashboard::PutNumber("IR-Lift-I-Value", 0.0);
//		SmartDashboard::PutNumber("IR-Lift-D-Value", 0.0);
		SmartDashboard::PutNumber("Lift-Speed",-1.0);
		SmartDashboard::PutNumber("DriveVoltageRampRate",6.0);
		SmartDashboard::PutNumber("Gyro Sensitivity", 0.0065);

//		SmartDashboard::PutNumber("voltage ramp rate", 6.0);
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
		SmartDashboard::PutNumber("BinSlapDrive", 2.0);
		SmartDashboard::PutNumber("BinSlapLiftDown", 0.0);
		SmartDashboard::PutNumber("BinSlapLiftUp", 0.0);
		SmartDashboard::PutNumber("BinSlap", 0.0);
		SmartDashboard::PutNumber("BinSlapSmallDriveForward", 0.0);
		SmartDashboard::PutNumber("BinSlapMediumDriveForward", 0.0);
		SmartDashboard::PutNumber("BinSlapDrive-To-Autozone", 0.0);
		autoChoose.AddDefault("Drive to Zone", new std::string("Drive-to-Zone"));
		autoChoose.AddObject("Move Set", new std::string("Move-Set"));
		autoChoose.AddObject("Bin Slap", new std::string("Bin-Slap"));

		SmartDashboard::PutData("auto-chooser", &autoChoose);

	}
	void Autonomous() {
		robot.teleopEnd();
		robot.outLog.startTime();
		robot.outLog.setMode(OutLog::AUTO);
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
				TurnAction turnToControl(drive,.90);
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
			}else if (choice=="Bin-Slap"){
				// move all the totes.....#smackattack
//				LiftAction binSlap_LiftDown(lift, SmartDashboard::GetNumber("BinSlapLiftDown"));
//				autoSeq.add_action(binSlap_LiftDown);
				LiftAction binSlap_LiftUp(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(binSlap_LiftUp);
				DriveAction binSlap_SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("BinSlapSmallDriveForward"));
				autoSeq.add_action(binSlap_SmallDriveForward);
				//We gon knock that bin now
				DriveAction binSlap_MediumDriveForward(drive, 0.9, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction binSlap_LiftDown2(lift, SmartDashboard::GetNumber("BinSlapLiftDown"));
				autoSeq.add_action(binSlap_LiftDown2);
				LiftAction binSlap_LiftUp2(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(binSlap_LiftUp2);
				DriveAction binSlap_SmallDriveForward2(drive, 0.9, SmartDashboard::GetNumber("BinSlapSmallDriveForward"));
				autoSeq.add_action(binSlap_SmallDriveForward2);
				//We gon knock that bin now 2
				DriveAction binSlap_MediumDriveForward2(drive, 0.9, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward2);
				AlignAction align2(drive);
				autoSeq.add_action(align2);
				LiftAction binSlap_LiftDown3(lift, SmartDashboard::GetNumber("BinSlapLiftDown"));
				autoSeq.add_action(binSlap_LiftDown3);
				LiftAction binSlap_LiftUp3(lift, SmartDashboard::GetNumber("BinSlapLiftUp", true));
				autoSeq.add_action(binSlap_LiftUp3);
				TurnAction binSlap_Turn(drive, 90);
				autoSeq.add_action(binSlap_Turn);
				DriveAction binSlap_DriveToAutozone(drive, 0.9, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone"));
				autoSeq.add_action(binSlap_DriveToAutozone);
				LiftAction binSlap_LiftDown4(lift, SmartDashboard::GetNumber("BinSlapLiftDown"));
				autoSeq.add_action(binSlap_LiftDown4);
				DriveAction binSlap_Back(drive, -0.9, SmartDashboard::GetNumber("BinSlapSmallDriveForward"));
				autoSeq.add_action(binSlap_Back);
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
		timeVal.Start();
		timeVal.Reset();
		lift.liftMotor.SetExpiration(.15);
		drive.frontLeft.SetExpiration(.15);
		drive.backLeft.SetExpiration(.15);
		drive.frontRight.SetExpiration(.15);
		drive.backRight.SetExpiration(.15);
		robot.teleopInit();
////		Watchdog& wd = GetWatchdog();
////		wd.SetExpiration(.5);
////		wd.SetEnabled(true);
////
		SmartDashboard::PutNumber("Timer", timeVal.Get());
		while (IsOperatorControl() && !IsDisabled()) {
			SmartDashboard::PutNumber("Timer", timeVal.Get());
			timeVal.Reset();
////			wd.Feed();
			robot.teleop();
			Wait(0.05); // wait for a motor update time
////			SmartDashboard::PutBoolean("safety",drive.driveMotors.IsSafetyEnabled());
		};
		robot.teleopEnd();
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

