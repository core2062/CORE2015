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
		lift(robot,drive),
		autoSeq(robot.outLog)
	{
		robot.add(drive);
		robot.add(lift);
	}

	void RobotInit() {
		robot.robotInit();

		//////////////////////////////////////////
		//            Actuator Values           //
		//////////////////////////////////////////
		//Joystick
		SmartDashboard::PutNumber("JoystickMultiplier", 0.5);

		//Drive PID
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
		SmartDashboard::PutNumber("MaxVel",20.0);
		SmartDashboard::PutBoolean("SimpleDrive",true);

		//Gyro Values
		SmartDashboard::PutNumber("gyroPValue",0.05);
		SmartDashboard::PutNumber("gyroIValue",0.0);
		SmartDashboard::PutNumber("gyroDValue",0.0);
		SmartDashboard::PutNumber("gyroDead",.009);

		//UltraSonic Values
		SmartDashboard::PutNumber("ultraPValue",0.075);
		SmartDashboard::PutNumber("ultraIValue",0.0);
		SmartDashboard::PutNumber("ultraDValue",0.0);
		SmartDashboard::PutNumber("ultraSetPoint", 30);
		SmartDashboard::PutNumber("ultraVConst", 4.84);

		//Tote Heights
		SmartDashboard::PutNumber("toteHeight", 7200.0);
		SmartDashboard::PutNumber("twoToteHeight", 21000.0);
		SmartDashboard::PutNumber("bottomHeight",1200.0);
		SmartDashboard::PutNumber("magicToteHeight",4600.0);
		SmartDashboard::PutNumber("OverToteHeight", 12000.0);

		//Lift Values
		SmartDashboard::PutNumber("Lift-P-Up-Value", 2.5);
		SmartDashboard::PutNumber("Lift-I-Up-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Up-Value", 0.0);
		SmartDashboard::PutNumber("Lift-P-Down-Value", 1.0);
		SmartDashboard::PutNumber("Lift-I-Down-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Down-Value", 0.0);
		SmartDashboard::PutNumber("Lift-Speed",-1.0);

		//Unused
//		SmartDashboard::PutNumber("DriveVoltageRampRate",6.0);
//		SmartDashboard::PutNumber("Gyro Sensitivity", 0.0065);
//		SmartDashboard::PutNumber("voltage ramp rate", 6.0);

		///////////////////////////////////////////
		//                 Auto Values           //
		///////////////////////////////////////////
		SmartDashboard::PutNumber("Small",1000.0);
		//Drive To Zone
		SmartDashboard::PutNumber("DriveToZoneDist",0.0);

		//Push to Zone
		SmartDashboard::PutNumber("PushToZoneDist",0.0);

		//Move Set
		SmartDashboard::PutNumber("SetLiftHeightAboveCan",1200.0);
		SmartDashboard::PutNumber("SetControlCanDist",10000.0);
		SmartDashboard::PutNumber("SetGoFarDist",5000.0);
		//Unused
		//SmartDashboard::PutNumber("SetStrafeCanControlDist",0.0);
		//SmartDashboard::PutNumber("SetBackupDist",0.0);
		//SmartDashboard::PutNumber("SetBackupToteDist",0.0);
		//SmartDashboard::PutNumber("LiftLowerWait",2.0);
		//SmartDashboard::PutNumber("LiftRaiseWait", 2.0);

		//Two Totes
		SmartDashboard::PutNumber("TTGotoTote",1000.0);
		SmartDashboard::PutNumber("TTStrafe",4000.0);
		SmartDashboard::PutNumber("TTAutoZone",0.0);

//		SmartDashboard::PutNumber("BinSlapDrive", 2.0);
		SmartDashboard::PutNumber("BinSlapLiftDown", 100.0);
		SmartDashboard::PutNumber("BinSlapLiftUp", 12000.0);
//		SmartDashboard::PutNumber("BinSlap", 0.0);
		SmartDashboard::PutNumber("Small", 1000);
		SmartDashboard::PutNumber("BinSlapMediumDriveForward", 10000);
		SmartDashboard::PutNumber("BinSlapDrive-To-Autozone", 2000);


		//ShakeN'Bake (Uses Slap Values otherwise
		SmartDashboard::PutNumber("shakeNBakeStrafe", 0);

		//Add Auto Choices
		autoChoose.AddDefault("Drive to Zone", new std::string("Drive-to-Zone"));
		autoChoose.AddObject("Move Set", new std::string("Move-Set"));
		autoChoose.AddObject("Bin Slap", new std::string("Bin-Slap"));
		autoChoose.AddObject("Shake n' Bake", new std::string("Shake-n'-Bake"));
		autoChoose.AddObject("Two Tote", new std::string("Two-Tote"));
		autoChoose.AddObject("Push Bin or Tote", new std::string("Push-to-Zone"));
		autoChoose.AddObject("Three Tote No Bins",new std::string("Three-Tote-Norm"));

		SmartDashboard::PutData("auto-chooser", &autoChoose);

	}

	void Autonomous() {
		robot.teleopEnd();
		robot.outLog.startTime();
		robot.outLog.setMode(OutLog::AUTO);
		std::string choice = *(std::string*) autoChoose.GetSelected();
		autoSeq.clear();
		std::cout<<"Auto mode:" <<choice<<std::endl;
		robot.outLog.throwLog(choice);

			if(choice=="Drive-to-Zone"){
				robot.outLog.throwLog("chose dtz");
				DriveAction driveToZoneAction(drive, 1.0, SmartDashboard::GetNumber("DriveToZoneDist"));
				autoSeq.add_action(driveToZoneAction);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if(choice=="Push-to-Zone"){
					robot.outLog.throwLog("Push to Zone");

					DriveAction pushToZoneAction(drive, 1.0, SmartDashboard::GetNumber("PushToZoneDist"));
					autoSeq.add_action(pushToZoneAction);
					autoSeq.init();
					while (IsAutonomous() and !IsDisabled()) {
						autoSeq.iter();
						Wait(0.05);
					}
			}else if(choice == "Move-Set"){
				robot.outLog.throwLog("Move set");
				LiftAction liftAboveCan(lift,SmartDashboard::GetNumber("SetLiftHeightAboveCan"));
				autoSeq.add_action(liftAboveCan);
				DriveAction driveToControl(drive, 1.0,SmartDashboard::GetNumber("SetControlCanDist"));
				autoSeq.add_action(driveToControl);
				TurnAction turnToControl(drive,90);
				autoSeq.add_action(turnToControl);
				DriveAction endOfAZone(drive, 1.0, SmartDashboard::GetNumber("SetGoFarDist"));
				autoSeq.add_action(endOfAZone);
				LiftAction lowerToBottom(lift, SmartDashboard::GetNumber("bottomEndHeight",100.0));
				autoSeq.add_action(lowerToBottom);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if (choice=="Bin-Slap"){
				robot.outLog.throwLog("Slaperino");

				// move all the totes.....#smackattack
//				LiftAction binSlap_LiftDown(lift, 100);
//				autoSeq.add_action(binSlap_LiftDown);
				LiftAction binSlap_LiftUp(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUp);
				DriveAction binSlap_SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(binSlap_SmallDriveForward);
				//We gon knock that bin now
				DriveAction binSlap_MediumDriveForward(drive, 0.9, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward);
//				PhotoDriveAction photoOne(drive);
//				autoSeq.add_action(photoOne);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction binSlap_LiftDown2(lift, 100);
				autoSeq.add_action(binSlap_LiftDown2);
				LiftAction binSlap_LiftUp2(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUp2);
				DriveAction binSlap_SmallDriveForward2(drive, 0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(binSlap_SmallDriveForward2);
				//We gon knock that bin now 2
				DriveAction binSlap_MediumDriveForward2(drive, 0.9, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward2);
//				PhotoDriveAction photoTwo(drive);
//				autoSeq.add_action(photoTwo);
//				AlignAction align2(drive);
//				autoSeq.add_action(align2);
				LiftAction binSlap_LiftDown3(lift, 100);
				autoSeq.add_action(binSlap_LiftDown3);
				LiftAction binSlap_LiftUp3(lift, SmartDashboard::GetNumber("OverToteHeight", true));
				autoSeq.add_action(binSlap_LiftUp3);
				TurnAction binSlap_Turn(drive, 90);
				autoSeq.add_action(binSlap_Turn);
				DriveAction binSlap_DriveToAutozone(drive, 0.9, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone"));
				autoSeq.add_action(binSlap_DriveToAutozone);
				LiftAction binSlap_LiftDown4(lift, 100);
				autoSeq.add_action(binSlap_LiftDown4);
				DriveAction binSlap_Back(drive, -0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(binSlap_Back);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if (choice=="Shake-n'-Bake"){

				robot.outLog.throwLog("shake it n' bake it");
				// move all the totes.....#smackattack
//				LiftAction binSlap_LiftDown(lift, 100);
//				autoSeq.add_action(binSlap_LiftDown);
				LiftAction binSlap_LiftUp(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(binSlap_LiftUp);
				StrafeAction strafeOne(drive, -.9, SmartDashboard::GetNumber("shakeNBakeStrafe"));
				autoSeq.add_action(strafeOne);
				DriveAction binSlap_SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(binSlap_SmallDriveForward);
				StrafeAction strafeTwo(drive, .9, SmartDashboard::GetNumber("shakeNBakeStrafe"));
				autoSeq.add_action(strafeTwo);
				//We gon knock that bin now
				PhotoDriveAction photoOne(drive);
				autoSeq.add_action(photoOne);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction binSlap_LiftDown2(lift, 100);
				autoSeq.add_action(binSlap_LiftDown2);
				LiftAction binSlap_LiftUp2(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(binSlap_LiftUp2);
				StrafeAction strafeThree(drive, -.9, SmartDashboard::GetNumber("shakeNBakeStrafe"));
				autoSeq.add_action(strafeThree);
				DriveAction binSlap_SmallDriveForward2(drive, 0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(binSlap_SmallDriveForward2);
				//We gon knock that bin now 2
				StrafeAction strafeFour(drive, .9, SmartDashboard::GetNumber("shakeNBakeStrafe"));
				autoSeq.add_action(strafeFour);
				PhotoDriveAction photoTwo(drive);
				autoSeq.add_action(photoTwo);
				AlignAction align2(drive);
				autoSeq.add_action(align2);
				LiftAction binSlap_LiftDown3(lift, 100);
				autoSeq.add_action(binSlap_LiftDown3);
				LiftAction binSlap_LiftUp3(lift, SmartDashboard::GetNumber("BinSlapLiftUp", true));
				autoSeq.add_action(binSlap_LiftUp3);
				TurnAction binSlap_Turn(drive, 90);
				autoSeq.add_action(binSlap_Turn);
				DriveAction binSlap_DriveToAutozone(drive, 0.9, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone"));
				autoSeq.add_action(binSlap_DriveToAutozone);
				LiftAction binSlap_LiftDown4(lift, 100);
				autoSeq.add_action(binSlap_LiftDown4);
				DriveAction binSlap_Back(drive, -0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(binSlap_Back);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if (choice=="Three-Tote-Norm"){

				robot.outLog.throwLog("Norm");
				// move all the totes
//				LiftAction LiftDown(lift, 100);
//				autoSeq.add_action( LiftDown);
				//Over Tote
				LiftAction LiftUp(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action( LiftUp);
				//GoTo Next, drive forward
				PhotoDriveAction photoOne(drive);
				autoSeq.add_action(photoOne);
				DriveAction SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action( SmallDriveForward);
				//Align and Get Tote
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction LiftDown2(lift, 100);
				autoSeq.add_action( LiftDown2);
				LiftAction LiftUp2(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action( LiftUp2);
				//Goto Next, small drive Forward
				PhotoDriveAction photoTwo(drive);
				autoSeq.add_action(photoTwo);
				DriveAction SmallDriveForward2(drive, 0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action( SmallDriveForward2);
				//Align and Lift Up
				AlignAction align2(drive);
				autoSeq.add_action(align2);
				LiftAction LiftDown3(lift, 100);
				autoSeq.add_action( LiftDown3);
				LiftAction LiftUp3(lift, SmartDashboard::GetNumber("OverToteHeight", true));
				autoSeq.add_action( LiftUp3);
				//Turn and drive to zone, drop and go back
				TurnAction Turn(drive, 90);
				autoSeq.add_action( Turn);
				DriveAction DriveToAutozone(drive, 0.9, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone"));
				autoSeq.add_action( DriveToAutozone);
				LiftAction LiftDown4(lift, 100);
				autoSeq.add_action( LiftDown4);
				DriveAction Back(drive, -0.9, SmartDashboard::GetNumber("Small"));
				autoSeq.add_action( Back);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if(choice=="Two-Tote"){
				robot.outLog.throwLog("two totez");
				LiftAction liftAboveCan(lift,SmartDashboard::GetNumber("magicToteHeight"));
				autoSeq.add_action(liftAboveCan);
//				DriveAction goBack(drive,-.9,-1000);
//				autoSeq.add_action(goBack);
				TurnAction turnToSlap(drive,180,2);
				autoSeq.add_action(turnToSlap);
				LiftAction liftUp(lift,SmartDashboard::GetNumber("OverToteHeight"));
				autoSeq.add_action(liftUp);
				StrafeAction strafeOne(drive, .9, SmartDashboard::GetNumber("TTStrafe"));
				autoSeq.add_action(strafeOne);
				PhotoDriveAction photoOne(drive);
				autoSeq.add_action(photoOne);
				DriveAction goToTote(drive,1.0,SmartDashboard::GetNumber("TTGotoTote"));
				autoSeq.add_action(goToTote);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction liftDown(lift,100);
				autoSeq.add_action(liftDown);
				LiftAction liftCarry(lift,SmartDashboard::GetNumber("OverToteHeight",0.0),true);
				autoSeq.add_action(liftCarry);
				TurnAction turnRight(drive,-90);
				autoSeq.add_action(turnRight);
				DriveAction endOfAZone(drive, 1.0, SmartDashboard::GetNumber("TTAutoZone"));
				autoSeq.add_action(endOfAZone);
				LiftAction lowerToBottom(lift, 100.0);
				autoSeq.add_action(lowerToBottom);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else{
				std::cout<<"Bad auto type"<<std::endl;
			}
	}

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

START_ROBOT_CLASS(CORE2015);


