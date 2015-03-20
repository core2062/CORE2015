#include "WPILib.h"
#include "CORERobot/CORERobot.h"
#include "Subsystems.h"
const double MOTORUPDATEFREQUENCY = 0.005;

using namespace CORE;

class CORE2015: public SampleRobot {
	CORERobot robot;
	DriveSubsystem drive;
	LiftSubsystem lift;
//	AuxiliarySubsystem Auxiliary;

	AutoSequencer autoSeq;
	SendableChooser autoChooser;
	Timer timeVal;
	double loopTime = 0.1;
public:
	CORE2015() :
		robot(),
		drive(robot),
		lift(robot,drive),
//		Auxiliary(robot),
		autoSeq(robot.outLog)
	{
		robot.add(drive);
		robot.add(lift);
//		robot.add(Auxiliary);
	}

	void RobotInit() {
		robot.robotInit();


//		robot.CD.putNum("test value", 1.4);
//		robot.CD.updateSD();
//		robot.outLog.throwLog(robot.CD.getNum("test value"));

		//////////////////////////////////////////
		//            Actuator Values           //
		//////////////////////////////////////////
		//Joystick
		SmartDashboard::PutNumber("JoystickMultiplier", 0.5);

		//Drive PID
		SmartDashboard::PutNumber("FrontLeftPValue",0.06);
		SmartDashboard::PutNumber("FrontLeftIValue",0.00085);
		SmartDashboard::PutNumber("FrontLeftDValue",0.01);
		SmartDashboard::PutNumber("FrontLeftFValue",.25);
		SmartDashboard::PutNumber("BackLeftPValue",0.06);
		SmartDashboard::PutNumber("BackLeftIValue",0.00085);
		SmartDashboard::PutNumber("BackLeftDValue",0.01);
		SmartDashboard::PutNumber("BackLeftFValue",.25);
		SmartDashboard::PutNumber("FrontRightPValue",0.06);
		SmartDashboard::PutNumber("FrontRightIValue",0.00085);
		SmartDashboard::PutNumber("FrontRightDValue",0.01);
		SmartDashboard::PutNumber("FrontRightFValue",.25);
		SmartDashboard::PutNumber("BackRightPValue",0.06);
		SmartDashboard::PutNumber("BackRightIValue",0.00085);
		SmartDashboard::PutNumber("BackRightDValue",0.01);
		SmartDashboard::PutNumber("BackRightFValue",.25);
		SmartDashboard::PutNumber("MaxVel",2600.0);
		SmartDashboard::PutBoolean("SimpleDrive",true);

		//Gyro Values
		SmartDashboard::PutNumber("gyroPValue",0.075);
		SmartDashboard::PutNumber("gyroIValue",0.0);
		SmartDashboard::PutNumber("gyroAutoIValue",0.001);
		SmartDashboard::PutNumber("gyroDValue",0.0);
		SmartDashboard::PutNumber("gyroDead",.009);
		SmartDashboard::PutBoolean("disableGyro", false);
		SmartDashboard::PutBoolean("deconstructGyro", false);



		//UltraSonic Values
		SmartDashboard::PutNumber("ultraPValue",0.075);
		SmartDashboard::PutNumber("ultraIValue",0.0);
		SmartDashboard::PutNumber("ultraDValue",0.0);
		SmartDashboard::PutNumber("ultraSetPoint", 26.0);
		SmartDashboard::PutNumber("ultraVConst", 4.84);
		SmartDashboard::PutNumber("CenterSpeed",1.0);

		//Tote Heights
		SmartDashboard::PutNumber("toteHeight", 7200.0);
		SmartDashboard::PutNumber("twoToteHeight", 21000.0);
		SmartDashboard::PutNumber("bottomHeight",1200.0);
		SmartDashboard::PutNumber("magicToteHeight",4600.0);
		SmartDashboard::PutNumber("OverToteHeight", 12000.0);
		SmartDashboard::PutNumber("pickupToteHeight",3000.0);

		//Lift Values
		SmartDashboard::PutNumber("Lift-P-Up-Value", 2.5);
		SmartDashboard::PutNumber("Lift-I-Up-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Up-Value", 0.0);
		SmartDashboard::PutNumber("Lift-P-Down-Value", 1.0);
		SmartDashboard::PutNumber("Lift-I-Down-Value", 0.0);
		SmartDashboard::PutNumber("Lift-D-Down-Value", 0.0);
		SmartDashboard::PutNumber("Lift-Speed",-1.0);


		///////////////////////////////////////////
		//                 Auto Values           //
		///////////////////////////////////////////
		SmartDashboard::PutNumber("Small",1000.0);
		//Drive To Zone
		SmartDashboard::PutNumber("DriveToZoneDist",10000.0);

		//Push to Zone
		SmartDashboard::PutNumber("PushToZoneDist",0.0);

		//Move Set
		SmartDashboard::PutNumber("TLiftHeightAboveCan",12000.0);
		SmartDashboard::PutNumber("TControlCanDist",7500.0);
		SmartDashboard::PutNumber("TGoFarDist",27000.0);
		SmartDashboard::PutNumber("TBackDist",4000.0);

		//Two Totes
		SmartDashboard::PutNumber("TTForwardOne", 3800.0);
		SmartDashboard::PutNumber("TTForwardTwo", 3000.0);
		SmartDashboard::PutNumber("TTGotoTote",1000.0);
		SmartDashboard::PutNumber("TTStrafe",15000.0);
		SmartDashboard::PutNumber("TTAutoZone",27000.0);
		SmartDashboard::PutNumber("TTHitAngle",90.0);
		SmartDashboard::PutNumber("TTMaxPhotoDist",10000.0);
		SmartDashboard::PutNumber("TTBackDist",2000.0);


		//Bin Slap
//		SmartDashboard::PutNumber("BinSlapDrive", 2.0);
		SmartDashboard::PutNumber("BinSlapLiftDown", 100.0);
		SmartDashboard::PutNumber("BinSlapLiftUp", 12000.0);
//		SmartDashboard::PutNumber("BinSlap", 0.0);
		SmartDashboard::PutNumber("Small", 1000);
		SmartDashboard::PutNumber("BinSlapMediumDriveForward", 10000);
		SmartDashboard::PutNumber("BinSlapDrive-To-Autozone", 24000);
		SmartDashboard::PutNumber("BinSlapDriveBack", 8000);
		SmartDashboard::PutNumber("BinSlapStrafeDist",2000);



		SmartDashboard::PutNumber("TTBinSlapDrive-To-Autozone", 19000);


		//Two Tote Alt
		SmartDashboard::PutNumber("TTAStrafeDist",10000.0);
		SmartDashboard::PutNumber("TTAForwardOne",11000.0);
//		SmartDashboard::PutNumber("TTAForwardTwo",0.0);
		SmartDashboard::PutNumber("TTAAutoZoneDist",27000.0);
		SmartDashboard::PutNumber("TTABackDist",4000.0);
		SmartDashboard::PutNumber("TTAPhotoDist",10000.0);
		SmartDashboard::PutNumber("TTAStrafeTwoDist",13000.0);


		//Shake and Bake
		SmartDashboard::PutNumber("SNBStrafeDist",7500.0);
		SmartDashboard::PutNumber("SNBForwardOne",9000.0);
		SmartDashboard::PutNumber("SNBAutoZoneDist",31500.0);
		SmartDashboard::PutNumber("SNBBackDist",4000.0);
		SmartDashboard::PutNumber("SNBPhotoDist",10000.0);
		SmartDashboard::PutNumber("SNBStrafeTwoDist",5000.0);


		// Push Over (uses Slap Values otherwise)
		SmartDashboard::PutNumber("PushOver-TurnAngle", 35);
		SmartDashboard::PutNumber("PushOver-SmallDrive", 500);
		SmartDashboard::PutNumber("PushOver-strafeBack", 300);
		SmartDashboard::PutNumber("PushOver-longDriveForward", 10000);

		//Add Auto Choices
		autoChooser.AddDefault("Drive to Zone", new std::string("Drive-to-Zone"));
		autoChooser.AddObject("One Tote", new std::string("One-Tote"));
		autoChooser.AddObject("Bin Slap", new std::string("Bin-Slap"));
//		autoChooser.AddObject("Shake n' Bake", new std::string("Shake-n'-Bake"));
		autoChooser.AddObject("Two Tote", new std::string("Two-Tote"));
//		autoChooser.AddObject("Push Bin or Tote", new std::string("Push-to-Zone"));
//		autoChooser.AddObject("Three Tote No Bins",new std::string("Three-Tote-Norm"));
//		autoChooser.AddObject("Push Over", new std::string("Push-Over"));
		autoChooser.AddObject("Two tote alt", new std::string("Two-Tote-Alt"));
		autoChooser.AddObject("Do Nothing", new std::string("Do-Nothing"));
		autoChooser.AddObject("Two Tote Punch", new std::string("Two-Tote-Punch"));
		SmartDashboard::PutData("auto-choose", &autoChooser);

		// gyro deconstructing code
		for(int i = 0; i<5; i++){
			if(i==4){
				SmartDashboard::PutBoolean("disableGyro", true);
				robot.outLog.throwLog("[PROBLEM] Gyro rate not stabilized");
				break;
			}
			double gyroRateTest = drive.rateTest();
			if(gyroRateTest > 1.0 || gyroRateTest <-1.0){
				drive.reconstructGyro();
				robot.outLog.throwLog(("[CHECK] Gyro Reset:"),i);
			}else{
				robot.outLog.throwLog("Gyro Test Passed");
				break;
			}
		}

	}

	void Autonomous() {
		drive.resetRot();
		lift.liftMotor.SetPosition(0);
		timeVal.Start();
		timeVal.Reset();
		robot.teleopEnd();
		robot.outLog.startTime();
		robot.outLog.setMode(OutLog::AUTO);
		std::string choice = *(std::string*) autoChooser.GetSelected();
		autoSeq.clear();
		std::cout<<"Auto mode:" <<choice<<std::endl;
		robot.outLog.throwLog(choice);

			if(choice=="Drive-to-Zone"){
				robot.outLog.throwLog("Auto Choice: drive to zone");
				DriveAction driveToZoneAction(drive, .9, SmartDashboard::GetNumber("DriveToZoneDist"));
				autoSeq.add_action(driveToZoneAction);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if(choice=="Push-to-Zone"){
					robot.outLog.throwLog("Auto Choice:Push to Zone");

					DriveAction pushToZoneAction(drive, 1.0, SmartDashboard::GetNumber("PushToZoneDist"));
					autoSeq.add_action(pushToZoneAction);
					autoSeq.init();
					while (IsAutonomous() and !IsDisabled()) {
						autoSeq.iter();
						loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
						Wait(loopTime); // wait for a motor update time
						SmartDashboard::PutNumber("Timer", timeVal.Get());
						if (timeVal.Get() >= .12){
							robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
						}
						timeVal.Reset();
					}
			}else if(choice == "One-Tote"){
				robot.outLog.throwLog("Auto Choice: Move set");
				LiftAction liftAboveCan(lift,SmartDashboard::GetNumber("TLiftHeightAboveCan"),true);
				autoSeq.add_action(liftAboveCan);
				DriveAction driveToControl(drive, 1.0,SmartDashboard::GetNumber("TControlCanDist"));
				autoSeq.add_action(driveToControl);
				TurnAction turnToControl(drive,90);
				autoSeq.add_action(turnToControl);
				DriveAction endOfAZone(drive, 1.0, SmartDashboard::GetNumber("TGoFarDist"));
				autoSeq.add_action(endOfAZone);
//				LiftAction lowerToBottom(lift, SmartDashboard::GetNumber("bottomHeight",100.0));
//				autoSeq.add_action(lowerToBottom);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if (choice=="Bin-Slap"){
				robot.outLog.throwLog("Auto Choice: Slaperino");

				// move all the totes.....#smackattack
//				LiftAction binSlap_LiftDown(lift, 100);
//				autoSeq.add_action(binSlap_LiftDown);
				LiftAction binSlap_LiftUpInit(lift, SmartDashboard::GetNumber("magicToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUpInit);
				PuncherAction punchOut(drive,DoubleSolenoid::kForward);
				autoSeq.add_action(punchOut);
				TurnActionNoReset turnOne(drive,15.0);
				autoSeq.add_action(turnOne);
//				PunchAction punchOne(drive,1.5,true);
//				autoSeq.add_action(punchOne);
				TurnActionNoReset turnTwo(drive,0.0);
				autoSeq.add_action(turnTwo);

				PuncherAction punchInOne(drive,DoubleSolenoid::kReverse);
				autoSeq.add_action(punchInOne);

				TurnSettleAction settleTwo(drive,.25);
				autoSeq.add_action(settleTwo);
//				WaitAction waitBumpOne(1);
//				autoSeq.add_action(waitBumpOne);

				LiftAction binSlap_LiftUp(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUp);
//				StrafeAction correctOne(drive,2.0,SmartDashboard::GetNumber("BinSlapStrafeDist"));
//				autoSeq.add_action(correctOne);
//				DriveAction binSlap_SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(binSlap_SmallDriveForward);

				//We gon knock that bin now
				DriveAction binSlap_MediumDriveForward(drive, 2, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward);
//				WaitAction waitOne(.5);
//				autoSeq.add_action(waitOne);
				PhotoDriveAction photoOne(drive,SmartDashboard::GetNumber("BinSlapMediumDriveForward")+2000,.45);
				autoSeq.add_action(photoOne);
				AlignAction align(drive);
				autoSeq.add_action(align);


				LiftAction binSlap_LiftDown2(lift, SmartDashboard::GetNumber("pickupToteHeight"));
				autoSeq.add_action(binSlap_LiftDown2);
				PuncherAction punchOutTwo(drive,DoubleSolenoid::kForward);
				autoSeq.add_action(punchOutTwo);
//				TurnActionNoReset turnThree(drive,15,.6);
//				autoSeq.add_action(turnThree);
//				PunchAction punchTwo(drive,1.5,true);
//				autoSeq.add_action(punchTwo);
				WaitAction WaitPunchTwo(1);
				autoSeq.add_action(WaitPunchTwo);
				PuncherAction punchIn(drive,DoubleSolenoid::kReverse);
				autoSeq.add_action(punchIn);
//				TurnActionNoReset turnFour(drive,0.0,.6);
//				autoSeq.add_action(turnFour);



				TurnSettleAction settleOne(drive,.25);
				autoSeq.add_action(settleOne);
//				WaitAction waitBumpTwo(1);
//				autoSeq.add_action(waitBumpTwo);


				LiftAction binSlap_LiftUp2(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUp2);
//				StrafeAction correctTwo(drive,2.0,SmartDashboard::GetNumber("BinSlapStrafeDist"));
//				autoSeq.add_action(correctTwo);
//				DriveAction binSlap_SmallDriveForward2(drive, 0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(binSlap_SmallDriveForward2);

				//We gon knock that bin now 2
				DriveAction binSlap_MediumDriveForward2(drive, 2, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward2);
//				WaitAction waitTwo(.5);
//				autoSeq.add_action(waitTwo);
				PhotoDriveAction photoTwo(drive,SmartDashboard::GetNumber("BinSlapMediumDriveForward")+2000,.45);
				autoSeq.add_action(photoTwo);
				AlignAction align2(drive);
				autoSeq.add_action(align2);
				LiftAction binSlap_LiftDown3(lift, SmartDashboard::GetNumber("pickupToteHeight"));
				autoSeq.add_action(binSlap_LiftDown3);
				LiftAction binSlap_LiftUp3(lift, SmartDashboard::GetNumber("magicToteHeight", true));
				autoSeq.add_action(binSlap_LiftUp3);
//				TurnAction binSlap_Turn(drive, 90);
//				autoSeq.add_action(binSlap_Turn);
				StrafeAction binSlap_DriveToAutozone(drive, 2, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone")/*/2*/);
				autoSeq.add_action(binSlap_DriveToAutozone);
//				DriveAction binSlap_DriveToAutozoneTwo(drive, .5, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone")/2);
//				autoSeq.add_action(binSlap_DriveToAutozoneTwo);
				WaitAction waitToSettle(.5);
				autoSeq.add_action(waitToSettle);
				LiftAction binSlap_LiftDown4(lift, 100);
				autoSeq.add_action(binSlap_LiftDown4);
				DriveAction binSlap_Back(drive, -2, SmartDashboard::GetNumber("BinSlapDriveBack"));
				autoSeq.add_action(binSlap_Back);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if (choice=="Two-Tote-Punch"){
				robot.outLog.throwLog("Auto Choice: Slaperino Dos Toterinos");

				// move all the totes.....#smackattack
//				LiftAction binSlap_LiftDown(lift, 100);
//				autoSeq.add_action(binSlap_LiftDown);
				LiftAction binSlap_LiftUpInit(lift, SmartDashboard::GetNumber("magicToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUpInit);
				PuncherAction punchOut(drive,DoubleSolenoid::kForward);
				autoSeq.add_action(punchOut);
				TurnActionNoReset turnOne(drive,15.0);
				autoSeq.add_action(turnOne);
//				PunchAction punchOne(drive,1.5,true);
//				autoSeq.add_action(punchOne);
				TurnActionNoReset turnTwo(drive,0.0);
				autoSeq.add_action(turnTwo);
				TurnSettleAction TTsettleTwo(drive,.25);
				autoSeq.add_action(TTsettleTwo);
//				WaitAction waitBumpOne(1);
//				autoSeq.add_action(waitBumpOne);
				PuncherAction punchInOne(drive,DoubleSolenoid::kReverse);
				autoSeq.add_action(punchInOne);
				LiftAction binSlap_LiftUp(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
				autoSeq.add_action(binSlap_LiftUp);
//				StrafeAction correctOne(drive,2.0,SmartDashboard::GetNumber("BinSlapStrafeDist"));
//				autoSeq.add_action(correctOne);
//				DriveAction binSlap_SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(binSlap_SmallDriveForward);

				//We gon knock that bin now
				DriveAction binSlap_MediumDriveForward(drive, 2, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
				autoSeq.add_action(binSlap_MediumDriveForward);
//				WaitAction waitOne(.5);
//				autoSeq.add_action(waitOne);
				PhotoDriveAction photoOne(drive,SmartDashboard::GetNumber("BinSlapMediumDriveForward")+2000,.6);
				autoSeq.add_action(photoOne);
				AlignAction align(drive);
				autoSeq.add_action(align);


				LiftAction binSlap_LiftDown2(lift, 300);
				autoSeq.add_action(binSlap_LiftDown2);
//				PuncherAction punchOutTwo(drive,DoubleSolenoid::kForward);
//				autoSeq.add_action(punchOutTwo);
//				TurnActionNoReset turnThree(drive,15);
//				autoSeq.add_action(turnThree);
//				PunchAction punchTwo(drive,1.5,true);
//				autoSeq.add_action(punchTwo);
//				TurnActionNoReset turnFour(drive,0);
//				autoSeq.add_action(turnFour);
//				WaitAction waitBumpTwo(1);
//				autoSeq.add_action(waitBumpTwo);
//				PuncherAction punchIn(drive,DoubleSolenoid::kReverse);
//				autoSeq.add_action(punchIn);
//				LiftAction binSlap_LiftUp2(lift, SmartDashboard::GetNumber("OverToteHeight"), true);
//				autoSeq.add_action(binSlap_LiftUp2);
//				StrafeAction correctTwo(drive,2.0,SmartDashboard::GetNumber("BinSlapStrafeDist"));
//				autoSeq.add_action(correctTwo);
//				DriveAction binSlap_SmallDriveForward2(drive, 0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(binSlap_SmallDriveForward2);

				//We gon knock that bin now 2
//				DriveAction binSlap_MediumDriveForward2(drive, 2, SmartDashboard::GetNumber("BinSlapMediumDriveForward"));
//				autoSeq.add_action(binSlap_MediumDriveForward2);
//				WaitAction waitTwo(.5);
//				autoSeq.add_action(waitTwo);
//				PhotoDriveAction photoTwo(drive,SmartDashboard::GetNumber("BinSlapMediumDriveForward")+2000,.6);
//				autoSeq.add_action(photoTwo);
//				AlignAction align2(drive);
//				autoSeq.add_action(align2);
//				LiftAction binSlap_LiftDown3(lift, SmartDashboard::GetNumber("pickupToteHeight"));
//				autoSeq.add_action(binSlap_LiftDown3);
				LiftAction binSlap_LiftUp3(lift, SmartDashboard::GetNumber("magicToteHeight", true));
				autoSeq.add_action(binSlap_LiftUp3);
				TurnAction binSlap_Turn(drive, 90);
				autoSeq.add_action(binSlap_Turn);
				DriveAction binSlap_DriveToAutozone(drive, 2, SmartDashboard::GetNumber("TTBinSlapDrive-To-Autozone")/2);
				autoSeq.add_action(binSlap_DriveToAutozone);
				DriveAction binSlap_DriveToAutozoneTwo(drive, .5, SmartDashboard::GetNumber("TTBinSlapDrive-To-Autozone")/2);
				autoSeq.add_action(binSlap_DriveToAutozoneTwo);
//				WaitAction waitToSettle(.5);
//				autoSeq.add_action(waitToSettle);
//				LiftAction binSlap_LiftDown4(lift, 100);
//				autoSeq.add_action(binSlap_LiftDown4);
//				DriveAction binSlap_Back(drive, -2, SmartDashboard::GetNumber("BinSlapDriveBack"));
//				autoSeq.add_action(binSlap_Back);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if (choice=="Shake-n'-Bake"){

				robot.outLog.throwLog("Auto Choice: shake it n' bake it");
				// move all the totes.....#smackaSNBck
//				LiftAction binSlap_LiftDown(lift, 100);
//				autoSeq.add_action(binSlap_LiftDown);
				LiftAction SNBLiftOne(lift,SmartDashboard::GetNumber("OverToteHeight"),true);
				autoSeq.add_action(SNBLiftOne);
				StrafeAction SNBStrafeDist(drive,-2.0, SmartDashboard::GetNumber("SNBStrafeDist",14000.0));
				autoSeq.add_action(SNBStrafeDist);
				WaitAction SNBWaitOne(.1);
				autoSeq.add_action(SNBWaitOne);
				DriveAction SNBForwardOne(drive,2.0, SmartDashboard::GetNumber("SNBForwardOne"));
				autoSeq.add_action(SNBForwardOne);
				WaitAction SNBWaitTwo(.1);
				autoSeq.add_action(SNBWaitTwo);
				StrafeAction SNBStrafeDistTwo(drive,2.0, SmartDashboard::GetNumber("SNBStrafeTwoDist",14000.0));
				autoSeq.add_action(SNBStrafeDistTwo);
				PhotoDriveAction SNBPhotoDist(drive, SmartDashboard::GetNumber("SNBPhotoDist",10000.0));
				autoSeq.add_action(SNBPhotoDist);
//				DriveAction SNBForwardTwo(drive,0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(SNBForwardTwo);
				AlignAction SNBAlign(drive);
				autoSeq.add_action(SNBAlign);
				LiftAction SNBLiftTwo(lift, SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(SNBLiftTwo);
				LiftAction SNBLiftThree(lift, SmartDashboard::GetNumber("magicToteHeight"),true);
				autoSeq.add_action(SNBLiftThree);

				LiftAction SNBLiftFour(lift,SmartDashboard::GetNumber("OverToteHeight"),true);
				autoSeq.add_action(SNBLiftFour);
				StrafeAction SNBStrafeDistThree(drive,-2.0, SmartDashboard::GetNumber("SNBStrafeDist",14000.0));
				autoSeq.add_action(SNBStrafeDistThree);
				WaitAction SNBWaitFour(.1);
				autoSeq.add_action(SNBWaitFour);
				DriveAction SNBForwardFour(drive,2.0, SmartDashboard::GetNumber("SNBForwardOne"));
				autoSeq.add_action(SNBForwardFour);
				WaitAction SNBWaitFive(.1);
				autoSeq.add_action(SNBWaitFive);
				StrafeAction SNBStrafeDistFive(drive,2.0, SmartDashboard::GetNumber("SNBStrafeTwoDist",14000.0));
				autoSeq.add_action(SNBStrafeDistFive);
				PhotoDriveAction SNBPhotoDistTwo(drive, SmartDashboard::GetNumber("SNBPhotoDist",10000.0));
				autoSeq.add_action(SNBPhotoDistTwo);
//				DriveAction SNBForwardFive(drive,0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(SNBForwardFive);
				AlignAction SNBAlignTwo(drive);
				autoSeq.add_action(SNBAlignTwo);
				LiftAction SNBLiftFive(lift, SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(SNBLiftFive);
				LiftAction SNBLiftSix(lift, SmartDashboard::GetNumber("magicToteHeight"),true);
				autoSeq.add_action(SNBLiftSix);


				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if (choice=="Three-Tote-Norm"){

				robot.outLog.throwLog("Auto Choice: 3tote Norm");
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
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if(choice=="Two-Tote"){

				//up, right, forward(2 ft), right, forward(2 ft), strafe right (2 feet), forward,align, down, (up bg), left, forward(long), down


				robot.outLog.throwLog("Auto Choice: two totez");
				LiftAction liftAboveCan(lift,SmartDashboard::GetNumber("OverToteHeight"),true);
				autoSeq.add_action(liftAboveCan);
//				DriveAction goBack(drive,-.9,-1000);
//				autoSeq.add_action(goBack);
				TurnAction turnToSlap(drive,SmartDashboard::GetNumber("TTHitAngle"));
				autoSeq.add_action(turnToSlap);
				DriveAction forwardOne(drive, .9, SmartDashboard::GetNumber("TTForwardOne"));
				autoSeq.add_action(forwardOne);
				TurnAction turnTwo(drive, 180-SmartDashboard::GetNumber("TTHitAngle"));
				autoSeq.add_action(turnTwo);
				DriveAction forwardTwo(drive,.9,SmartDashboard::GetNumber("TTForwardTwo"));
				autoSeq.add_action(forwardTwo);
				StrafeAction strafeOne(drive, 2.0, SmartDashboard::GetNumber("TTStrafe"));
				autoSeq.add_action(strafeOne);
				PhotoDriveAction photoOne(drive,SmartDashboard::GetNumber("TTMaxPhotoDist"));
				autoSeq.add_action(photoOne);
//				DriveAction goToTote(drive,1.0,SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(goToTote);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction liftDown(lift,SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(liftDown);
				LiftAction liftCarry(lift,SmartDashboard::GetNumber("magicToteHeight",0.0),true);
				autoSeq.add_action(liftCarry);
				TurnAction turnRight(drive,-90);
				autoSeq.add_action(turnRight);
				DriveAction endOfAZone(drive, .75, SmartDashboard::GetNumber("TTAutoZone"));
				autoSeq.add_action(endOfAZone);
//				LiftAction lowerToBottom(lift, 300.0);
//				autoSeq.add_action(lowerToBottom);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if (choice=="Push-Over"){
				robot.outLog.throwLog("Auto choice: Push the bins over");
				// push all the totes.....
				LiftAction binSlap_LiftUp(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(binSlap_LiftUp);
				TurnAction turn_one(drive, SmartDashboard::GetNumber("PushOver-TurnAngle", 35));
				autoSeq.add_action(turn_one);
				DriveAction binSlap_SmallDriveForward(drive, 0.9, SmartDashboard::GetNumber("PushOver-SmallDrive", 500));
				autoSeq.add_action(binSlap_SmallDriveForward);
				TurnAction turn_back_one(drive, -SmartDashboard::GetNumber("PushOver-TurnAngle", 35));
				autoSeq.add_action(turn_back_one);
				StrafeAction strafeTwo(drive, .9, SmartDashboard::GetNumber("PushOver-strafeBack", 300));
				autoSeq.add_action(strafeTwo);
				DriveAction driveForward(drive, 0.9, SmartDashboard::GetNumber("PushOver-longDriveForward", 10000));
				autoSeq.add_action(driveForward);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction binSlap_LiftDown2(lift, 100);
				autoSeq.add_action(binSlap_LiftDown2);
				LiftAction liftUp2(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(liftUp2);
				TurnAction turn_two(drive, SmartDashboard::GetNumber("PushOver-TurnAngle", 35));
				autoSeq.add_action(turn_two);
				DriveAction smallDriveForward(drive, 0.9, SmartDashboard::GetNumber("PushOver-SmallDrive", 500));
				autoSeq.add_action(smallDriveForward);
				TurnAction turn_back_two(drive, -SmartDashboard::GetNumber("PushOver-TurnAngle", 35));
				autoSeq.add_action(turn_back_two);
				StrafeAction strafethree(drive, .9, SmartDashboard::GetNumber("PushOver-strafeBack", 300));
				autoSeq.add_action(strafethree);
				DriveAction driveForwardtwo(drive, 0.9, SmartDashboard::GetNumber("PushOver-longDriveForward", 10000));
				autoSeq.add_action(driveForwardtwo);
				AlignAction alignTwo(drive);
				autoSeq.add_action(alignTwo);
				LiftAction binSlap_LiftDown3(lift, 100);
				autoSeq.add_action(binSlap_LiftDown3);
				LiftAction liftUp3(lift, SmartDashboard::GetNumber("BinSlapLiftUp"), true);
				autoSeq.add_action(liftUp3);
				TurnAction turn_90(drive, 90);
				autoSeq.add_action(turn_90);
				DriveAction driveToAuto(drive, 0.9, SmartDashboard::GetNumber("BinSlapDrive-To-Autozone", 2000));
				autoSeq.add_action(driveToAuto);
				LiftAction binSlap_LiftDown4(lift, 100);
				autoSeq.add_action(binSlap_LiftDown4);

				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else if(choice=="Two-Tote-Alt"){
				LiftAction TTALiftOne(lift,SmartDashboard::GetNumber("OverToteHeight"),true);
				autoSeq.add_action(TTALiftOne);
				StrafeAction TTAStrafeDist(drive,1, SmartDashboard::GetNumber("TTAStrafeDist",14000.0));
				autoSeq.add_action(TTAStrafeDist);
				WaitAction TTAWaitOne(.1);
				autoSeq.add_action(TTAWaitOne);
				DriveAction TTAForwardOne(drive,0.9, SmartDashboard::GetNumber("TTAForwardOne"));
				autoSeq.add_action(TTAForwardOne);
				WaitAction TTAWaitTwo(.1);
				autoSeq.add_action(TTAWaitTwo);
				StrafeAction TTAStrafeDistTwo(drive,-1, SmartDashboard::GetNumber("TTAStrafeTwoDist",14000.0));
				autoSeq.add_action(TTAStrafeDistTwo);
				PhotoDriveAction TTAPhotoDist(drive, SmartDashboard::GetNumber("TTAPhotoDist",10000.0));
				autoSeq.add_action(TTAPhotoDist);
//				DriveAction TTAForwardTwo(drive,0.9, SmartDashboard::GetNumber("Small"));
//				autoSeq.add_action(TTAForwardTwo);
				AlignAction TTAAlign(drive);
				autoSeq.add_action(TTAAlign);
				LiftAction TTALiftTwo(lift, SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(TTALiftTwo);
				LiftAction TTALiftThree(lift, SmartDashboard::GetNumber("magicToteHeight"),true);
				autoSeq.add_action(TTALiftThree);
				TurnAction TTATurn(drive,90);
				autoSeq.add_action(TTATurn);
				DriveAction TTAForwardThree(drive,0.75, SmartDashboard::GetNumber("TTAAutoZoneDist"));
				autoSeq.add_action(TTAForwardThree);
//				LiftAction TTALiftfour(lift, SmartDashboard::GetNumber("bottomHeight"));
//				autoSeq.add_action(TTALiftfour);

				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
					Wait(loopTime); // wait for a motor update time
					SmartDashboard::PutNumber("Timer", timeVal.Get());
					if (timeVal.Get() >= .12){
						robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
					}
					timeVal.Reset();
				}
			}else{
				std::cout<<"Bad auto type"<<std::endl;
			}
	}

	void OperatorControl() {
		timeVal.Start();
		timeVal.Reset();
		lift.liftMotor.SetExpiration(.25);
		drive.frontLeft.SetExpiration(.25);
		drive.backLeft.SetExpiration(.25);
		drive.frontRight.SetExpiration(.25);
		drive.backRight.SetExpiration(.25);
		robot.teleopInit();
		SmartDashboard::PutNumber("Timer", timeVal.Get());
		while (IsOperatorControl() && !IsDisabled()) {

			robot.teleop();
			loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
			Wait(loopTime); // wait for a motor update time
			SmartDashboard::PutNumber("Timer", timeVal.Get());
			if (timeVal.Get() >= .12){
				robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
			}
			timeVal.Reset();
		};
		robot.teleopEnd();
	};

	/**
	 * Runs during test mode
	 */
	void Test() {

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


