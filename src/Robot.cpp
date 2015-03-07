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
	SendableChooser autoChooser;
	Timer timeVal;
	double loopTime = 0.1;
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
		SmartDashboard::PutNumber("gyroPValue",0.05);
		SmartDashboard::PutNumber("gyroIValue",0.0);
		SmartDashboard::PutNumber("gyroDValue",0.0);
		SmartDashboard::PutNumber("gyroDead",.009);
		SmartDashboard::PutBoolean("disableGyro", false);

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
		SmartDashboard::PutNumber("DriveToZoneDist",10000.0);

		//Push to Zone
		SmartDashboard::PutNumber("PushToZoneDist",0.0);

		//Move Set
		SmartDashboard::PutNumber("TLiftHeightAboveCan",12000.0);
		SmartDashboard::PutNumber("TControlCanDist",7500.0);
		SmartDashboard::PutNumber("TGoFarDist",34500.0);
		SmartDashboard::PutNumber("TBackDist",4000.0);
		//Unused
		//SmartDashboard::PutNumber("SetStrafeCanControlDist",0.0);
		//SmartDashboard::PutNumber("SetBackupDist",0.0);
		//SmartDashboard::PutNumber("SetBackupToteDist",0.0);
		//SmartDashboard::PutNumber("LiftLowerWait",2.0);
		//SmartDashboard::PutNumber("LiftRaiseWait", 2.0);

		//Two Totes
		SmartDashboard::PutNumber("TTForwardOne", 3800.0);
		SmartDashboard::PutNumber("TTForwardTwo", 3875.0);
		SmartDashboard::PutNumber("TTGotoTote",1000.0);
		SmartDashboard::PutNumber("TTStrafe",14000.0);
		SmartDashboard::PutNumber("TTAutoZone",31500.0);
		SmartDashboard::PutNumber("TTHitAngle",90.0);
		SmartDashboard::PutNumber("TTMaxPhotoDist",10000.0);
		SmartDashboard::PutNumber("TTBackDist",2000.0);



//		SmartDashboard::PutNumber("BinSlapDrive", 2.0);
		SmartDashboard::PutNumber("BinSlapLiftDown", 100.0);
		SmartDashboard::PutNumber("BinSlapLiftUp", 12000.0);
//		SmartDashboard::PutNumber("BinSlap", 0.0);
		SmartDashboard::PutNumber("Small", 1000);
		SmartDashboard::PutNumber("BinSlapMediumDriveForward", 10000);
		SmartDashboard::PutNumber("BinSlapDrive-To-Autozone", 2000);

		//Two Tote Alt
		SmartDashboard::PutNumber("TTAStrafeDist",14000.0);
		SmartDashboard::PutNumber("TTAForwardOne",0.0);
		SmartDashboard::PutNumber("TTAAutoZoneDist",31500.0);
		SmartDashboard::PutNumber("TTABackDist",4000.0);
		SmartDashboard::PutNumber("TTAPhotoDist",10000.0);

		//ShakeN'Bake (Uses Slap Values otherwise)
		SmartDashboard::PutNumber("shakeNBakeStrafe", 0);

		// Push Over (uses Slap Values otherwise)
		SmartDashboard::PutNumber("PushOver-TurnAngle", 35);
		SmartDashboard::PutNumber("PushOver-SmallDrive", 500);
		SmartDashboard::PutNumber("PushOver-strafeBack", 300);
		SmartDashboard::PutNumber("PushOver-longDriveForward", 10000);

		//Add Auto Choices
		autoChooser.AddDefault("Drive to Zone", new std::string("Drive-to-Zone"));
		autoChooser.AddObject("One Tote", new std::string("One-Tote"));
		autoChooser.AddObject("Bin Slap", new std::string("Bin-Slap"));
		autoChooser.AddObject("Shake n' Bake", new std::string("Shake-n'-Bake"));
		autoChooser.AddObject("Two Tote", new std::string("Two-Tote"));
		autoChooser.AddObject("Push Bin or Tote", new std::string("Push-to-Zone"));
		autoChooser.AddObject("Three Tote No Bins",new std::string("Three-Tote-Norm"));
		autoChooser.AddObject("Push Over", new std::string("Push-Over"));
		autoChooser.AddObject("Two tote alt", new std::string("Two-tote-alt"));

		SmartDashboard::PutData("auto-choose", &autoChooser);

	}

	void Autonomous() {
		robot.teleopEnd();
		robot.outLog.startTime();
		robot.outLog.setMode(OutLog::AUTO);
		std::string choice = *(std::string*) autoChooser.GetSelected();
		autoSeq.clear();
		std::cout<<"Auto mode:" <<choice<<std::endl;
		robot.outLog.throwLog(choice);

			if(choice=="Drive-to-Zone"){
				robot.outLog.throwLog("Auto Choice: drive to zone");
//				StrafeAction driveToZoneAction(drive, .9, SmartDashboard::GetNumber("DriveToZoneDist"));
				DriveAction driveToZoneAction(drive, .9, SmartDashboard::GetNumber("DriveToZoneDist"));
				autoSeq.add_action(driveToZoneAction);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if(choice=="Push-to-Zone"){
					robot.outLog.throwLog("Auto Choice:Push to Zone");

					DriveAction pushToZoneAction(drive, 1.0, SmartDashboard::GetNumber("PushToZoneDist"));
					autoSeq.add_action(pushToZoneAction);
					autoSeq.init();
					while (IsAutonomous() and !IsDisabled()) {
						autoSeq.iter();
						Wait(0.05);
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
				LiftAction lowerToBottom(lift, SmartDashboard::GetNumber("bottomHeight",100.0));
				autoSeq.add_action(lowerToBottom);
				DriveAction goBack(drive,-.9,SmartDashboard::GetNumber("TBackDist"));
				autoSeq.add_action(goBack);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
				}
			}else if (choice=="Bin-Slap"){
				robot.outLog.throwLog("Auto Choice: Slaperino");

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

				robot.outLog.throwLog("Auto Choice: shake it n' bake it");
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
					Wait(0.05);
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
				DriveAction goToTote(drive,1.0,SmartDashboard::GetNumber("Small"));
				autoSeq.add_action(goToTote);
				AlignAction align(drive);
				autoSeq.add_action(align);
				LiftAction liftDown(lift,SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(liftDown);
				LiftAction liftCarry(lift,SmartDashboard::GetNumber("magicToteHeight",0.0),true);
				autoSeq.add_action(liftCarry);
				TurnAction turnRight(drive,-90);
				autoSeq.add_action(turnRight);
				DriveAction endOfAZone(drive, 1.0, SmartDashboard::GetNumber("TTAutoZone"));
				autoSeq.add_action(endOfAZone);
				LiftAction lowerToBottom(lift, 300.0);
				autoSeq.add_action(lowerToBottom);
				DriveAction goBack(drive,-.9,SmartDashboard::GetNumber("TTBackDist"));
				autoSeq.add_action(goBack);
				autoSeq.init();
				while (IsAutonomous() and !IsDisabled()) {
					autoSeq.iter();
					Wait(0.05);
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
					Wait(0.05);
				}
			}else if(choice=="two-tote-alt"){
				LiftAction TTALiftOne(lift,SmartDashboard::GetNumber("OverToteHeight"),true);
				autoSeq.add_action(TTALiftOne);
				StrafeAction TTAStrafeDist(drive,2.0, SmartDashboard::GetNumber("TTAStrafeDist",14000.0));
				autoSeq.add_action(TTAStrafeDist);
				DriveAction TTAForwardOne(drive,0.9, SmartDashboard::GetNumber("TTAForwardOne"));
				autoSeq.add_action(TTAForwardOne);
				StrafeAction TTAStrafeDistTwo(drive,2.0, SmartDashboard::GetNumber("TTAStrafeDist",-14000.0));
				autoSeq.add_action(TTAStrafeDistTwo);
				PhotoDriveAction TTAPhotoDist(drive, SmartDashboard::GetNumber("TTAPhotoDist",10000.0));
				autoSeq.add_action(TTAPhotoDist);
				DriveAction TTAForwardTwo(drive,0.9, SmartDashboard::GetNumber("TTAForwardTwo"));
				autoSeq.add_action(TTAForwardTwo);
				AlignAction TTAAlign(drive);
				autoSeq.add_action(TTAAlign);
				LiftAction TTALiftTwo(lift, SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(TTALiftTwo);
				LiftAction TTALiftThree(lift, SmartDashboard::GetNumber("magicToteHeight"),true);
				autoSeq.add_action(TTALiftThree);
				TurnAction TTATurn(drive,90);
				autoSeq.add_action(TTATurn);
				DriveAction TTAForwardThree(drive,0.9, SmartDashboard::GetNumber("TTAForwardThree"));
				autoSeq.add_action(TTAForwardThree);
				LiftAction TTALiftfour(lift, SmartDashboard::GetNumber("bottomHeight"));
				autoSeq.add_action(TTALiftfour);
				DriveAction TTABackward(drive,0.9, SmartDashboard::GetNumber("TTABackward"));
				autoSeq.add_action(TTABackward);


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

////			wd.Feed();
			robot.teleop();
			loopTime = timeVal.Get()<.1?0.1-timeVal.Get():0.0;
			Wait(loopTime); // wait for a motor update time
			SmartDashboard::PutNumber("Timer", timeVal.Get());
			if (timeVal.Get() >= .12){
				robot.outLog.throwLog("[PROBLEM] Loop Time High! Timer at: ",timeVal.Get());
			}
			timeVal.Reset();
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


