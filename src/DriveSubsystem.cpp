#include "DriveSubsystem.h"
#include <iostream>


std::string DriveSubsystem::name(void){
	return "Drive";
}

void DriveSubsystem::robotInit(void){
    serial_port = new SerialPort(57600,SerialPort::kMXP);
    uint8_t update_rate_hz = 50;
	#if defined(ENABLE_AHRS)
    imu = new AHRS(serial_port,update_rate_hz);
	#elif defined(ENABLE_IMU_ADVANCED)
    imu = new IMUAdvanced(serial_port,update_rate_hz);
	#else // ENABLE_IMU
    imu = new IMU(serial_port,update_rate_hz);
	#endif

	robot.outLog.throwLog("DriveSubsystem: RobotInit Success");
}
void DriveSubsystem::teleopInit(void){
	robot.outLog.throwLog("DriveSubsystem: TeleopInit Success");

    bool is_calibrating = imu->IsCalibrating();
    if ( !is_calibrating ) {
//        Wait( 0.3 );
        imu->ZeroYaw();
    }

	frontLeft.SetSafetyEnabled(true);
	backLeft.SetSafetyEnabled(true);
	frontRight.SetSafetyEnabled(true);
	backRight.SetSafetyEnabled(true);
	frontLeft.Set(0.0);
	backLeft.Set(0.0);
	frontRight.Set(0.0);
	backRight.Set(0.0);
	robot.joystick.register_axis("drive_x", 1, 0);
	robot.joystick.register_axis("drive_rotation", 1, 2);
	robot.joystick.register_axis("drive_y", 1, 1);
	robot.joystick.register_button("shoulderSpeed", 1, 5);
	robot.joystick.register_button("lift_align",1,7);
	robot.joystick.register_button("top_lift_align",1,8);
	robot.joystick.register_button("rotate",1,3);
	robot.joystick.register_button("ultra",1,4); //TODO Find Permanent Button for Ultra PID
	robot.joystick.register_button("centerDrive",1,1);
	robot.joystick.register_button("ultraSet",1,9,JoystickCache::RISING);
	robot.joystick.register_button("punch",1,10,JoystickCache::RISING);
	robot.joystick.register_button("alignEverything",1,6);
	robot.joystick.register_button("polarAlign",1,2);
	robot.joystick.joystick1.GetPOV();



	//UltraSonic Values
//	feederStationChooser.AddDefault("Right Feeder Station", new std::string("right"));
//	feederStationChooser.AddObject("Left Feeder Station", new std::string("left"));
//	SmartDashboard::PutData("feederStation", &feederStationChooser);

	timer.Start();
	gyroTimer.Start();
	gyroTimer.Reset();
	leftUltraTimer.Start();
	leftUltraTimer.Reset();
	rightUltraTimer.Start();
	rightUltraTimer.Reset();
	feederAlignTimer.Start();
	feederAlignTimer.Reset();
//	gyro.Reset();
	imu->ResetDisplacement();
}

void DriveSubsystem::teleop(void){

//    SmartDashboard::PutBoolean( "IMU_Connected", imu->IsConnected());
//    SmartDashboard::PutNumber("IMU_Yaw", imu->GetYaw());
//    SmartDashboard::PutNumber("IMU_Pitch", imu->GetPitch());
//    SmartDashboard::PutNumber("IMU_Roll", imu->GetRoll());
//    SmartDashboard::PutNumber("IMU_CompassHeading", imu->GetCompassHeading());
//    SmartDashboard::PutNumber("IMU_Update_Count", imu->GetUpdateCount());
//    SmartDashboard::PutNumber("IMU_Byte_Count", imu->GetByteCount());

//	#if defined (ENABLE_IMU_ADVANCED) || defined(ENABLE_AHRS)
//    	SmartDashboard::PutNumber("IMU_Accel_X", imu->GetWorldLinearAccelX());
//    	SmartDashboard::PutNumber("IMU_Accel_Y", imu->GetWorldLinearAccelY());
//    	SmartDashboard::PutBoolean("IMU_IsMoving", imu->IsMoving());
//    	SmartDashboard::PutNumber("IMU_Temp_C", imu->GetTempC());
//    	SmartDashboard::PutBoolean("IMU_IsCalibrating", imu->IsCalibrating());
//	#if defined (ENABLE_AHRS)
//    	SmartDashboard::PutNumber("Velocity_X",             imu->GetVelocityX() );
//    	SmartDashboard::PutNumber("Velocity_Y",             imu->GetVelocityY() );
//    	SmartDashboard::PutNumber("Displacement_X",     imu->GetDisplacementX() );
//    	SmartDashboard::PutNumber("Displacement_Y",     imu->GetDisplacementY() );
//	#endif
//	#endif
	leftUltraVal = getLeftUltra();
	if (leftUltraVal > 150 || leftUltraVal<23){
		leftUltraVal = leftUltraPID.lastValue;
	}
	rightUltraVal = getRightUltra();
	if (rightUltraVal > 150 || rightUltraVal<23){
		rightUltraVal = rightUltraPID.lastValue;
	}
	feederUltraVal = getFeederAlignUltra();
	if (feederUltraVal > 150 || feederUltraVal<23){
		feederUltraVal = feederAlignPID.lastValue;
	}



//	robot.outLog.throwLog("PID Sets");
if (robot.joystick.button("ultraSet")){
	double FrontLeftUltra = leftUltraVal;
	robot.outLog.throwLog("Left Ultra New Value: ", FrontLeftUltra);
	double FrontRightUltra = rightUltraVal;
	robot.outLog.throwLog("Right Ultra New Value: ", FrontRightUltra);
	double FeederUltra = ((feederUltraVal/cos((SmartDashboard::GetNumber("feederWallAngle")-90)*(PI/180))));
	robot.outLog.throwLog("Feeder Ultra New Value: ", FeederUltra);
	SmartDashboard::PutNumber("leftUltraSetPoint", FrontLeftUltra);
	SmartDashboard::PutNumber("rightUltraSetPoint", FrontRightUltra);
	SmartDashboard::PutNumber("feederXSet", FeederUltra);
}




	if(!simple){
		frontLeft.SetPID(SmartDashboard::GetNumber("FrontLeftPValue"),
			SmartDashboard::GetNumber("FrontLeftIValue"),
			SmartDashboard::GetNumber("FrontLeftDValue"),
			SmartDashboard::GetNumber("FrontLeftFValue"));
		backLeft.SetPID(SmartDashboard::GetNumber("BackLeftPValue"),
			SmartDashboard::GetNumber("BackLeftIValue"),
			SmartDashboard::GetNumber("BackLeftDValue"),
			SmartDashboard::GetNumber("BackLeftFValue"));
		frontRight.SetPID(SmartDashboard::GetNumber("FrontRightPValue"),
			SmartDashboard::GetNumber("FrontRightIValue"),
			SmartDashboard::GetNumber("FrontRightDValue"),
			SmartDashboard::GetNumber("FrontRightFValue"));
		backRight.SetPID(SmartDashboard::GetNumber("BackRightPValue"),
			SmartDashboard::GetNumber("BackRightIValue"),
			SmartDashboard::GetNumber("BackRightDValue"),
			SmartDashboard::GetNumber("BackRightFValue"));
	}
	if (robot.joystick.button("punch")){
		if (binPunch.Get() == DoubleSolenoid::kReverse){
			binPunch.Set(DoubleSolenoid::kForward);
		}else{
			binPunch.Set(DoubleSolenoid::kReverse);
		}
	}
//SmartDashboard::PutNumber("Punch Set",binPunch.Get());
//	robot.outLog.throwLog("PID Sets Done");
	//robot.outLog.throwLog("start and smt dshbrd");
	double gyroRate = imu->GetYaw();
/*	if (robot.joystick.button("reset")){

		Pulse();
		resetDistance();
	}
*/
		SmartDashboard::PutNumber("Left Ultra Dist", leftUltraVal);
		SmartDashboard::PutNumber("Right Ultra Dist", rightUltraVal);
		SmartDashboard::PutNumber("Feeder Ultra Dist", feederUltraVal);


	simple = SmartDashboard::GetBoolean("SimpleDrive");
//	gyro.SetDeadband(SmartDashboard::GetNumber("gyroDead"));
//	if (gyroRate>-2 && gyroRate<2){
//		gyroRate = 0.0;
//	}
//	robot.outLog.throwLog("gyro and simple set");
	POV = robot.joystick.joystick1.GetPOV();
//	SmartDashboard::PutNumber("POV", POV);
	shoulderSpeed = robot.joystick.button("shoulderSpeed");
	SmartDashboard::PutNumber("drive x", drive_x);
	SmartDashboard::PutNumber("drive y", drive_y);
	SmartDashboard::PutNumber("drive rot", drive_rotation);
//	SmartDashboard::PutBoolean("Encoders-Status", isBroken);
	SmartDashboard::PutNumber("Front Left Set", frontLeft.Get());
	SmartDashboard::PutNumber("Front Right Set", frontRight.Get());
	SmartDashboard::PutNumber("Back Left Set", backLeft.Get());
	SmartDashboard::PutNumber("Back Right Set", backRight.Get());
//	SmartDashboard::PutNumber("Gyro Rate", gyro.GetRate());
	SmartDashboard::PutNumber("Gyro Angle", imu->GetYaw());
//	SmartDashboard::PutNumber("Shoulder Speed", shoulderSpeed);
	SmartDashboard::PutBoolean("Centering Photo", centerPhoto.Get());

	SmartDashboard::PutNumber("AutoUltra", getAutoUltra());

	SmartDashboard::PutNumber("FeederX", (getFeederAlignUltra()/cos((SmartDashboard::GetNumber("feederWallAngle")-90)*(PI/180))));


	SmartDashboard::PutNumber("FLE", frontLeft.GetEncPosition());
	SmartDashboard::PutNumber("FRE", frontRight.GetEncPosition());
	SmartDashboard::PutNumber("BLE", backLeft.GetEncPosition());
	SmartDashboard::PutNumber("BRE", backRight.GetEncPosition());

//	SmartDashboard::PutNumber("FLV", frontLeft.GetEncVel());
//	SmartDashboard::PutNumber("FRV", frontRight.GetEncVel());
//	SmartDashboard::PutNumber("BLV", backLeft.GetEncVel());
//	SmartDashboard::PutNumber("BRV", backRight.GetEncVel());

//	SmartDashboard::PutNumber("Accel X", accel.GetX());
//	SmartDashboard::PutNumber("Accel Y", accel.GetY());
	SmartDashboard::PutNumber("Jumper Voltage", getJumper());


//	SmartDashboard::PutBoolean("Tote align", robot.joystick.button("lift_align"));
//	SmartDashboard::PutBoolean("Top Tote align", robot.joystick.button("top_lift_align"));

// Setting gyro and Ultrasonic PID's
	gyroPID.P = (SmartDashboard::GetNumber("gyroPValue"));
	gyroPID.I = (SmartDashboard::GetNumber("gyroIValue"));
	gyroPID.D = (SmartDashboard::GetNumber("gyroDValue"));
	leftUltraPID.P = (SmartDashboard::GetNumber("leftUltraPValue"));
	leftUltraPID.I = (SmartDashboard::GetNumber("leftUltraIValue"));
	leftUltraPID.D =(SmartDashboard::GetNumber("leftUltraDValue"));
	leftUltraPID.setPoint = (SmartDashboard::GetNumber("leftUltraSetPoint"));
	rightUltraPID.P = (SmartDashboard::GetNumber("rightUltraPValue"));
	rightUltraPID.I = (SmartDashboard::GetNumber("rightUltraIValue"));
	rightUltraPID.D =(SmartDashboard::GetNumber("rightUltraDValue"));
	rightUltraPID.setPoint = (SmartDashboard::GetNumber("rightUltraSetPoint"));
	feederAlignPID.P = (SmartDashboard::GetNumber("feederAlignUltraPValue"));
	feederAlignPID.I = (SmartDashboard::GetNumber("feederAlignUltraIValue"));
	feederAlignPID.D =(SmartDashboard::GetNumber("feederAlignUltraDValue"));
	feederAlignPID.setPoint = (SmartDashboard::GetNumber("feederAlignUltraSetPoint"));

	if (POV == -1){
		dPad = false;
//Simple Dead-banding
		drive_x = robot.joystick.axis("drive_x");
		if (drive_x < 0.05 && drive_x > -.05){
			drive_x = 0;
		}
		drive_y = robot.joystick.axis("drive_y");
		if (drive_y < .05 && drive_y > -.05){
			drive_y = 0;
		}
		drive_y *= -1;
	}else{
		dPad = true;
		switch (POV){
		case 0:
			drive_x = 0;
			drive_y = 1;
			dPad = true;
			break;
		case 45:
			drive_x = 1;
			drive_y = 1;
			dPad = false;
			break;
		case 90:
			drive_x = 1.0;
			drive_y = 0;
			dPad = true;
			break;
		case 135:
			drive_x = 1;
			drive_y = -1;
			dPad = false;
			break;
		case 180:
			drive_x = 0.0;
			drive_y = -1;
			dPad = true;
			break;
		case 225:
			drive_x = -1;
			drive_y = -1;
			dPad = false;
			break;
		case 270:
			drive_x = -1.0;
			drive_y = 0;
			dPad = true;
			break;
		case 315:
			drive_x = -1;
			drive_y = 1;
			dPad = false;
			break;
		default:
			drive_x = 0;
			drive_y = 0;
			dPad = false;
			break;
		}
	}
	drive_rotation = robot.joystick.axis("drive_rotation");
	if (drive_rotation < .05 && drive_rotation > -.05){
		drive_rotation = 0;
	}



//TODO Tune
	//Ultrasonic PID
	double error = (leftUltraVal)-(rightUltraVal);
	double xError = feederUltraVal-feederAlignPID.setPoint;
////////////////////////////////////////////////////////////////////////////////////////////////
	if(false){
		leftUltraPID.actualPosition = (((getLeftUltra()+getRightUltra())/2)>23.0?((getLeftUltra()+getRightUltra())/2):leftUltraPID.lastValue);
		if(((leftUltraPID.actualPosition >leftUltraPID.setPoint+5 || leftUltraPID.actualPosition < leftUltraPID.setPoint-5) || leftUltraDistCorrect) && drive_y ==0) {
					leftUltraPID.actualPosition = (((getLeftUltra()+getRightUltra())/2)>23.0?((getLeftUltra()+getRightUltra())/2):leftUltraPID.lastValue);
					if(leftUltraPID.actualPosition>70){
						drive_y = 1.0;
					}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+20){
						drive_y = .6;
					}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+10){
						drive_y = .45;
					}else if (leftUltraPID.actualPosition >leftUltraPID.setPoint+5){
						drive_y = .25;
					}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+.5){
						drive_y = .2;
					}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint-.5){
						drive_y = 0;
					}else if (leftUltraPID.actualPosition > leftUltraPID.setPoint-5){
						drive_y = -.2;
					}else{
						drive_y = -.3;
					}

					}else if(error<-2 || error>2) {
						if(error>5.0){
							drive_rotation = .6;
						}else if (error>3){
							drive_rotation = .5;
						}else if (error>2){
							drive_rotation = .4;
						}else if (error > .5){
							drive_rotation = .3;
						}else if (error>-.5){
							drive_rotation = 0;
						}else if (error>-2){
							drive_rotation = -.3;
						}else if (error> -3){
							drive_rotation = -.4;
						}else if (error > -5){
							drive_rotation = -.5;
						}else if (error <=-5){
							drive_rotation = -.6;
						}else{
							robot.outLog.throwLog("No value for turning to align to feeder station!");
							drive_rotation = 0;
						}

					}else if(((xError>3 || xError<-3) || feederAlignUltraDistCorrect) && drive_x ==0) {


					if(xError>8){
						//more than 8
						drive_x = 1.2;
					}else if (xError>5){
						//8-5
						drive_x = 1;
					}else if (xError>3){
						//3-5
						drive_x = .9;
					}else if (xError > .5){
						//.5-3
						drive_x = .8;
					}else if (xError>-.5){
						//-.5-.5
						drive_x = 0;
					}else if (xError>-3){
						//-.5--3
						drive_x = -.8;
					}else if (xError> -5){
						//-3--5
						drive_x = -.9;
					}else if (xError >-8){
						//-5--8
						drive_x = -1;
					}else if (xError <=-8){
						//less than 8
						drive_x = -1.2;
					}else{
						drive_x= 0;
					}
	}else if(((leftUltraPID.actualPosition >leftUltraPID.setPoint+1 || leftUltraPID.actualPosition < leftUltraPID.setPoint-1) || leftUltraDistCorrect) && drive_y ==0) {
						leftUltraPID.actualPosition = (((getLeftUltra()+getRightUltra())/2)>23.0?((getLeftUltra()+getRightUltra())/2):leftUltraPID.lastValue);
						if(leftUltraPID.actualPosition>70){
							drive_y = 1.0;
						}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+20){
							drive_y = .6;
						}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+10){
							drive_y = .45;
						}else if (leftUltraPID.actualPosition >leftUltraPID.setPoint+5){
							drive_y = .25;
						}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+.5){
							drive_y = .2;
						}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint-.5){
							drive_y = 0;
						}else if (leftUltraPID.actualPosition > leftUltraPID.setPoint-5){
							drive_y = -.2;
						}else{
							drive_y = -.3;
						}
						leftUltraPID.lastValue = (((getLeftUltra()+getRightUltra())/2)>23.0?((getLeftUltra()+getRightUltra())/2):leftUltraPID.lastValue);
						}
					else if(((xError>.5 || xError<-.5) || feederAlignUltraDistCorrect) && drive_x ==0) {

			//			SmartDashboard::PutNumber("Feeder Ultra Error", error);
						if(xError>8){
							//more than 8
							drive_x = 1.2;
						}else if (xError>5){
							//8-5
							drive_x = 1;
						}else if (xError>3){
							//3-5
							drive_x = .9;
						}else if (xError > .5){
							//.5-3
							drive_x = .8;
						}else if (xError>-.5){
							//-.5-.5
							drive_x = 0;
						}else if (xError>-3){
							//-.5--3
							drive_x = -.8;
						}else if (xError> -5){
							//-3--5
							drive_x = -.9;
						}else if (xError >-8){
							//-5--8
							drive_x = -1;
						}else if (xError <=-8){
							//less than 8
							drive_x = -1.2;
						}else{
							drive_x= 0;
						}
					}
			else if(error<-.5 || error>.5) {
							if(error>5.0){
								drive_rotation = .6;
							}else if (error>3){
								drive_rotation = .5;
							}else if (error>2){
								drive_rotation = .4;
							}else if (error > .5){
								drive_rotation = .3;
							}else if (error>-.5){
								drive_rotation = 0;
							}else if (error>-2){
								drive_rotation = -.3;
							}else if (error> -3){
								drive_rotation = -.4;
							}else if (error > -5){
								drive_rotation = -.5;
							}else if (error <=-5){
								drive_rotation = -.6;
							}else{
								robot.outLog.throwLog("No value for turning to align to feeder station!");
								drive_rotation = 0;
							}
							leftUltraPID.lastValue = (getLeftUltra()>23.0?getLeftUltra():leftUltraPID.lastValue);
							rightUltraPID.lastValue = (getRightUltra()>23.0?getRightUltra():rightUltraPID.lastValue);
						}

		feederAlignPID.lastValue = getFeederAlignUltra();
	}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////Will Be One Button Polar Align//////////////////////////////////////////////////////////
		if(robot.joystick.button("alignEverything")){
//						polar = true;
					drive_x = 0;
					drive_y = 0;
						if(error<-.5 || error>.5) {
							polar = false;
							if(error>5.0){
								drive_rotation = .7;
							}else if (error>3){
								drive_rotation = .6;
							}else if (error>2){
								drive_rotation = .5;
							}else if (error > .25){
								drive_rotation = .4;
							}else if (error>-.25){
								drive_rotation = 0;
							}else if (error>-2){
								drive_rotation = -.4;
							}else if (error> -3){
								drive_rotation = -.5;
							}else if (error > -5){
								drive_rotation = -.6;
							}else if (error <=-5){
								drive_rotation = -.7;
							}else{
								robot.outLog.throwLog("No value for turning to align to feeder station!");
								drive_rotation = 0;
							}

						}else{
							if (polarState == 0){
								polar = true;
								double yDifference = (((leftUltraVal+rightUltraVal)/2) - leftUltraPID.setPoint);
								double xDifference = (((feederUltraVal/cos((SmartDashboard::GetNumber("feederWallAngle")-90)*(PI/180))))-SmartDashboard::GetNumber("feederXSet"));
								xDifference *= (SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1);
								yDifference = (yDifference<.25 && yDifference >-.25)?0.01:yDifference;
								xDifference = (xDifference<.25 && xDifference >-.25)?0.0:xDifference;

								double distanceDifference = (pow(pow(yDifference,2)+pow(xDifference,2),(1.0/2.0)));

								if (xDifference == 0 && yDifference == 0){
									polarAngle = 0;
									distanceDifference = 0;
								}else if (xDifference == 0 && yDifference > 0){
									polarAngle = 0;
								}else if (xDifference == 0 && yDifference <=0){
									polarAngle = PI;
								}else if (yDifference == 0 && xDifference > 0){
									polarAngle = (3*PI/2);
								}else if (yDifference == 0 && xDifference <= 0){
									polarAngle = (PI/2);
								}else{
									if (yDifference > 0 ){
										polarAngle = atan((xDifference/yDifference));
									}else{
										polarAngle = ((PI/2)+((PI/2)-atan(-xDifference/yDifference)));
									}
								}

								if (polarAngle<0.0){
									polarAngle+=(2*PI);
								}


								SmartDashboard::PutNumber("Polar Y Diff", yDifference);
								SmartDashboard::PutNumber("Polar X Diff", xDifference);
								SmartDashboard::PutNumber("Polar distanceDiff", distanceDifference);


								if(distanceDifference>8){
									//more than 8
									polarMag = .2;
								}else if (distanceDifference>5){
									//8-5
									polarMag = .2;
								}else if (distanceDifference>3){
									//3-5
									polarMag = .2;
								}else if (distanceDifference > .5){
									//.5-3
									polarMag = .2;
								}else{
									polarMag= 0;
								}
				//				if (((polarAngle * (180/PI))>65 && (polarAngle * (180/PI))<115)||(((polarAngle * (180/PI))>255 && (polarAngle * (180/PI))<=270) || ((polarAngle * (180/PI))<-65 && (polarAngle * (180/PI))>=-90))){
				//					polarMag+=.7;
				//				}
								if (polarAngle == (PI/2) || polarAngle == (3*PI/2)){
									polarMag *= 4;
								}else if (polarAngle == 0 || polarAngle == (PI)){
									polarMag *= .9;
								}
								else if (polarAngle > 0 && polarAngle < PI){
									polarMag*=((90-abs(90-(polarAngle*(180/PI))))*.055)+1;
								}else {
									polarMag*=((90-abs(270-(polarAngle*(180/PI))))*.055)+1;
								}


								if (abs(xDifference) < .5 && abs(yDifference) < .5){
									polarState = 1;
									polarMag = 0;
								}

								SmartDashboard::PutNumber("Polar Mag", polarMag);
								SmartDashboard::PutNumber("Polar Angle", polarAngle * (180/PI));

					//			leftUltraPID.lastValue = (getLeftUltra()>23.0?getLeftUltra():leftUltraPID.lastValue);
					//			rightUltraPID.lastValue = (getRightUltra()>23.0?getRightUltra():rightUltraPID.lastValue);
								leftUltraPID.lastValue = (leftUltraVal);
								rightUltraPID.lastValue = (rightUltraVal);
								feederAlignPID.lastValue = feederUltraVal;



						}
						}
			feederAlignPID.lastValue = getFeederAlignUltra();
					//TODO Tune Polar Drive
			}else if((robot.joystick.button("polarAlign")) && (drive_x == 0 && drive_y == 0)) {
			if (polarState == 0){
				polar = true;
				double yDifference = (((leftUltraVal+rightUltraVal)/2) - leftUltraPID.setPoint);
				double xDifference = (((feederUltraVal/cos((SmartDashboard::GetNumber("feederWallAngle")-90)*(PI/180))))-SmartDashboard::GetNumber("feederXSet"));
				xDifference *= (SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1);
				yDifference = (yDifference<.25 && yDifference >-.25)?0.01:yDifference;
				xDifference = (xDifference<.25 && xDifference >-.25)?0.0:xDifference;

				double distanceDifference = (pow(pow(yDifference,2)+pow(xDifference,2),(1.0/2.0)));

				if (xDifference == 0 && yDifference == 0){
					polarAngle = 0;
					distanceDifference = 0;
				}else if (xDifference == 0 && yDifference > 0){
					polarAngle = 0;
				}else if (xDifference == 0 && yDifference <=0){
					polarAngle = PI;
				}else if (yDifference == 0 && xDifference > 0){
					polarAngle = (3*PI/2);
				}else if (yDifference == 0 && xDifference <= 0){
					polarAngle = (PI/2);
				}else{
					if (yDifference > 0 ){
						polarAngle = atan((xDifference/yDifference));
					}else{
						polarAngle = ((PI/2)+((PI/2)-atan(-xDifference/yDifference)));
					}
				}

				if (polarAngle<0.0){
					polarAngle+=(2*PI);
				}


				SmartDashboard::PutNumber("Polar Y Diff", yDifference);
				SmartDashboard::PutNumber("Polar X Diff", xDifference);
				SmartDashboard::PutNumber("Polar distanceDiff", distanceDifference);


				if(distanceDifference>8){
					//more than 8
					polarMag = .2;
				}else if (distanceDifference>5){
					//8-5
					polarMag = .2;
				}else if (distanceDifference>3){
					//3-5
					polarMag = .2;
				}else if (distanceDifference > .5){
					//.5-3
					polarMag = .2;
				}else{
					polarMag= 0;
				}
//				if (((polarAngle * (180/PI))>65 && (polarAngle * (180/PI))<115)||(((polarAngle * (180/PI))>255 && (polarAngle * (180/PI))<=270) || ((polarAngle * (180/PI))<-65 && (polarAngle * (180/PI))>=-90))){
//					polarMag+=.7;
//				}
				if (polarAngle == (PI/2) || polarAngle == (3*PI/2)){
					polarMag *= 4;
				}else if (polarAngle == 0 || polarAngle == (PI)){
					polarMag *= .9;
				}
				else if (polarAngle > 0 && polarAngle < PI){
					polarMag*=((90-abs(90-(polarAngle*(180/PI))))*.055)+1;
				}else {
					polarMag*=((90-abs(270-(polarAngle*(180/PI))))*.055)+1;
				}


				if (abs(xDifference) < .5 && abs(yDifference) < .5){
					polarState = 1;
					polarMag = 0;
				}

				SmartDashboard::PutNumber("Polar Mag", polarMag);
				SmartDashboard::PutNumber("Polar Angle", polarAngle * (180/PI));

	//			leftUltraPID.lastValue = (getLeftUltra()>23.0?getLeftUltra():leftUltraPID.lastValue);
	//			rightUltraPID.lastValue = (getRightUltra()>23.0?getRightUltra():rightUltraPID.lastValue);
				leftUltraPID.lastValue = (leftUltraVal);
				rightUltraPID.lastValue = (rightUltraVal);
				feederAlignPID.lastValue = feederUltraVal;
			}else{
				polar = false;
				leftUltraPID.actualPosition = ((leftUltraVal+rightUltraVal)/2);
				if(leftUltraPID.actualPosition>70){
					drive_y = 1.0;
				}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+20){
					drive_y = .6;
				}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+10){
					drive_y = .45;
				}else if (leftUltraPID.actualPosition >leftUltraPID.setPoint+5){
					drive_y = .25;
				}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+.5){
					drive_y = .15;
				}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint-.5){
					drive_y = 0;
				}else if (leftUltraPID.actualPosition > leftUltraPID.setPoint-5){
					drive_y = -.15;
				}else{
					drive_y = -.3;
				}
				leftUltraPID.lastValue = ((leftUltraVal+rightUltraVal)/2);
			}
			}else{
				polarState = 0;
				polarLast = false;
				polar = false;
			}
	//////////////////////////////////////////////////////////////////////////////////////////////////
		if((robot.joystick.button("ultra") || leftUltraDistCorrect) && drive_y ==0) {
			leftUltraPID.actualPosition = (((leftUltraVal+rightUltraVal)/2));
			if(leftUltraPID.actualPosition>70){
				drive_y = 1.0;
			}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+20){
				drive_y = .6;
			}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+10){
				drive_y = .45;
			}else if (leftUltraPID.actualPosition >leftUltraPID.setPoint+5){
				drive_y = .25;
			}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint+.5){
				drive_y = .2;
			}else if (leftUltraPID.actualPosition>leftUltraPID.setPoint-.5){
				drive_y = 0;
			}else if (leftUltraPID.actualPosition > leftUltraPID.setPoint-5){
				drive_y = -.2;
			}else{
				drive_y = -.3;
			}
			leftUltraPID.lastValue = (((leftUltraVal+rightUltraVal)/2));
		}
		//TODO Tune
		//Ultrasonic Rotation
			if((robot.joystick.button("rotate"))/* && drive_rotation == 0*/) {
				if(error>5.0){
					drive_rotation = .5;
				}else if (error>2){
					drive_rotation = .45;
				}else if (error>1){
					drive_rotation = .4;
				}else if (error > .25){
					drive_rotation = .3;
				}else if (error>-.25){
					drive_rotation = 0.0;
				}else if (error>-1){
					drive_rotation = -.3;
				}else if (error> -2){
					drive_rotation = -.4;
				}else if (error > -5){
					drive_rotation = -.45;
				}else if (error <=-5){
					drive_rotation = -.5;
				}else{
					robot.outLog.throwLog("No value for turning to align to feeder station!");
					drive_rotation = 0;
				}

				leftUltraPID.lastValue = (leftUltraVal);
				rightUltraPID.lastValue = (rightUltraVal);


			}
			//TODO Tune Side To Side
		if((robot.joystick.button("centerDrive") || feederAlignUltraDistCorrect) && drive_x ==0) {
//			SmartDashboard::PutNumber("Feeder Ultra Error", error);
			if(xError>8){
				//more than 8
				drive_x = 1.2* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError>5){
				//8-5
				drive_x = 1* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError>3){
				//3-5
				drive_x = .9* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError > .5){
				//.5-3
				drive_x = .8* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError>-.5){
				//-.5-.5
				drive_x = 0;
			}else if (xError>-3){
				//-.5--3
				drive_x = -.8* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError> -5){
				//-3--5
				drive_x = -.9* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError >-8){
				//-5--8
				drive_x = -1* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else if (xError <=-8){
				//less than 8
				drive_x = -1.2* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1;
			}else{
				drive_x= 0;
			}
//			leftUltraPID.lastValue = (getLeftUltra()>23.0?getLeftUltra():leftUltraPID.lastValue);
//			rightUltraPID.lastValue = (getRightUltra()>23.0?getRightUltra():rightUltraPID.lastValue);

		}
		feederAlignPID.lastValue = feederUltraVal;


		if ((drive_rotation == 0) && (oldRot != 0.0)){
			resetQ = 6;
		}
		if (resetQ != 0){
			if (resetQ == 3){
				imu->ZeroYaw();
//				gyroPID.setPoint = imu->GetYaw();
			}
			resetQ--;
		}
		oldRot = drive_rotation;

		//Gyro PID
			if((drive_rotation==0.0 && resetQ == 0 && !SmartDashboard::GetBoolean("disableGyro"))){
				gyroTime = gyroTimer.Get();
				gyroPID.mistake =  gyroPID.setPoint - gyroRate;
	//			SmartDashboard::PutNumber("Gyro PID Error", gyroPID.mistake);
				gyroPID.integral += (gyroPID.mistake *gyroTime);
				gyroPID.derivative = (gyroPID.mistake - gyroPID.lastError)/gyroTime;
				double gyroOutput = (gyroPID.P*gyroPID.mistake) + (gyroPID.I*gyroPID.integral) + (gyroPID.D*gyroPID.derivative);
	//			SmartDashboard::PutNumber("Gyro PID Out before", gyroOutput);
				gyroOutput = gyroOutput > 1.0 ? 1.0 : (gyroOutput < -1.0 ? -1.0 : gyroOutput); //Conditional (Tenerary) Operator limiting values to between 1 and -1
				drive_rotation = gyroOutput;
				gyroPID.lastError = gyroPID.mistake;
	//			SmartDashboard::PutNumber("Gyro PID Out", gyroOutput);
				gyroTimer.Reset();
				if (drive_rotation < .05 && drive_rotation > -.05){
					drive_rotation = 0;
				}
				if(dPad && SmartDashboard::GetBoolean("useDPad")){
					//Modify the y if perfect x is wanted
					if((drive_x != 0.0) && (drive_y == 0.0)){
						drive_y = (gyroPID.mistake * SmartDashboard::GetNumber("DPadYPValue"));
					}else if (drive_y != 0.0){
						drive_x = (gyroPID.mistake * SmartDashboard::GetNumber("DPadXPValue"));
					}
				}




			}

		//Bottom Tote Alignment
			if ((robot.joystick.button("lift_align") || alignOne) && drive_x == 0.0){
				//set vars
				leftPhotoVar = leftPhoto.Get();
				middlePhotoVar = middlePhoto.Get();
				rightPhotoVar = rightPhoto.Get();

				//different cases
				if ((leftPhotoVar && !rightPhotoVar)){
					alignPowerLeft+=-.05;
					drive_x = alignPowerLeft;
					alignError = false;
				}else if ((!leftPhotoVar && rightPhotoVar)){
					alignPowerRight+=.05;
					drive_x = alignPowerRight;
					alignError = false;
				}else if (!leftPhotoVar && middlePhotoVar && !rightPhotoVar){
					alignPowerLeft = -.5;
					alignPowerRight = .5;
					drive_x = 0;
					alignError = false;
					alignOne = false;
				}else if ((!leftPhotoVar && !middlePhotoVar && !rightPhotoVar) || (leftPhotoVar && middlePhotoVar && rightPhotoVar)){
					alignPowerLeft = -.5;
					alignPowerRight = .5;
					drive_x = 0;
					alignError = true;
					alignOne = false;
				}

				//SmartDashboard Set #Josh shenanigans
				if (!alignError){
					SmartDashboard::PutBoolean("leftPhoto", leftPhotoVar);
					SmartDashboard::PutBoolean("middlePhoto", middlePhotoVar);
					SmartDashboard::PutBoolean("rightPhoto", rightPhotoVar);
					SmartDashboard::PutBoolean("Tote align", robot.joystick.button("lift_align"));
				}else{
					if (SmartDashboard::GetBoolean("Tote align", true)){
						SmartDashboard::PutBoolean("leftPhoto", false);
						SmartDashboard::PutBoolean("middlePhoto", false);
						SmartDashboard::PutBoolean("rightPhoto", false);
						SmartDashboard::PutBoolean("Tote align", false);
					}else{
						SmartDashboard::PutBoolean("leftPhoto", true);
						SmartDashboard::PutBoolean("middlePhoto", true);
						SmartDashboard::PutBoolean("rightPhoto", true);
						SmartDashboard::PutBoolean("Tote align", true);
					}
				}
			}else{
				SmartDashboard::PutBoolean("leftPhoto", leftPhoto.Get());
				SmartDashboard::PutBoolean("middlePhoto", middlePhoto.Get());
				SmartDashboard::PutBoolean("rightPhoto", rightPhoto.Get());
				SmartDashboard::PutBoolean("Tote align", false);
			}

			//Top Tote Alignment
				if ((robot.joystick.button("top_lift_align") || alignTwo) && drive_x == 0.0){
					//set vars
					tl = topLeftPhoto.Get();
					tm = middlePhoto.Get();
					tr = topRightPhoto.Get();

					//different cases
					if ((tl && !tr)){
						alignPowerLeft+=-.05;
						drive_x = alignPowerLeft;
						alignError = false;
					}else if ((!tl && tr)){
						alignPowerRight+=.05;
						drive_x = alignPowerRight;
						alignError = false;
					}else if (!tl && tm && !tr){
						alignPowerLeft = -.5;
						alignPowerRight = .5;
						drive_x = 0;
						alignError = false;
						alignTwo = false;
					}else if ((!tl && !tm && !tr) || (tl && tm && tr)){
						alignPowerLeft = -.5;
						alignPowerRight = .5;
						drive_x = 0;
						alignError = true;
						alignTwo = false;
					}

					//SmartDashboard Set more #Josh shenanigains
					if (!alignError){
						SmartDashboard::PutBoolean("leftPhoto", tl);
						SmartDashboard::PutBoolean("middlePhoto", tm);
						SmartDashboard::PutBoolean("rightPhoto", tr);
						SmartDashboard::PutBoolean("Top Tote align", robot.joystick.button("lift_align"));
					}else{
						if (SmartDashboard::GetBoolean("Tote align", true)){
							SmartDashboard::PutBoolean("topLeftPhoto", false);
							SmartDashboard::PutBoolean("topMiddlePhoto", false);
							SmartDashboard::PutBoolean("topRightPhoto", false);
							SmartDashboard::PutBoolean("Top Tote align", false);
						}else{
							SmartDashboard::PutBoolean("topLeftPhoto", true);
							SmartDashboard::PutBoolean("topMiddlePhoto", true);
							SmartDashboard::PutBoolean("topRightPhoto", true);
							SmartDashboard::PutBoolean("Top Tote align", true);
						}
					}
				}else{
					SmartDashboard::PutBoolean("topLeftPhoto", topLeftPhoto.Get());
					SmartDashboard::PutBoolean("topMiddlePhoto", middlePhoto.Get());
					SmartDashboard::PutBoolean("topRightPhoto", topRightPhoto.Get());
					SmartDashboard::PutBoolean("Top Tote align", false);
				}

				if (/*!frontRight.GetControlMode() == mode*/!modeSet){
					robot.outLog.throwLog("set to mode");
					frontRight.SetControlMode(mode);
					frontLeft.SetControlMode(mode);
					backRight.SetControlMode(mode);
					backLeft.SetControlMode(mode);
					robot.outLog.throwLog("[CHANGE] Motors were switched to ", (int)mode);
					modeSet = true;
					}
//				robot.outLog.throwLog("test");

		// Drive modes
		//robot.outLog.throwLog("drive motor norm");
		if (simple){

		/// RIP simplicity
		if (!polar){
			if(shoulderSpeed){
				frontLeft.Set(frontLeftInvert
						*(drive_x+drive_y+drive_rotation)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
				frontRight.Set(frontRightInvert
						*(-drive_x+drive_y-drive_rotation)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
				backLeft.Set(backLeftInvert
						*(-drive_x+drive_y+drive_rotation)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
				backRight.Set(backRightInvert
						*(drive_x+drive_y-drive_rotation)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
			}else{
				frontLeft.Set(frontLeftInvert
						*(drive_x+drive_y+drive_rotation)
						* SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
				frontRight.Set(frontRightInvert
						*(-drive_x+drive_y-drive_rotation)
						*SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
				backLeft.Set(backLeftInvert
						*(-drive_x+drive_y+drive_rotation)
						*SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
				backRight.Set(backRightInvert
						*(drive_x+drive_y-drive_rotation)
						* SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*((mode == CANSpeedController::kVoltage)?12.0:1.0));
			}
		}else{
//			imu->ZeroYaw();

				frontLeft.Set(frontLeftInvert
						*(polarMag*sin(polarAngle+(PI/4.0)) + drive_rotation)
						*SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*(mode == CANSpeedController::kVoltage?12:1));
				frontRight.Set(frontRightInvert
						*(polarMag*cos(polarAngle+(PI/4.0)) - drive_rotation)
						*SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*(mode == CANSpeedController::kVoltage?12:1));
				backLeft.Set(backLeftInvert
						*(polarMag*cos(polarAngle+(PI/4.0)) + drive_rotation)
						*SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*(mode == CANSpeedController::kVoltage?12:1));
				backRight.Set(backRightInvert
						*(polarMag*sin(polarAngle+(PI/4.0)) - drive_rotation)
						*SmartDashboard::GetNumber("JoystickMultiplier",.2)
						*(mode == CANSpeedController::kVoltage?12:1));

		}
		}else{

//			if (!(frontRight.GetControlMode() == CANSpeedController::kSpeed)){
				robot.outLog.throwLog("set to Speed");
				frontRight.SetControlMode(CANSpeedController::kSpeed);
				frontLeft.SetControlMode(CANSpeedController::kSpeed);
				backRight.SetControlMode(CANSpeedController::kSpeed);
				backLeft.SetControlMode(CANSpeedController::kSpeed);
				robot.outLog.throwLog("[CHANGE] Encoders were switched to  Speed Mode");
//			}
		if(shoulderSpeed){
			frontLeft.Set(SmartDashboard::GetNumber("MaxVel")
					*frontLeftInvert
					*(drive_x+drive_y+drive_rotation));
			frontRight.Set(SmartDashboard::GetNumber("MaxVel")
					*frontRightInvert
					*(-drive_x+drive_y-drive_rotation));
			backLeft.Set(SmartDashboard::GetNumber("MaxVel")
					*backLeftInvert
					*(-drive_x+drive_y+drive_rotation));
			backRight.Set(SmartDashboard::GetNumber("MaxVel")
					*backRightInvert
					*(drive_x+drive_y-drive_rotation));
		}else{
			frontLeft.Set(SmartDashboard::GetNumber("MaxVel")
					*frontLeftInvert
					*(drive_x+drive_y+drive_rotation)
					*SmartDashboard::GetNumber("JoystickMultiplier",.2));
			frontRight.Set(SmartDashboard::GetNumber("MaxVel")
					*frontRightInvert
					*(-drive_x+drive_y-drive_rotation)
					*SmartDashboard::GetNumber("JoystickMultiplier",.2));
			backLeft.Set(SmartDashboard::GetNumber("MaxVel")
					*backLeftInvert
					*(-drive_x+drive_y+drive_rotation)
					*SmartDashboard::GetNumber("JoystickMultiplier"),.2);
			backRight.Set(SmartDashboard::GetNumber("MaxVel")
					*backRightInvert
					*(drive_x+drive_y-drive_rotation)
					*SmartDashboard::GetNumber("JoystickMultiplier"),.2);
		}
//		SmartDashboard::PutNumber("FrontLeftSetVel",SmartDashboard::GetNumber("MaxVel")*frontLeftInvert*(drive_x+drive_y+drive_rotation)* SmartDashboard::GetNumber("JoystickMultiplier",.2));
//		SmartDashboard::PutNumber("FrontRightSetVel",SmartDashboard::GetNumber("MaxVel")*frontRightInvert*(-drive_x+drive_y-drive_rotation)* SmartDashboard::GetNumber("JoystickMultiplier",.2));
//		SmartDashboard::PutNumber("BackLeftSetVel",SmartDashboard::GetNumber("MaxVel")*backLeftInvert*(-drive_x+drive_y+drive_rotation)*SmartDashboard::GetNumber("JoystickMultiplier",.2));
//		SmartDashboard::PutNumber("BackRightSetVel",SmartDashboard::GetNumber("MaxVel")*backRightInvert*(drive_x+drive_y-drive_rotation)* SmartDashboard::GetNumber("JoystickMultiplier",.2));
		}


	}

void DriveSubsystem::teleopEnd(void){
	robot.outLog.throwLog("drive tele end");
	frontLeft.SetSafetyEnabled(false);
	frontRight.SetSafetyEnabled(false);
	backLeft.SetSafetyEnabled(false);
	backRight.SetSafetyEnabled(false);
	frontLeft.Set(0.0);
	frontRight.Set(0.0);
	backLeft.Set(0.0);
	backRight.Set(0.0);
}
void DriveSubsystem::Pulse(){
	ultraPulse.SetVoltage(5.0);
	Wait(.00002);
	ultraPulse.SetVoltage(0.0);
}


float DriveSubsystem::getJumper(void)
{
	//Get Constant From Smart Dashboard
	return jumper.GetVoltage();
}
float DriveSubsystem::getLeftUltra(void)
{
//	ultraPulse.SetVoltage(5.0);
//	Wait(.00002);
//	ultraPulse.SetVoltage(0.0);
	return ((1000.0 * leftUltra.GetVoltage()) / ((getJumper() * 1000.0) / ultraVoltageScale));
}
float DriveSubsystem::getRightUltra(void)
{
//	ultraPulse.SetVoltage(5.0);
//	Wait(.00002);
//	ultraPulse.SetVoltage(0.0);
	return ((1000.0 * rightUltra.GetVoltage()) / ((getJumper() * 1000.0) / ultraVoltageScale));
}

float DriveSubsystem::getAutoUltra(void)
{
//	ultraPulse.SetVoltage(5.0);
//	Wait(.00002);
//	ultraPulse.SetVoltage(0.0);
	return ((1000.0 * autoUltra.GetVoltage()) / ((getJumper() * 1000.0) / ultraVoltageScale));
}

float DriveSubsystem::getFeederAlignUltra(void)
{
//	ultraPulse.SetVoltage(5.0);
//	Wait(.00002);
//	ultraPulse.SetVoltage(0.0);
	float currentUltraVoltage;
	//* SmartDashboard::GetBoolean("LeftSideFeeder")?-1:1
//	std::string choice = *(std::string*) feederStationChooser.GetSelected();
	if(!SmartDashboard::GetBoolean("LeftSideFeeder")){
		currentUltraVoltage = rightFeederAlignUltra.GetVoltage();
	}
	else {
		currentUltraVoltage = leftFeederAlignUltra.GetVoltage();
	}
	return ((1000.0 * currentUltraVoltage) / ((getJumper() * 1000.0) / ultraVoltageScale));
}


double DriveSubsystem::getDistance(void)
{
	double distance = (fabs(frontLeft.GetEncPosition())+fabs(frontRight.GetEncPosition())+fabs(backRight.GetEncPosition())+fabs(backLeft.GetEncPosition()))/4;
	return distance;
}
void DriveSubsystem::resetDistance(void){
	frontLeft.SetPosition(0);
	frontRight.SetPosition(0);
	backLeft.SetPosition(0);
	backRight.SetPosition(0);
}

void DriveSubsystem::punchSet(DoubleSolenoid::Value v){
	binPunch.Set(v);
}

void DriveSubsystem::mec_drive(double drive_x, double drive_y, double rotation)
{
	frontLeft.Set(frontLeftInvert
			*(drive_x+drive_y+rotation)
			*SmartDashboard::GetNumber("JoystickMultiplier",.2)
			*(mode == CANSpeedController::kVoltage?12:1));
	frontRight.Set(frontRightInvert
			*(-drive_x+drive_y-rotation)
			*SmartDashboard::GetNumber("JoystickMultiplier",.2)
			*(mode == CANSpeedController::kVoltage?12:1));
	backLeft.Set(backLeftInvert
			*(-drive_x+drive_y+rotation)
			*SmartDashboard::GetNumber("JoystickMultiplier",.2)
			*(mode == CANSpeedController::kVoltage?12:1));
	backRight.Set(backRightInvert
			*(drive_x+drive_y-rotation)
			*SmartDashboard::GetNumber("JoystickMultiplier",.2)
			*(mode == CANSpeedController::kVoltage?12:1));
}
double DriveSubsystem::getRot(void)
{
	return imu->GetYaw();
}
void DriveSubsystem::resetRot(void)
{
	imu->ZeroYaw();
//	gyroPID.setPoint = imu->GetYaw();
}
void DriveSubsystem::setPositionMode(void){
	frontLeft.SetControlMode(CANSpeedController::kPosition);
	backLeft.SetControlMode(CANSpeedController::kPosition);
	frontRight.SetControlMode(CANSpeedController::kPosition);
	backRight.SetControlMode(CANSpeedController::kPosition);
}
void DriveSubsystem::setVoltageMode(void){
	frontLeft.SetControlMode(CANSpeedController::kVoltage);
	backLeft.SetControlMode(CANSpeedController::kVoltage);
	frontRight.SetControlMode(CANSpeedController::kVoltage);
	backRight.SetControlMode(CANSpeedController::kVoltage);
}
void DriveSubsystem::setMode(CANSpeedController::ControlMode m){
	frontLeft.SetControlMode(m);
	backLeft.SetControlMode(m);
	frontRight.SetControlMode(m);
	backRight.SetControlMode(m);
}




void DriveSubsystem::setSpeedMode(void){
//	frontLeft.SetControlMode(CANSpeedController::kSpeed);
//	backLeft.SetControlMode(CANSpeedController::kSpeed);
//	frontRight.SetControlMode(CANSpeedController::kSpeed);
//	backRight.SetControlMode(CANSpeedController::kSpeed);
}
void DriveSubsystem::setFrontLeftMotor(double value){
	frontLeft.Set(value);
}
void DriveSubsystem::setFrontRightMotor(double value){
	frontRight.Set(value);
}
void DriveSubsystem::setBackLeftMotor(double value){
	backLeft.Set(value);
}
void DriveSubsystem::setBackRightMotor(double value){
	backRight.Set(value);
}
double DriveSubsystem::getJoystickMultiplier(void){
	return SmartDashboard::GetNumber("joystickMultiplier");
}
void DriveSubsystem::giveLog(std::string stringVar){
	robot.outLog.throwLog(stringVar);
}
double DriveSubsystem::gyroPIDCalc(double set, double rot, int mult){
	SmartDashboard::PutNumber("Gyro Angle", imu->GetYaw());
	gyroPID.P=(SmartDashboard::GetNumber("gyroPValue"));
	gyroPID.I=(SmartDashboard::GetNumber("gyroAutoIValue"));
	gyroPID.D=(SmartDashboard::GetNumber("gyroDValue"));
		gyroPID.mistake =  (set+gyroPID.setPoint) - rot;
		SmartDashboard::PutNumber("Gyro PID Error", gyroPID.mistake);
		double output = (gyroPID.P*gyroPID.mistake);
		output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
		drive_rotation = output;
//		gyroPID.lastError = gyroPID.mistake;
		SmartDashboard::PutNumber("Gyro PID Out", output);
	if (rot < .05 && rot > -.05){
		rot = 0;
	}
	output*=mult;
	return output;
}

double DriveSubsystem::autoUltraPIDCalc(double set){
	double ultraVal = getAutoUltra();
	double mistake =  ((set) - ((ultraVal<150)?ultraVal:set));
	double output = (fabs(mistake)>1.0)?(SmartDashboard::GetNumber("AutoUltraPValue")*mistake):0.0;
	robot.outLog.throwLog("\nUndeadbanded Ultra Output", output);
	output = output > .5 ? .5 : (output < -.5 ? -.5 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1 TODO Change?
	drive_rotation = output;
	if (output < .05 && output > -.05){
		output = 0;
	}
	if (output != 0){
		robot.outLog.throwLog("Auto Ultra Correcting at: ", output);
		robot.outLog.throwLog("Ultra At:", ultraVal);
	}
	return output;
}

bool DriveSubsystem::getLeftPhoto(){
	return leftPhoto.Get();
}
bool DriveSubsystem::getMiddlePhoto(){
	return middlePhoto.Get();
}
bool DriveSubsystem::getRightPhoto(){
	return rightPhoto.Get();
}
void DriveSubsystem::setMotorExpiration(bool position){
	frontLeft.SetSafetyEnabled(position);
	frontRight.SetSafetyEnabled(position);
	backLeft.SetSafetyEnabled(position);
	backRight.SetSafetyEnabled(position);
}
double DriveSubsystem::rateTest(void){
	double gyroSum = 0.0;
	for(int i = 0; i<11; i++){
//		gyroSum += fabs(gyro.GetRate());
	}
	robot.outLog.throwLog("Gyro Average Rate",gyroSum/10.0);
	return gyroSum/10.0;
}
void DriveSubsystem::reconstructGyro(void){
//	gyro.InitGyro();
}
void DriveSubsystem::setHeading(double val){
	gyroPID.setPoint = val;

//	gyro.InitGyro();
}

