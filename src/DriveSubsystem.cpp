#include "DriveSubsystem.h"
#include <iostream>


std::string DriveSubsystem::name(void){
	return "Drive";
}


void DriveSubsystem::robotInit(void){
	robot.outLog.throwLog("DriveSubsystem: RobotInit Success");
}
void DriveSubsystem::teleopInit(void){
	robot.outLog.throwLog("DriveSubsystem: TeleopInit Success");

	frontLeft.SetSafetyEnabled(true);
	backLeft.SetSafetyEnabled(true);
	frontRight.SetSafetyEnabled(true);
	backRight.SetSafetyEnabled(true);
	frontLeft.Set(0.0);
	backLeft.Set(0.0);
	frontRight.Set(0.0);
	backRight.Set(0.0);
//	frontRight.SetExpiration(0.1);
//	frontLeft.SetExpiration(0.1);
//	backRight.SetExpiration(0.1);
//	backLeft.SetExpiration(0.1);
	robot.joystick.register_axis("drive_x", 1, 0);
	robot.joystick.register_axis("drive_rotation", 1, 2);
	robot.joystick.register_axis("drive_y", 1, 1);
	robot.joystick.register_button("shoulderSpeed", 1, 5);
	robot.joystick.register_button("lift_align",1,2);
	robot.joystick.register_button("top_lift_align",1,1);

//	frontLeft.SetControlMode(CANSpeedController::kSpeed);
//	backLeft.SetControlMode(CANSpeedController::kSpeed);
//	frontRight.SetControlMode(CANSpeedController::kSpeed);
//	backRight.SetControlMode(CANSpeedController::kSpeed);
//	oldFrontRight = frontRight.GetEncPosition();
//	oldFrontLeft = frontLeft.GetEncPosition();
//	oldBackRight = backRight.GetEncPosition();
//	oldBackLeft = backLeft.GetEncPosition();
	timer.Start();
	gyroTimer.Start();
	gyroTimer.Reset();
}
	
void DriveSubsystem::teleop(void){
//	gyro.SetSensitivity(SmartDashboard::GetNumber("Gyro Sensitivity", 0.0065));
	//robot.outLog.throwLog("start and smt dshbrd");
	double gyroRate = gyro.GetAngle();

//	if (gyroRate>-2 && gyroRate<2){
//		gyroRate = 0.0;
//	}

	shoulderSpeed = robot.joystick.button("shoulderSpeed");
	SmartDashboard::PutNumber("drive x", drive_x);
	SmartDashboard::PutNumber("drive y", drive_y);
	SmartDashboard::PutNumber("drive rot", drive_rotation);
	SmartDashboard::PutBoolean("Encoders-Status", isBroken);
	SmartDashboard::PutNumber("Front Left Set", frontLeft.Get());
	SmartDashboard::PutNumber("Front Right Set", frontRight.Get());
	SmartDashboard::PutNumber("Back Left Set", backLeft.Get());
	SmartDashboard::PutNumber("Back Right Set", backRight.Get());
//	SmartDashboard::PutNumber("Gyro Rate", gyroRate);
	SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
	SmartDashboard::PutNumber("Shoulder Speed", shoulderSpeed);

	SmartDashboard::PutBoolean("Tote align", robot.joystick.button("lift_align"));
	SmartDashboard::PutBoolean("Top Tote align", robot.joystick.button("top_lift_align"));
//	frontRight.SetVoltageRampRate(SmartDashboard::GetNumber("DriveVoltageRampRate"));
//	frontLeft.SetVoltageRampRate(SmartDashboard::GetNumber("DriveVoltageRampRate"));
//	backRight.SetVoltageRampRate(SmartDashboard::GetNumber("DriveVoltageRampRate"));
//	backLeft.SetVoltageRampRate(SmartDashboard::GetNumber("DriveVoltageRampRate"));


	switchEncoderMode = SmartDashboard::GetBoolean("Swtich-Encoder-Status", false);

	gyroPID.P=(SmartDashboard::GetNumber("gyroPValue"));
	gyroPID.I=(SmartDashboard::GetNumber("gyroIValue"));
	gyroPID.D=(SmartDashboard::GetNumber("gyroDValue"));

//	SmartDashboard::PutNumber("FLE", frontLeft.GetEncPosition());
//	SmartDashboard::PutNumber("FRE", frontRight.GetEncPosition());
//	SmartDashboard::PutNumber("BLE", backLeft.GetEncPosition());
//	SmartDashboard::PutNumber("BRE", backRight.GetEncPosition());

//Simple Dead-banding
	drive_x = robot.joystick.axis("drive_x");
	if (drive_x < 0.05 && drive_x > -.05){
		drive_x = 0;
	}

	drive_rotation = robot.joystick.axis("drive_rotation");
	if (drive_rotation < .05 && drive_rotation > -.05){
		drive_rotation = 0;
	}
	drive_y = robot.joystick.axis("drive_y");
	if (drive_y < .05 && drive_y > -.05){
		drive_y = 0;		
	}
	drive_y *= -1;
	if ((drive_rotation == 0) && (oldRot != 0.0)){
		resetQ = 6;
	}
	if (resetQ != 0){
		if (resetQ == 3){
			gyro.Reset();
		}
		resetQ--;
	}
	oldRot = drive_rotation;
	//Gyro PID
		if((drive_rotation==0.0 && resetQ == 0)){
			gyroTime = gyroTimer.Get();
			gyroPID.mistake =  gyroPID.setPoint - gyroRate;
			SmartDashboard::PutNumber("Gyro PID Error", gyroPID.mistake);
			gyroPID.integral += (gyroPID.mistake *gyroTime);
			gyroPID.derivative = (gyroPID.mistake - gyroPID.lastError)/gyroTime;
			double output = (gyroPID.P*gyroPID.mistake) + (gyroPID.I*gyroPID.integral) + (gyroPID.D*gyroPID.derivative);
			SmartDashboard::PutNumber("Gyro PID Out before", output);
			output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
			drive_rotation = output;
			gyroPID.lastError = gyroPID.mistake;
			SmartDashboard::PutNumber("Gyro PID Out", output);
			gyroTimer.Reset();
		}

		//Tote Alignment
			if (robot.joystick.button("lift_align") && drive_x == 0.0){
				//set vars
				leftPhotoVar = leftPhoto.Get();
				middlePhotoVar = middlePhoto.Get();
				rightPhotoVar = rightPhoto.Get();

				//different cases
				if ((leftPhotoVar && !rightPhotoVar)){
					drive_x = -.6;
					alignError = false;
				}else if ((!leftPhotoVar && rightPhotoVar)){
					drive_x = .6;
					alignError = false;
				}else if (!leftPhotoVar && middlePhotoVar && !rightPhotoVar){
					drive_x = 0;
					alignError = false;
				}else if ((!leftPhotoVar && !middlePhotoVar && !rightPhotoVar) || (leftPhotoVar && middlePhotoVar && rightPhotoVar)){
					drive_x = 0;
					alignError = true;
				}

				//SmartDashboard Set
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

			//Tote Alignment
				if (robot.joystick.button("top_lift_align") && drive_x == 0.0){
					//set vars
					tl = topLeftPhoto.Get();
					tm = middlePhoto.Get();
					tr = topRightPhoto.Get();

					//different cases
					if ((tl && !tr)){
						drive_x = -.6;
						alignError = false;
					}else if ((!tl && tr)){
						drive_x = .6;
						alignError = false;
					}else if (!tl && tm && !tr){
						drive_x = 0;
						alignError = false;
					}else if ((!tl && !tm && !tr) || (tl && tm && tr)){
						drive_x = 0;
						alignError = true;
					}

					//SmartDashboard Set
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



		if(flag == false){
			robot.outLog.throwLog("set to voltage");
			frontRight.SetControlMode(CANSpeedController::kPercentVbus);
			frontLeft.SetControlMode(CANSpeedController::kPercentVbus);
			backRight.SetControlMode(CANSpeedController::kPercentVbus);
			backLeft.SetControlMode(CANSpeedController::kPercentVbus);
			robot.outLog.throwLog("[CHANGE] Encoders were switched to  Percent Mode");
			flag = true;
		}
		//robot.outLog.throwLog("drive motor norm");
		if(shoulderSpeed){
			frontLeft.Set(frontLeftInvert*(drive_x+drive_y+drive_rotation));
			frontRight.Set(frontRightInvert*(-drive_x+drive_y-drive_rotation));
			backLeft.Set(backLeftInvert*(-drive_x+drive_y+drive_rotation));
			backRight.Set(backRightInvert*(drive_x+drive_y-drive_rotation));
		}else{
			frontLeft.Set(frontLeftInvert*(drive_x+drive_y+drive_rotation)* SmartDashboard::GetNumber("JoystickMultipier",.2));
			frontRight.Set(frontRightInvert*(-drive_x+drive_y-drive_rotation)* SmartDashboard::GetNumber("JoystickMultipier",.2));
			backLeft.Set(backLeftInvert*(-drive_x+drive_y+drive_rotation)*SmartDashboard::GetNumber("JoystickMultipier"),.2);
			backRight.Set(backRightInvert*(drive_x+drive_y-drive_rotation)* SmartDashboard::GetNumber("JoystickMultipier"),.2);
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

void DriveSubsystem::mec_drive(double drive_x, double drive_y, double rotation)
{
	frontLeftSet = ((drive_x+drive_y+rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	frontRightSet = ((-drive_x+drive_y-rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	backLeftSet = ((-drive_x+drive_y+rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
	backRightSet = ((drive_x+drive_y-rotation)*SmartDashboard::GetNumber("JoystickMultiplier"));
}
double DriveSubsystem::getRot(void)
{
	return gyro.GetAngle();
}
void DriveSubsystem::resetRot(void)
{
	gyro.Reset();
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

double DriveSubsystem::gyroPIDCalc(double set, double rot){
	gyroPID.P=(SmartDashboard::GetNumber("gyroPValue"));
	gyroPID.I=(SmartDashboard::GetNumber("gyroIValue"));
	gyroPID.D=(SmartDashboard::GetNumber("gyroDValue"));
		gyroTime = gyroTimer.Get();
		gyroPID.mistake =  set - rot;
		SmartDashboard::PutNumber("Gyro PID Error", gyroPID.mistake);
		gyroPID.integral += (gyroPID.mistake *gyroTime);
		gyroPID.derivative = (gyroPID.mistake - gyroPID.lastError)/gyroTime;
		double output = (gyroPID.P*gyroPID.mistake) + (gyroPID.I*gyroPID.integral) + (gyroPID.D*gyroPID.derivative);
		SmartDashboard::PutNumber("Gyro PID Out before", output);
		output = output > 1.0 ? 1.0 : (output < -1.0 ? -1.0 : output); //Conditional (Tenerary) Operator limiting values to between 1 and -1
		drive_rotation = output;
		gyroPID.lastError = gyroPID.mistake;
		SmartDashboard::PutNumber("Gyro PID Out", output);
		gyroTimer.Reset();

	if (rot < .05 && rot > -.05){
		rot = 0;
	}
	return rot;
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

