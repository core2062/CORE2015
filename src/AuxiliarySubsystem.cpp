/*
 * AuxiliarySubsystem.cpp

 *
 *  Created on: Mar 10, 2015
 *      Author: core
 */

#include "AuxiliarySubsystem.h"

void AuxiliarySubsystem::robotInit(void){
	robot.outLog.throwLog("AuxiliarySubsystem: RobotInit");
}
void AuxiliarySubsystem::teleopInit(void){
//	liftMotor.SetSafetyEnabled(true);
//	liftMotor.Set(0);
//	liftMotor.SetExpiration(0.25);
	robot.outLog.throwLog("AuxiliarySubsystem: TeleopInit Success");
}

void AuxiliarySubsystem::teleop(void){

}
void AuxiliarySubsystem::teleopEnd(){
//	liftMotor.SetSafetyEnabled(false);
//	liftMotor.Set(0.0);
}
void AuxiliarySubsystem::giveLog(std::string stringVar){
	robot.outLog.throwLog(stringVar);
}
