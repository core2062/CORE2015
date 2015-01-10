#include "DriveSubsystem.h"
#include <iostream>


std::string DriveSubsystem::name(void){
	return "Drive";
}


void DriveSubsystem::robotInit(void){
	std::cout << "                                                  drive robo init" << std::endl;
//	robot.outLog.throwLog("DriveRobotInit");
}
void DriveSubsystem::teleopInit(void){
	robot.joystick.register_axis("drive_x", 1, 1);
	robot.joystick.register_axis("drive_rotation", 1, 4);
	robot.joystick.register_axis("drive_y", 1, 2);
	robot.joystick.register_button("liftUpButton", 2, 1);
	robot.joystick.register_button("liftDownButton", 2, 2);
}

void DriveSubsystem::teleop(void)
{
	drive_x = robot.joystick.axis("drive_x");
	if (drive_x < .2 && drive_x > -.2){
		drive_x = 0;
	}
	drive_rotation = robot.joystick.axis("drive_rotation");
	if (drive_rotation < .2 && drive_rotation > -.2){
		drive_rotation = 0;
	}
	drive_y = robot.joystick.axis("drive_y");
	if (drive_y < .2 && drive_y > -.2){
		drive_y = 0;
	}

	driveMotors.MecanumDrive_Cartesian(drive_x, drive_y, drive_rotation);
}
