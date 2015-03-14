#ifndef CORESUBSYSTEM_H
#define CORESUBSYSTEM_H

#include <string>

#include "WPILib.h"
#include "JoystickCache.h"
#include "COREAuto.h"
#include "COREDrive.h"
#include "log.h"
#include "COREDashboard.h"

namespace CORE {

class CORESubsystem;

class CORERobot {
	std::vector<CORESubsystem*> subsystems;
public:
	JoystickCache joystick;
	OutLog outLog;
	COREDash CD;
	
	CORERobot():
		subsystems(),
		joystick(),
		outLog(),
		CD(outLog)
	{}
	
	void add(CORESubsystem& subsystem);
	
	std::vector<CORESubsystem*>& getSubsystems(void);
	
	void robotInit(void);
	void teleopInit(void);
	void teleop(void);
	void teleopEnd(void);
	
	~CORERobot(){
	}
	
	void requirePneumatics(void);
};

class CORESubsystem{
	public:	
	virtual std::string name(void) = 0;
	CORERobot& robot;
	CORESubsystem(CORERobot& robot):
	robot(robot)
	{}
	
	
	virtual ~CORESubsystem(){};	// Suppresses GNU GCC warning. Can be removed under GCC version 4.3
	
	// Called before loop at start of Teleop period
	virtual void robotInit(void) = 0;
	
	virtual void teleopInit(void) = 0;
	
	//Called sequentially during loop, interleaved with other subsystems
	virtual void teleop(void) = 0;
	virtual void teleopEnd(void){
		//nothing
	}
};

}
#endif
