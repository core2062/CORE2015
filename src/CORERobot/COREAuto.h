#ifndef COREAUTO_H
#define COREAUTO_H

#include <string>
#include <queue>
#include <vector>
#include <iostream>
#include "CORERobot.h"
#include "log.h"

#include "WPILib.h"

namespace CORE{
	
	/*
	 * Basic Action class
	 * an action can return 3 states:
	 * Continue : the action will happen again
	 * End : the action is done
	 * Background : The action is on a background list where all background actions can happen simultaneously
	 */
class Action{
	public:
		std::string name;
		enum ControlFlow{
			CONTINUE,
			END,
			BACKGROUND
		};
		Action();
		virtual ControlFlow call(void){std::cout << "action base class operator"<< std::endl; return END;} // =0
		virtual void init(void){name = "default";}
		virtual ~Action(){}
	};

	/*
	 * A spacer action allowing the robot to wait a period of time
	 */
class WaitAction : public Action{
	float m_duration;
	Timer timer;
public:
	WaitAction(float duration);
	virtual ControlFlow call(void);
	virtual ~WaitAction(void){};
};

	/*
	 * Holds all auto actions that are added to it
	 */
class AutoSequencer{
	std::queue<Action*> aqueue;
	std::vector<Action*> background;
	OutLog& log;
	bool empty_flag;
public:
	AutoSequencer(OutLog& l):
		aqueue(),
		background(),
		log(l),
		empty_flag(false)
		{}
	void clear(void);
	void add_action(Action& action);
	void add_action(Action* action);
	void init(void);
	void iter(void);
};

}
#endif
