#include "COREAuto.h"
#include "WPILib.h"
using namespace CORE;

Action::Action(void){};

WaitAction::WaitAction(float duration):
	timer(){
	m_duration	= duration;
}

Action::ControlFlow WaitAction::call(void){
	if(timer.Get() == 0){
		timer.Start();
	}
	if(timer.Get()<=m_duration){
		return CONTINUE;
	}else{
		return END;
	}
}



void AutoSequencer::clear(void){
    while (!aqueue.empty())
    {
        aqueue.pop();
    }
	background.clear();
}

void AutoSequencer::add_action(Action& action){
	add_action(&action);
}

void AutoSequencer::add_action(Action* action){
//	cout<<"adding action"<<endl;
//	cout<<action<<endl;
	aqueue.push(action);
}

void AutoSequencer::init(void){
	aqueue.front()->init();
}

void AutoSequencer::iter(void){
//	cout<<"iter start"<<endl;
	if(aqueue.empty()){
		if(!empty_flag){
			log.throwLog("Auto Done");
			std::cout << "No remaining auto actions!"<<std::endl;
			empty_flag = true;
		}
		return;
	}
	Action* a = aqueue.front();
//	cout<<"current action: "<<a<<endl;
	Action::ControlFlow return_val = a->call();
	
//	cout<<"after action"<<endl;
	switch(return_val){
	case Action::CONTINUE:
		break;
	case Action::BACKGROUND:
		std::cout << "adding to background: " << a->name << std::endl;
		log.throwLog("adding to background:");
		log.throwLog(a->name);
		background.push_back(a);
		/* no break */
	case Action::END:
//		log.throwLog("Ending:");
//		log.throwLog(a->name);
		aqueue.pop();
		if(!aqueue.empty()){
			a = aqueue.front();
			std::cout << "new task: " << a << std::endl;
			a->init();
		}
//		log.throwLog("Starting:");
//		log.throwLog(a->name);

		break;
	}
	std::vector<Action*>::iterator it = background.begin();
	while(it != background.end()){
		std::cout << "  bkgrnd: " << *it << std::endl;
		Action::ControlFlow return_val = (*it)->call();
		if (return_val == Action::END){
			it = background.erase(it);
		} else {
			++it;
		}
	}
//	cout<<"after iter"<<endl;
}
