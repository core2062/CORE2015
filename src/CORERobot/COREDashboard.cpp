#include "COREDashboard.h"
////
////using namespace CORE;
////
void COREDash::putNum(std::string s, double value){
	int pos = findPos(s);
	if (pos == -1){
		names.push_back(s);
		values.push_back(value);
		types.push_back("num");
	}else{
		values[pos] = value;
	}
}

void COREDash::putBool(std::string s, bool value){
	int pos = findPos(s);
	if (pos == -1){
		names.push_back(s);
		values.push_back((double)value);
		types.push_back("bool");
	}else{
		values[pos] = (double)value;
	}
}

double COREDash::getNum(std::string s){
	int pos = findPos(s);
	if (pos == -1){
		outLog.throwLog("ERROR could not find: "+s+" on SmartDashboard");
		return 0;
	}else{
		return (double)values[pos];
	}
}

bool COREDash::getBool(std::string s){
	int pos = findPos(s);
	if (pos == -1){
		outLog.throwLog("ERROR could not find: "+s+" on SmartDashboard");
		return 0;
	}else{
		return (bool)values[pos];
	}
}

int COREDash::findPos(std::string s){
	for (uint32_t i = 0; i <= names.size(); i++){
		if (s == names[i]){
			return i;
		}
	}
	return -1;
}

void COREDash::updateSD(void){
	//put
	for (uint32_t i = 0; i <= names.size(); i++){
		if (types[i] == "bool"){
			SmartDashboard::PutBoolean(names[i],(bool)values[i]);
		}else if (types[i] == "num"){
			SmartDashboard::PutNumber(names[i],values[i]);
		}
	}
	//get
	for (uint32_t i = 0; i <= names.size(); i++){
		if (types[i] == "bool"){
			values[i] = (double)SmartDashboard::GetBoolean(names[i],false);
		}else if (types[i] == "num"){
			values[i] = SmartDashboard::GetNumber(names[i],false);
		}
	}

}
