///*
// * COREDashboard.h
// *
// *  Created on: Mar 10, 2015
// *      Author: core
// */
#ifndef COREDASHBOARD_H
#define COREDASHBOARD_H

#include "WPILib.h"
#include <string>
#include <vector>
#include <iostream>
#include "log.h"



//namespace CORE{



class COREDash{
	std::vector<std::string> names;
	std::vector<double> values;
	std::vector<std::string> types;
	OutLog& outLog;
	public:
		COREDash(OutLog& log):
			outLog(log)
	{
			names.push_back("test");
			values.push_back(5);
			types.push_back("num");

	}

	void putNum(std::string s, double value);

	void putBool(std::string s, bool value);

	double getNum(std::string s);

	bool getBool(std::string s);

	int findPos(std::string s);

	void updateSD(void);

};

//}


#endif
