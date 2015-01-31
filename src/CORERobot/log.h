#ifndef LOG_H
#define LOG_H

#include "WPILib.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>

class OutLog {
	private:
    	std::string fileName;
    	std::ofstream outFile;
    	double time;

	public:
    	OutLog(){
    		createFile();
    	time = DriverStation::GetInstance()->GetMatchTime();
    	}
    	void createFile();
    	void throwLog(std::string s);
		~OutLog(){
			outFile.close();
		}
	
	
};

#endif
