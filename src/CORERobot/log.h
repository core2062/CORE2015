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
    	Timer timer;
    	uint8_t mode;

	public:
    	enum Mode{
    		TELE = 0,
			AUTO = 1
    	};
    	OutLog(){
    		createFile();
    		mode = TELE;
    	}
    	void createFile();
    	void throwLog(std::string s);
    	void throwLog(double s);
    	void throwLog(std::string s, double d);
    	void startTime();
    	void setMode(Mode m);
		~OutLog(){
			outFile.close();
		}
	
	
};

#endif
