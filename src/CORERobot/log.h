#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>

class log {
	private:
    	std::string fileName;
//    	ofstream outFile;
	public:
    	log(){
    		createFile();
    	}
    	void createFile();
    	void throwLog(std::string s);
		~log(){
//			outFile.close();
		}
	
	
};

#endif
