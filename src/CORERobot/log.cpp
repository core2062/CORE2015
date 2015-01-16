#include "log.h"

void log::createFile(){


		remove("\\logs\\robotLog10.txt");
		rename("\\logs\\robotLog9.txt","\\logs\\robotLog2t.txt");
		rename("\\logs\\robotLog8.txt","\\logs\\robotLog9.txt");
		rename("\\logs\\robotLog7.txt","\\logs\\robotLog8.txt");
		rename("\\logs\\robotLog6.txt","\\logs\\robotLog7.txt");
		rename("\\logs\\robotLog5.txt","\\logs\\robotLog6.txt");
		rename("\\logs\\robotLog4.txt","\\logs\\robotLog5.txt");
		rename("\\logs\\robotLog3.txt","\\logs\\robotLog4.txt");
		rename("\\logs\\robotLog2.txt","\\logs\\robotLog3.txt");
		rename("\\logs\\robotLog1.txt","\\logs\\robotLog2.txt");
		fileName = "\\logs\\robotLog1.txt";
//	    outFile.open(fileName.c_str());
//	    if(!outFile){
	    	std::cout << "ERROR: log file not created!" << std::endl;
//	    }
//	    outFile << "Log File Successfully Created!" << std::endl;
}
void log::throwLog(std::string s){
	// std::cout << "Writing " << s.c_str() << " to log" << std::endl;
//	outFile << s.c_str() << std::endl;
}
