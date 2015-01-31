#include "log.h"

void OutLog::createFile(){


		remove("/home/lvuser/logs/robotLog10.txt");
		rename("/home/lvuser/logs/robotLog9.txt","/home/lvuser/logs/robotLog2t.txt");
		rename("/home/lvuser/logs/robotLog8.txt","/home/lvuser/logs/robotLog9.txt");
		rename("/home/lvuser/logs/robotLog7.txt","/home/lvuser/logs/robotLog8.txt");
		rename("/home/lvuser/logs/robotLog6.txt","/home/lvuser/logs/robotLog7.txt");
		rename("/home/lvuser/logs/robotLog5.txt","/home/lvuser/logs/robotLog6.txt");
		rename("/home/lvuser/logs/robotLog4.txt","/home/lvuser/logs/robotLog5.txt");
		rename("/home/lvuser/logs/robotLog3.txt","/home/lvuser/logs/robotLog4.txt");
		rename("/home/lvuser/logs/robotLog2.txt","/home/lvuser/logs/robotLog3.txt");
		rename("/home/lvuser/logs/robotLog1.txt","/home/lvuser/logs/robotLog2.txt");
		fileName = "/home/lvuser/logs/robotLog1.txt";
	    outFile.open(fileName.c_str());
	    if(!outFile){
	    	std::cout << "ERROR: log file not created! at " << fileName << std::endl;
	    }
	    outFile << "Log File Successfully Created!" << std::endl;
}
void OutLog::throwLog(std::string s){
	time = DriverStation::GetInstance()->GetMatchTime();
	 std::cout << "Writing " << s.c_str() << " to log" << std::endl;
	outFile << "[" << time << "] " << s.c_str() << std::endl;
}
