// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Logging.h"
#include "Filesystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

Logging::Logging() = default;

void Logging::StartNewLogFile() {
    if(m_logFile.is_open()) 
        m_logFile.close();
    std::string usbDirectory = GetFirstDirectory("/media");
    std::string filename;
    if(usbDirectory.length() == 0)
    {
        std::string odometryDirectory = "/home/lvuser/odometry";
        filename = GetFirstModifiedFile(odometryDirectory);
        filename = odometryDirectory + "/" + filename;
    }
    else
    {
        usbDirectory = "/media/" + usbDirectory;
        int i;
        std::string random_str = RandomString();
        for(i = 0; std::ifstream{usbDirectory + "/" + random_str + std::to_string(i) + ".csv"}.good(); i++);
        filename = usbDirectory + "/" + random_str + std::to_string(i) + ".csv";
    }
    std::cout << "Log File:" << filename << std::endl;
    m_logFile.open(filename);
    if(!m_logFile.good() || !m_logFile.is_open())
    {
        std::cout << "Log File failed to open" << std::endl;
    }
    m_logFile << "timestamp,angle,poseX,poseY\n" << std::flush;
    frc::SmartDashboard::PutString("Odometry File", filename);
}

void Logging::Log(std::string logLine) {
    //std::string data = m_container.GetLoggingData();
    m_logFile << logLine << std::flush;
}
