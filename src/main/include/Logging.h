// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#pragma once

class Logging {
public:
  Logging();

  void StartNewLogFile();
  void Log(std::string logLine);

private:
  std::ofstream m_logFile;
};
