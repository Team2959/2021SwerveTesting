// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include "Constants.h"
#include "Logging.h"
#include <ctime>
#include <unistd.h>
#include <hal/cpp/fpga_clock.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_navX.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});


  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_navX.GetRotation2d(), m_frontLeft.GetState(),
                    m_frontRight.GetState(), m_backLeft.GetState(),
                    m_backRight.GetState());
}



std::string Drivetrain::OutputOdometry() {
  //Get Pose
  auto pose = m_odometry.GetPose();
  // Get Position (x,y) from pose
  auto poseX = pose.X();
  auto poseY = pose.Y();
  // Publish to smart dashboard
  frc::SmartDashboard::PutNumber(n_name + "/pose x", poseX.to<double>());
  frc::SmartDashboard::PutNumber(n_name + "/pose y", poseY.to<double>());

  std::string data = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(hal::fpga_clock::now().time_since_epoch()).count()) + ","
        + std::to_string(m_navX.GetAngle()) + ","
        + std::to_string(double(pose.X())) + ","
        + std::to_string(double(pose.Y())) + ","
        + "\n";
    return data;

}

void Drivetrain::SetInitialSwervePositions()
{    
    m_frontLeft.SetInitialPosition();
    m_frontRight.SetInitialPosition();
    m_backLeft.SetInitialPosition();
    m_backRight.SetInitialPosition();
}

void Drivetrain::DirectMotorDrive(bool drive, int motor, double percentage)
{
  switch (motor)
  {
    case 1:
      m_frontLeft.DirectDrive(drive, percentage);
      break;
    case 2:
      m_frontRight.DirectDrive(drive, percentage);
      break;
    case 3:
      m_backRight.DirectDrive(drive, percentage);
      break;
    case 4:
      m_backLeft.DirectDrive(drive, percentage);
      break;
  }
}


void Drivetrain::UpdateDashboardOnUpdate()
{
  m_frontLeft.UpdateDashboardOnEnable();
  m_frontRight.UpdateDashboardOnEnable();
  m_backLeft.UpdateDashboardOnEnable();
  m_backRight.UpdateDashboardOnEnable();
}