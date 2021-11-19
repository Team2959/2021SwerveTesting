// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include "Constants.h"

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