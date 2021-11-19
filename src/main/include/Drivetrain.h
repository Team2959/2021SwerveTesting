// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/math>
#include <AHRS.h>

#include "Constants.h"

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { 
    
  frc::SmartDashboard::PutNumber("Swerve/P", 0);
  frc::SmartDashboard::PutNumber("Swerve/I", 0);
  frc::SmartDashboard::PutNumber("Swerve/D", 0);
  frc::SmartDashboard::PutNumber("Swerve/FF", 0.1);
    m_navX.Reset();
     }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();
  void SetInitialSwervePositions();
  void UpdateDashboardOnUpdate();
  void DirectMotorDrive(bool drive, int motor, double percentage);

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second

 private:
  // frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  // frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
//   frc::Translation2d m_backLeftLocation{+0.381_m, +0.381_m};
//   frc::Translation2d m_backRightLocation{+0.381_m, -0.381_m};
  // frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  // frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};


  frc::Translation2d m_frontLeftLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_frontRightLocation{-0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, +0.381_m};

  SwerveModule m_frontLeft{1, 11, 5, "front left", ZeroedFrontLeft};
  SwerveModule m_frontRight{2, 12, 6, "front right", ZeroedFrontRight};
  SwerveModule m_backLeft{4, 14, 7, "back left", ZeroedBackLeft};
  SwerveModule m_backRight{3, 13, 8, "back right", ZeroedBackRight};

  AHRS m_navX{frc::SPI::kMXP};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_navX.GetRotation2d()};
};
