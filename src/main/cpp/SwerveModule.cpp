// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int dutyCycleChannel,
                           std::string name)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
     m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
     m_dutyCycleInput(dutyCycleChannel),
     m_dutyCycleEncoder(m_dutyCycleInput)
{
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetPositionConversionFactor(kDrivePositionFactor);
  m_driveEncoder.SetVelocityConversionFactor(kDrivePositionFactor / 60.0);

  m_drivePIDController.SetP(1.0);
  m_drivePIDController.SetI(0.0);
  m_drivePIDController.SetD(0.0);

  m_turningPIDController.SetP(1.0);
  m_turningPIDController.SetI(0.0);
  m_turningPIDController.SetD(0.0);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetPositionConversionFactor(2.0 * wpi::math::pi /*/ kEncoderResolution*/);
  m_name = name;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()))};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_turningEncoder.GetPosition()));

    frc::SmartDashboard::PutNumber(m_name + "/Current Speed", m_driveEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber(m_name + "/Target Speed", state.speed.to<double>());
    //m_drivePIDController.SetReference(state.speed.to<double>(), rev::ControlType::kVelocity);
    
    auto delta = state.angle - frc::Rotation2d(units::radian_t(m_turningEncoder.GetPosition()));
    auto setpoint = units::radian_t(m_turningEncoder.GetPosition()) + delta.Radians();

    frc::SmartDashboard::PutNumber(m_name + "/Current Angle", m_turningEncoder.GetPosition());
    frc::SmartDashboard::PutNumber(m_name + "/Target Angle", setpoint.to<double>());
    //m_turningPIDController.SetReference(setpoint.to<double>(), rev::ControlType::kPosition);

    frc::SmartDashboard::PutNumber(m_name + "/Current Angle Abs", m_dutyCycleEncoder.GetOutput());
    frc::SmartDashboard::PutNumber(m_name + "/Current Angle Abs (Freq)", m_dutyCycleEncoder.GetFrequency());
}

void SwerveModule::SetInitialPosition(double offsetInRadians)
{
  m_turningEncoder.SetPosition(offsetInRadians);
}

void SwerveModule::DirectDrive(bool drive, double percentage)
{
  if (drive)
    m_driveMotor.Set(percentage);
  else
    m_turningMotor.Set(percentage);
}
