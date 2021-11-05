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
                           std::string name,
                           units::radian_t turnOffset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
     m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
     m_dutyCycleInput(dutyCycleChannel),
     m_dutyCycleEncoder(m_dutyCycleInput),
     m_drivePIDController(m_driveMotor.GetPIDController()),
     m_turningPIDController(m_turningMotor.GetPIDController()),
     m_turnOffset(turnOffset)
{
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetPositionConversionFactor(kDrivePositionFactor);
  m_driveEncoder.SetVelocityConversionFactor(kDrivePositionFactor / 60.0);

  m_drivePIDController.SetP(0.4);
  m_drivePIDController.SetI(0.0);
  m_drivePIDController.SetD(0.0);

  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);
  m_turningPIDController.SetP(0.4);
  m_turningPIDController.SetI(0.00001);
  m_turningPIDController.SetD(0.0);
  m_turningPIDController.SetIZone(1.0);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetPositionConversionFactor(2.0 * wpi::math::pi /*/ kEncoderResolution*/);
  m_name = name;
}

double SwerveModule::GetAbsoluteEncoderPosition()
{
  auto initalPosition = m_dutyCycleEncoder.GetOutput(); // 0 to 1
  auto initalPositionInRadians = units::radian_t(initalPosition * 2 * wpi::math::pi); // radians
  auto initalPositionInRadiansScaled = frc::Rotation2d(initalPositionInRadians - m_turnOffset).Radians();
  return initalPositionInRadiansScaled.to<double>();
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

    frc::SmartDashboard::PutNumber(m_name + "/Current Angle", frc::Rotation2d{units::radian_t{m_turningEncoder.GetPosition()}}.Radians().to<double>());
    frc::SmartDashboard::PutNumber(m_name + "/Target Angle", setpoint.to<double>());
    m_turningPIDController.SetReference(setpoint.to<double>(), rev::ControlType::kPosition);

    frc::SmartDashboard::PutNumber(m_name + "/Current Angle Abs", GetAbsoluteEncoderPosition());
    frc::SmartDashboard::PutNumber(m_name + "/Current Angle Abs (Freq)", m_dutyCycleEncoder.GetFrequency());
}

void SwerveModule::SetInitialPosition()
{
  m_turningEncoder.SetPosition(GetAbsoluteEncoderPosition()/* / m_turningEncoder.GetPositionConversionFactor()*/);
  frc::SmartDashboard::PutNumber(m_name + "/Current Angle Abs", GetAbsoluteEncoderPosition());
  frc::SmartDashboard::PutNumber(m_name + "/Current Angle", frc::Rotation2d{units::radian_t{m_turningEncoder.GetPosition()}}.Radians().to<double>());

}

void SwerveModule::DirectDrive(bool drive, double percentage)
{
  if (drive)
    m_driveMotor.Set(percentage);
  else
    m_turningMotor.Set(percentage);
}
