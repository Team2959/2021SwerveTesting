// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>
#include <frc/DutyCycle.h>
#include <frc/DigitalInput.h>

class SwerveModule {
 public:
    SwerveModule(int driveMotorChannel, int turningMotorChannel, int dutyCycleChannel, std::string name, units::radian_t turnOffset);
    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState& state);
    void SetInitialPosition();
    void DirectDrive(bool drive, double percentage);
    double GetAbsoluteEncoderPosition();
 private:
    static constexpr double kWheelRadius = 2.0 * 0.254; // 2" * 0.254 m / inch
    static constexpr int kEncoderResolution = 4096;
    const double kGearboxRatio = 1.0 / 6.86; // One turn of the wheel is 6.86 turns of the motor
    const double kDrivePositionFactor = (2.0 * wpi::math::pi * kWheelRadius * kGearboxRatio);

    static constexpr auto kModuleMaxAngularVelocity =
        wpi::math::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kModuleMaxAngularAcceleration =
        wpi::math::pi * 2_rad_per_s / 1_s;  // radians per second^2

    std::string m_name;

    rev::CANSparkMax m_driveMotor;
    rev::CANSparkMax m_turningMotor;

    frc::DigitalInput m_dutyCycleInput;
    frc::DutyCycle m_dutyCycleEncoder;

    rev::CANEncoder m_driveEncoder{m_driveMotor.GetEncoder()};
    rev::CANEncoder m_turningEncoder{m_turningMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, kEncoderResolution)};
    
    rev::CANPIDController m_drivePIDController;
    rev::CANPIDController m_turningPIDController;

    units::radian_t m_turnOffset;

    //   frc2::PIDController m_drivePIDController{1.0, 0, 0};
    /*frc::ProfiledPIDController<units::radians> m_turningPIDController{
        1.0,
        0.0,
        0.0,
        {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};*/

    /*frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                    3_V / 1_mps};*/
    /*frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
        1_V, 0.5_V / 1_rad_per_s};*/
};
