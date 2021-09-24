// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include "Conditioning.h"
#include "Dashboard.h"
#include <thread>
#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override
    {
        //std::thread threadRunFast{Dashboard::RunFast};
        //std::thread threadRunSlow{Dashboard::RunSlow};
        //std::thread threadRunReallySlow{Dashboard::RunReallySlow};
        m_swerve.SetInitialSwervePositions();

        m_conditioning.SetDeadband(0.2);
        m_conditioning.SetExponent(4.0);

        frc::SmartDashboard::PutBoolean("Drive or Turn", true);
        frc::SmartDashboard::PutNumber("Motor Corner", 1);
        frc::SmartDashboard::PutNumber("Motor Percentage", 0);
    }

    void AutonomousPeriodic() override {
        DriveWithJoystick(false);
        m_swerve.UpdateOdometry();
    }

    void TeleopPeriodic() override
    {
        DriveWithJoystick(true);
        m_swerve.UpdateOdometry();
    }

    void TestPeriodic() override
    {
        auto drive = frc::SmartDashboard::GetBoolean("Drive or Turn", true);
        auto motor = (int)frc::SmartDashboard::GetNumber("Motor Corner", 1);
        auto percentage = frc::SmartDashboard::GetNumber("Motor Percentage", 0);

        m_swerve.DirectMotorDrive(drive, motor, percentage);
    }

private:
    frc::XboxController m_controller{0};
    frc::Joystick m_joystick{0};
    cwtech::UniformConditioning m_conditioning{};
    Drivetrain m_swerve;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
    // to 1.
    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

    void DriveWithJoystick(bool fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        auto x = m_joystick.GetX();
        x = m_conditioning.Condition(x);
        auto xSpeed = -m_xspeedLimiter.Calculate(x) * Drivetrain::kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        auto y = m_joystick.GetY();
        y = m_conditioning.Condition(y);
        auto ySpeed = -m_yspeedLimiter.Calculate(y) * Drivetrain::kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        auto r = m_joystick.GetTwist();
        r = m_conditioning.Condition(r);
        auto rot = -m_rotLimiter.Calculate(r) * Drivetrain::kMaxAngularSpeed;

        m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
