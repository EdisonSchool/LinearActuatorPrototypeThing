// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

// This method will be called once per scheduler run
Shooter::Shooter()
{
    m_left_motor.SetSmartCurrentLimit(30);
    //TODO add robotinit and teleop init code here
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

 //Shooter
    frc::SmartDashboard::PutNumber("P Gain", m_pidCoeff.kP);
    frc::SmartDashboard::PutNumber("I Gain", m_pidCoeff.kI);
    frc::SmartDashboard::PutNumber("D Gain", m_pidCoeff.kD);
    frc::SmartDashboard::PutNumber("I Zone", m_pidCoeff.kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", m_pidCoeff.kFF);
    frc::SmartDashboard::PutNumber("Max Output", m_pidCoeff.kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", m_pidCoeff.kMinOutput);

 //Hood
    frc::SmartDashboard::PutNumber("P Gain", m_pidCoeffHood.kP);
    frc::SmartDashboard::PutNumber("I Gain", m_pidCoeffHood.kI);
    frc::SmartDashboard::PutNumber("D Gain", m_pidCoeffHood.kD);
    frc::SmartDashboard::PutNumber("I Zone", m_pidCoeffHood.kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", m_pidCoeffHood.kFF);
    frc::SmartDashboard::PutNumber("Max Output", m_pidCoeffHood.kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", m_pidCoeffHood.kMinOutput);

    frc::SmartDashboard::PutNumber("rpm", m_rpm);
    frc::SmartDashboard::PutNumber("position", m_pos);

   

 // Set PID coefficients Shooter
    m_pid.SetP(m_pidCoeff.kP);
    m_pid.SetI(m_pidCoeff.kI);
    m_pid.SetD(m_pidCoeff.kD);
    m_pid.SetIZone(m_pidCoeff.kIz);
    m_pid.SetFF(m_pidCoeff.kFF);
    m_pid.SetOutputRange(m_pidCoeff.kMinOutput, m_pidCoeff.kMaxOutput);

 // Set PID coefficients Hood
    m_Hood_Pid.SetP(m_pidCoeffHood.kP);
    m_Hood_Pid.SetI(m_pidCoeffHood.kI);
    m_Hood_Pid.SetD(m_pidCoeffHood.kD);
    m_Hood_Pid.SetIZone(m_pidCoeffHood.kIz);
    m_Hood_Pid.SetFF(m_pidCoeffHood.kFF);
    m_Hood_Pid.SetOutputRange(m_pidCoeffHood.kMinOutput, m_pidCoeffHood.kMaxOutput);
    
    m_hood_motor.SetSmartCurrentLimit(40);
    m_left_motor.SetSmartCurrentLimit(40);
    m_right_motor.SetSmartCurrentLimit(40);
    m_right_motor.Follow(m_left_motor, true);
}
void Shooter::spin(double speed)
{
    m_pid.SetReference(speed, rev::ControlType::kVelocity);
};


void Shooter::set_hood(double pos)
{
    m_Hood_Pid.SetReference(pos, rev::CANSparkMax::ControlType::kPosition);
}

frc2::CommandPtr Shooter::shoot(){
    return frc2::RunCommand(
        [this]{
            set_hood(m_pos);
            if(m_hood_encoder.GetPosition() <= 1.02* m_pos && m_hood_encoder.GetPosition() >= .98* m_pos)
            {
                fmt::print("test");
                spin(m_rpm);
            }
    }, {this}
    ).ToPtr();
}

