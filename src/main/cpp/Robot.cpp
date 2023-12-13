// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>

void Robot::RobotInit()
{
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
frc::SmartDashboard::PutNumber("position", m_position);

m_hood_motor.SetSmartCurrentLimit(40);
m_left_motor.SetSmartCurrentLimit(40);
m_right_motor.SetSmartCurrentLimit(40);


}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit()
{
 
// Get PID coefficients from dashboard
  // m_pidCoeff.kP         = frc::SmartDashboard::GetNumber("P Gain", m_pidCoeff.kP);
  // m_pidCoeff.kI         = frc::SmartDashboard::GetNumber("I Gain", m_pidCoeff.kI);
  // m_pidCoeff.kD         = frc::SmartDashboard::GetNumber("D Gain", m_pidCoeff.kD);
  // m_pidCoeff.kIz        = frc::SmartDashboard::GetNumber("I Zone", m_pidCoeff.kIz);
  // m_pidCoeff.kFF        = frc::SmartDashboard::GetNumber("Feed Forward", m_pidCoeff.kFF);
  // m_pidCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Max Output", m_pidCoeff.kMaxOutput);
  // m_pidCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Min Output", m_pidCoeff.kMinOutput);

  // Set PID coefficients Shooter
  m_pid.SetP(m_pidCoeff.kP);
  m_pid.SetI(m_pidCoeff.kI);
  m_pid.SetD(m_pidCoeff.kD);
  m_pid.SetIZone(m_pidCoeff.kIz);
  m_pid.SetFF(m_pidCoeff.kFF);
  m_pid.SetOutputRange(m_pidCoeff.kMinOutput, m_pidCoeff.kMaxOutput);

  // Set PID coefficients Hood
  m_HoodPid.SetP(m_pidCoeffHood.kP);
  m_HoodPid.SetI(m_pidCoeffHood.kI);
  m_HoodPid.SetD(m_pidCoeffHood.kD);
  m_HoodPid.SetIZone(m_pidCoeffHood.kIz);
  m_HoodPid.SetFF(m_pidCoeffHood.kFF);
  m_HoodPid.SetOutputRange(m_pidCoeffHood.kMinOutput, m_pidCoeffHood.kMaxOutput);
  
  m_right_motor.Follow(m_left_motor, true);

// Because why not, goes to bottom when initialized
 //m_HoodPid.SetReference(m_position, rev::CANSparkMax::ControlType::kPosition);

// Because why not, goes to bottom when initialized
 //m_hood_motor.Set(0.1);
 m_state = DOWN;
 //if (m_limit_switch.Get()){
  //m_hood_encoder.SetPosition(0);
 //}

}

void Robot::TeleopPeriodic(){

m_vision.getDistanceToTarget();

//Lime Light Number 10.22.40.85 
//The port to watch limelight is :5801
//10.22.40.1:5801

//CHANGE STATES ON BUTTON PRESS//
  if (stick.GetLeftY() < -0.5)
{
  m_state = UP;
}

 else if (stick.GetLeftY() > 0.5)
{
  m_state = DOWN;
}

 else
{
  m_state = STOP;
}


//STATE MACHINE//
switch (m_state)
{
case UP:
//std::cout << "up\n"; commented out since hood difficulties
    m_HoodPid.SetReference(-1 * m_position, rev::CANSparkMax::ControlType::kPosition);
break;

case DOWN:
    //std::cout << "down\n"; commented out since hood difficulties
    m_HoodPid.SetReference(m_position, rev::CANSparkMax::ControlType::kPosition);

break;

case STOP:
    //std::cout << "stopped\n"; commented out since hood difficulties
    m_hood_motor.Set(0);

break;


default:
  break;
}

if (!m_limit_switch.Get())

{
  std::cout << "Limit Switch\n";
  m_state = STOP;
  m_hood_encoder.SetPosition(0);
}


//Soft Limits
m_hood_motor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
m_hood_motor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0);



//Lowest position we want for 72

    frc::SmartDashboard::PutNumber("current rpm", m_spin_encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("current position", m_hood_encoder.GetPosition());
    frc::SmartDashboard::PutNumber("rpm", m_rpm);
    frc::SmartDashboard::PutNumber("position", m_position);

  //INTAKING//
 if(stick.GetRightTriggerAxis() >= 0.1)
  {


    m_pid.SetReference(m_rpm, rev::CANSparkMax::ControlType::kVelocity);
    //m_left_motor.Set(stick.GetLeftTriggerAxis());

  }
  //EXTAKING//
 else if(stick.GetLeftTriggerAxis()>= 0.1)
 {
   m_pid.SetReference(-1 * m_rpm, rev::CANSparkMax::ControlType::kVelocity);
  //m_left_motor.Set(-1*stick.GetLeftTriggerAxis());
 }
 //STOPPED//
 else{
   m_left_motor.Set(0);
 }

}


//VISION//
units::meter_t Vision::getDistanceToTarget()
{

  if((m_table->GetNumber("tv", 0.0) > 0.5)){
    
    auto target = m_table->GetNumberArray("camerapose_targetspace", m_zero_vector);
  frc::SmartDashboard::PutNumberArray("value list", target);
    // frc::SmartDashboard::PutNumber("tx", target[0]);
    // frc::SmartDashboard::PutNumber("ty", target[1]);
    // frc::SmartDashboard::PutNumber("tz", target[2]);
    // frc::SmartDashboard::PutNumber("rx", target[3]);
    // frc::SmartDashboard::PutNumber("ry", target[4]);
    // frc::SmartDashboard::PutNumber("rz", target[5]);


    auto distance = units::meter_t{sqrt(target[0] * target[0] + target[1] * target[1] + target[2] * target[2])};
    frc::SmartDashboard::PutNumber("distance", distance.value());

    return distance;
    // return 0.0_in;
  }
  else{
    return 0.0_in;
  }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
