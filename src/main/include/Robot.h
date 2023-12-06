// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <units/length.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Vision{

  public:

    units::meter_t getDistanceToTarget();


  private:

  std::shared_ptr<nt::NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  std::vector<double> m_zero_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  rev::CANSparkMax m_hood_motor {22, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::XboxController stick {0};

  rev::CANSparkMax m_left_motor {30, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_right_motor {29, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::SparkMaxPIDController m_pid = m_left_motor.GetPIDController();
  rev::SparkMaxPIDController m_HoodPid = m_hood_motor.GetPIDController();

  frc::DigitalInput m_limit_switch{1};

  rev::SparkMaxRelativeEncoder m_hood_encoder = m_hood_motor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_spin_encoder = m_left_motor.GetEncoder();

  Vision m_vision;
  enum STATE
  {
    STOP,
    UP,
    DOWN

  } m_state;


  double m_rpm = 6000.0;
  double m_position = 200.0;


    struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };
  pidCoeff m_pidCoeff {0.0001, 0.0, 0.002, 0.0, 0.00018, -1.0, 3.0};
  pidCoeff m_pidCoeffHood {0.2, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
};
