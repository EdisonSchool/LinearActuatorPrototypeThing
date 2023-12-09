// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <units/length.h>
#include <frc/DigitalInput.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc2/command/RunCommand.h>
#include <fmt/core.h>

class Shooter : public frc2::SubsystemBase {
  public:
    Shooter();
    void spin(double rpm);
    void set_hood(double pos);
    frc2::CommandPtr shoot();
  private:
  
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

    rev::CANSparkMax m_hood_motor {22, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_left_motor {30, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_right_motor {29, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

    rev::SparkMaxPIDController m_pid = m_left_motor.GetPIDController();
    rev::SparkMaxPIDController m_Hood_Pid = m_hood_motor.GetPIDController();

    frc::DigitalInput m_limit_switch{1};

    rev::SparkMaxRelativeEncoder m_hood_encoder = m_hood_motor.GetEncoder();
    rev::SparkMaxRelativeEncoder m_spin_encoder = m_left_motor.GetEncoder();

    double m_rpm = 400.0;
  
    double m_pos = -240.0;

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
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
