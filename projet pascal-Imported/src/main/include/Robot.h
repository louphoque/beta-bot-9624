// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogInput.h>
#include <frc/Servo.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/Encoder.h>
#include <CANVenom.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Timer.h>
#include "AHRS.h"
#include <array>
#include <frc/AddressableLED.h>
//#include <frc/I2C.h>


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
  void Rainbow();
  void SolidColor();
  void UpdateLEDsMatrix(int message);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom1 = "My Auto";
  const std::string kAutoNameCustom2 = "My Bike";
  const std::string kAutoNameCustom3 = "My Cata";
  const std::string kAutoNameCustom4 = "My Delta";
  const std::string kAutoNameCustom5 = "My Rocket";
  const std::string kAutoNameCustom6 = "My Submarine";
  std::string m_autoSelected;

  // Declaration des objets
  frc::Timer m_timer;
  frc::XboxController xbox{0};
  // Digital inputs
  frc::DigitalInput limit_switch{9};
  frc::Encoder left_encoder{0,1};
  frc::DigitalInput encoder_switch{2};
  // Analog inputs
  frc::AnalogInput ai{0};
  // PWM
  frc::Servo s1{8};
  frc::Servo s2{9};
  //Pneumatique
  frc::Compressor compressor{0,frc::PneumaticsModuleType::CTREPCM};
  frc::Solenoid solenoid0{0,frc::PneumaticsModuleType::CTREPCM,0};
  frc::Solenoid solenoid1{0,frc::PneumaticsModuleType::CTREPCM,1};
  frc::Solenoid solenoid2{0,frc::PneumaticsModuleType::CTREPCM,2};
  frc::Solenoid solenoid3{0,frc::PneumaticsModuleType::CTREPCM,3};
  frc::Solenoid solenoid4{0,frc::PneumaticsModuleType::CTREPCM,4};
  // Robot drive system
  pwf::CANVenom CANVenom_left_1{2};
  pwf::CANVenom CANVenom_left_2{3};
  pwf::CANVenom CANVenom_right_1{1};
  pwf::CANVenom CANVenom_right_2{4};
  
  frc::MotorControllerGroup MotorControllerGroup_left	{CANVenom_left_1,CANVenom_left_2};
  frc::MotorControllerGroup MotorControllerGroup_right	{CANVenom_right_1,CANVenom_right_2};
  frc::DifferentialDrive m_robotDrive {MotorControllerGroup_left,MotorControllerGroup_right};

  frc::DigitalOutput do1 {3};
  frc::DigitalOutput do2 {4};
  frc::DigitalOutput do3 {5};
  //frc::I2C i2c;


  // Declaration des variables
  // Xbox controller
  bool leftbump=false;
  bool rightbump=false;
  bool bA=false;
  bool bB=false;
  bool bX=false;
  bool bY=false;
  double left_x=0.0;
  double left_y=0.0;
  double right_x=0.0;
  double right_y=0.0;
  double right_trigger=0.0;
  double left_trigger=0.0;
  bool lock_color=false;
  // Compteurs et capteurs
  int it=0;
  int ia=0;
  bool limit_switch_value=false;
  int ai_raw=0;
  double ai_voltage=0.0;
  bool compressor_status=false;
  int encoder_count=0;
  double encoder_distance=0.0;
  bool encoder_switch_status=false;
  double sign=1.0;
  

  AHRS *ahrs;
  float yaw;

  static constexpr int kLength = 30;
    frc::AddressableLED m_led{0};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;

};
