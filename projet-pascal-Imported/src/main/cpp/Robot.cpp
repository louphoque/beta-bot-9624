// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

//#include <frc/SPI.h>

/////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);
  m_chooser.AddOption(kAutoNameCustom3, kAutoNameCustom3);
  m_chooser.AddOption(kAutoNameCustom4, kAutoNameCustom4);
  m_chooser.AddOption(kAutoNameCustom5, kAutoNameCustom5);
  m_chooser.AddOption(kAutoNameCustom6, kAutoNameCustom6);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Un tour de l'encodeur=20 impulsions. Diametre de la roue=6 pouces (exemple).
  left_encoder.SetDistancePerPulse(3.14159*6/20);

  CANVenom_left_1.SetInverted(true);
  CANVenom_left_2.SetInverted(true);

  //ahrs = new AHRS(SPI::Port::kMXP);

    // Length is expensive to set, so only set it once, then just update data
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("  it: ", it);
  frc::SmartDashboard::PutNumber("  ia: ", ia);

  frc::SmartDashboard::PutBoolean("  LB: ", leftbump);
  frc::SmartDashboard::PutBoolean("  RB: ", rightbump);

  frc::SmartDashboard::PutBoolean("  bA: ", bA);
  frc::SmartDashboard::PutBoolean("  bB: ", bB);
  frc::SmartDashboard::PutBoolean("  bX: ", bX);
  frc::SmartDashboard::PutBoolean("  bY: ", bY);

  frc::SmartDashboard::PutNumber("  left x: ", left_x);
  frc::SmartDashboard::PutNumber("  left y: ", left_y);
  frc::SmartDashboard::PutNumber("  right x: ", right_x);
  frc::SmartDashboard::PutNumber("  right y: ", right_y);

  frc::SmartDashboard::PutNumber("  left_trigger: ", left_trigger);
  frc::SmartDashboard::PutNumber("  right_trigger: ", right_trigger);
  frc::SmartDashboard::PutNumber("  firstPixelHue: ", firstPixelHue);
  frc::SmartDashboard::PutBoolean("  lock_color: ", lock_color);

  frc::SmartDashboard::PutBoolean("  limit switch: ", limit_switch_value);

  frc::SmartDashboard::PutNumber("  ai_raw: ", ai_raw);
  frc::SmartDashboard::PutNumber("  ai_voltage: ", ai_voltage);

  frc::SmartDashboard::PutBoolean("  comp sw: ", compressor_status);

  frc::SmartDashboard::PutNumber("  enc count: ", encoder_count);
  frc::SmartDashboard::PutNumber("  enc distance: ", encoder_distance);
  frc::SmartDashboard::PutBoolean("  !enc switch: ", !encoder_switch_status);

}

/////////////////////////////////////////////////////////////////////////////////////////////////
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
void Robot::AutonomousInit() {
  m_timer.Restart();
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom1) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  ia=0;
  left_encoder.Reset();
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom1) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  ia++;

  // Drive for 2 seconds
  if (m_timer.Get() < 5_s) {
    // Drive forwards half speed, make sure to turn input squaring off
    m_robotDrive.ArcadeDrive(0.1, 0.0, false);
  } else {
    // Stop robot
    m_robotDrive.ArcadeDrive(0.0, 0.0, false);
  }


  UpdateLEDsMatrix(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::TeleopInit() {
  it=0;
  m_robotDrive.SetExpiration(100_ms);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::TeleopPeriodic() {
  it++;
  leftbump=xbox.GetLeftBumper();
  rightbump=xbox.GetRightBumper();
  bA=xbox.GetAButton();
  bB=xbox.GetBButton();
  bX=xbox.GetXButton();
  bY=xbox.GetYButton();

  left_x=xbox.GetLeftX();
  left_y=xbox.GetLeftY();

  right_x=xbox.GetRightX();
  right_y=xbox.GetRightY();

  right_trigger=xbox.GetRightTriggerAxis();
  left_trigger=xbox.GetLeftTriggerAxis();

  limit_switch_value=limit_switch.Get();

  ai_raw=ai.GetValue();
  ai_voltage=ai.GetVoltage();

  // Offset pour varier la position du servo entre 0 et 1 avec une joystick allant de -1 a +1
  s1.Set((1+left_y)/2); 
  s2.Set((1+right_y)/2);

  compressor_status=compressor.GetPressureSwitchValue();

  solenoid3.Set(bA);
  solenoid4.Set(bB);

  // Toggle color locking boolean on and off when right bumber is released
  if (xbox.GetRightBumperReleased()){
    lock_color=!lock_color;
  }

  // If LED color is not locked then it is adjusted based on trigger position
  if (!lock_color){
    // Explicit conversion from double to int
    int LEDhue = (int)(left_trigger*180.0) + 1;
    firstPixelHue=std::clamp(LEDhue,0,180);
  }



  // Old school motor control
  // speed limit or maximum power
  /*
  if (left_y<-0.5)
    left_y=-0.5;
  
  if (left_y>0.5)
    left_y=0.5;

  if (right_y<-0.5)
    right_y=-0.5;
  
  if (right_y>0.5)
    right_y=0.5;

  // deadband
  if (fabs(left_y)<0.1) 
    left_y=0.0;

  if (fabs(right_y)<0.1) 
    right_y=0.0;

  // Control "parabolique" de la vitesse
  if (left_y<0.0)
    sign=-1.0;
  else
    sign=1.0;
  
  left_y=left_y*left_y*sign;

  if (right_y<0.0)
    sign=-1.0;
  else
    sign=1.0;
  
  right_y=right_y*right_y*sign; 

  // Commnande des moteurs
  CANVenom_left.Set(left_y);
  CANVenom_right.Set(right_y);

  */
  left_x=std::clamp(left_x,-0.5,0.5);  // Limiter l'amplitude
  left_y=std::clamp(left_y,-0.5,0.5);  // Limiter l'amplitude
  right_x=std::clamp(right_x,-0.5,0.5);  // Limiter l'amplitude
  right_y=std::clamp(right_y,-0.75,0.75);  // Limiter l'amplitude

  // Deadband est inclus dans la classe mais a 0.02. Changeons le.
  m_robotDrive.SetDeadband(0.1);

  //m_robotDrive.TankDrive(left_y,right_y,true);
  m_robotDrive.ArcadeDrive(-right_y,-right_x,true);

    // Fill the buffer with a rainbow
  //Rainbow();
  SolidColor();
  // Set the LEDs
  m_led.SetData(m_ledBuffer);

  encoder_count=left_encoder.Get();
  encoder_distance=left_encoder.GetDistance();
  encoder_switch_status=encoder_switch.Get();
  // Simple test of the routine to send messages to Arduino.
  UpdateLEDsMatrix(encoder_count%7);
}

void Robot::Rainbow() {
  // For every pixel
  for (int i = 0; i < kLength; i++) {
    // Calculate the hue - hue is easier for rainbows because the color
    // shape is a circle so only one value needs to precess
    const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
    // Set the value
    m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
  }
  // Increase by to make the rainbow "move"
  firstPixelHue += 3;
  // Check bounds
  firstPixelHue %= 180;
}

void Robot::SolidColor() {
  // For every pixel
  for (int i = 0; i < kLength; i++) {
    // Calculate the hue - hue is easier for rainbows because the color
    // shape is a circle so only one value needs to precess
    const auto pixelHue = firstPixelHue %180;
    // Set the value
    m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
  }
}

void Robot::UpdateLEDsMatrix(int message) {
// High/Low voltage levels of DIO outputs are set based on the bit states corresponding to input message (int)
// DIO levels are monitored by an Arduino digital inputs. Coorespond message can be reconstructed on Arduino 
// then used to update LEDs matrix.
// split an integer "message" into individual bits and send to the Arduino    
// maximum i = 2^n - 1
        bool b1=false,b2=false,b3=false;
        do1.Set( ((message>>2)&1)==1 );
        do2.Set( ((message>>1)&1)==1);
        do3.Set(message&1 == 1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
