#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <CANVenom.h>

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    //wpi::SendableRegistry::AddChild(&m_robotDrive, &m_left);
    //wpi::SendableRegistry::AddChild(&m_robotDrive, &m_right);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_left);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_right);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_right.SetInverted(true);
    CANVenom_left.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  }

  void AutonomousInit() override { m_timer.Restart(); }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 2_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.5, 0.0, false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    // Drive with arcade style (50% reduction of the steering because it is very rapid)
    m_robotDrive.ArcadeDrive(-m_controller.GetRightY(),
                             -0.5*m_controller.GetRightX());
  }

  void TestInit() override {}

  void TestPeriodic() override {}

 private:
  // Robot drive system
  //frc::PWMSparkMax m_left{0};
  //frc::PWMSparkMax m_right{1};
  
  pwf::CANVenom CANVenom_left{2};
  pwf::CANVenom CANVenom_right{1};
  //frc::DifferentialDrive m_robotDrive{
      //[&](double output) { m_left.Set(output); },
      //[&](double output) { m_right.Set(output); }};
  frc::DifferentialDrive m_robotDrive{
      [&](double output) { CANVenom_left.Set(output); },
      [&](double output) { CANVenom_right.Set(output); }};
  frc::XboxController m_controller{0};
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
