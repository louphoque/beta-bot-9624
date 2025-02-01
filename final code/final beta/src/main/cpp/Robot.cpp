#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <CANVenom.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cameraserver/CameraServer.h>
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
    CANVenom_right.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
    std::thread visionThread(VisionThread);
    visionThread.detach();
  }

  double x;
  double x_final;
  double y;
  double y_final;
  double max_speed_tele;
  int pov;

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
    m_robotDrive.ArcadeDrive(-m_controller.GetLeftY(),
                             -0.5*m_controller.GetRightX());
  }

  void TestInit() override {}

  void TestPeriodic() override {}

  void maxSpeedTele()
  {
    pov = m_controller.GetPOV();

    if (pov == 90 && max_speed_tele < 1)
    {
      max_speed_tele += 0.01;
      
    }
    if (pov == 270 && max_speed_tele > 0.1)
    {
      max_speed_tele -= 0.01;
    }
  }

// ----------------------------------------------------------------------------
//


// ----------------------------------------------------------------------------
//


 private:
  // Robot drive system
  //frc::PWMSparkMax m_left{0};
  //frc::PWMSparkMax m_right{1};
  //frc::PWMSparkMax m_left{0};
  //frc::PWMSparkMax m_right{1};
  pwf::CANVenom CANVenom_left{2};
  pwf::CANVenom CANVenom_right{1};
  //frc::DifferentialDrive m_robotDrive{
      //[&](double output) { m_left.Set(output); },
      //[&](double output) { m_right.Set(output); }};
  frc::DifferentialDrive m_robotDrive{
      [&](double max_speed_tele) { CANVenom_left.Set(-1*max_speed_tele); },
      [&](double max_speed_tele) { CANVenom_right.Set(-1*max_speed_tele); }};
  frc::XboxController m_controller{0};
  frc::Timer m_timer;
  static void VisionThread() {
      // Get the USB camera from CameraServer
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
      // Set the resolution
      camera.SetResolution(640, 480);

      // Get a CvSink. This will capture Mats from the Camera
      cs::CvSink cvSink = frc::CameraServer::GetVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      cs::CvSource outputStream =
          frc::CameraServer::PutVideo("Rectangle", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      cv::Mat mat;

      while (true) {
        // Tell the CvSink to grab a frame from the camera and
        // put it
        // in the source mat.  If there is an error notify the
        // output.
        if (cvSink.GrabFrame(mat) == 0) {
          // Send the output the error.
          outputStream.NotifyError(cvSink.GetError());
          // skip the rest of the current iteration
          continue;
        }
        // Put a rectangle on the image
        rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
                  cv::Scalar(255, 255, 255), 5);
        // Give the output stream a new image to display
        outputStream.PutFrame(mat);
      }
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
