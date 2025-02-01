//--------------------------------------------------------
// Introduction des library
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

//-------------------------------------------------------
class Robot : public frc::TimedRobot {
 public:
  Robot() {

    //---------------------------------------------------
    // Intégrer les deux moteurs de controle au meme parent
    wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_left);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_right);
    
    //---------------------------------------------------
    // Selection du moteur qui doit être inverser pour
    // assuer le bon controle du robot
    CANVenom_right.SetInverted(true);

    //---------------------------------------------------
    // Sécuriter des moteurs
    m_robotDrive.SetExpiration(100_ms);

    //---------------------------------------------------
    // Start un timer (je pense)
    m_timer.Start();

    //---------------------------------------------------
    // Introduction du system pour la caméra
    std::thread visionThread(VisionThread);
    visionThread.detach();

    //--------------------------------------------------
    
    
  }

//----------------------------------------------------
// Utile pour des automatisations
  void AutonomousInit() override { m_timer.Restart(); } // Initialisation du code pour un auto

  void AutonomousPeriodic() override {} //  Une section de code pour lire les variables périodiquement
//----------------------------------------------------

//----------------------------------------------------
// Utile pour le Teleop
  void TeleopInit() override {} // Initialisation du code pour le Teleop

  void TeleopPeriodic() override { //  Une section de code pour lire les variables périodiquement
    
    // Défnition des variables de controle
    int pr_speed = 0.4;
    int pr_rotation = 0.5;

    // Definition du code de controle
    double forward = m_controller.GetRightTriggerAxis();
    double backward = m_controller.GetLeftTriggerAxis();
    double turn = m_controller.GetLeftX();

    // Définition de la variable speed pour avancer
    double speed = forward - backward;

    // Definition du code  pour l'action du robot ( avancer, reculer et touner)
    m_robotDrive.ArcadeDrive(pr_speed*speed, pr_rotation*turn);
  }

//---------------------------------------------------
// Partie du code pour les test
  void TestInit() override {}

  void TestPeriodic() override {}
//---------------------------------------------------

//--------------------------------------------------
// Section des variable
 private:
  //
  pwf::CANVenom CANVenom_left{2};
  pwf::CANVenom CANVenom_right{1};
  //frc::DifferentialDrive m_robotDrive{
      //[&](double output) { m_left.Set(output); },
      //[&](double output) { m_right.Set(output); }};
  frc::DifferentialDrive m_robotDrive{
  CANVenom_left, CANVenom_right
  };
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
