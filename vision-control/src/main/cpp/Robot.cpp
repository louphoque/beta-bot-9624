#include <frc/TimedRobot.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cameraserver/CameraServer.h>
#include <CANVenom.h>
#include <frc/Servo.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override {
        table = nt::NetworkTableInstance::GetDefault().GetTable("vision");

        //---------------------------------------------------
        // Intégrer les deux moteurs de controle au meme parent
        wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_left);
        wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_right);
        
        //---------------------------------------------------
        // Selection du moteur qui doit être inverser pour
        // assurer le bon controle du robot
        CANVenom_right.SetInverted(true);

        //---------------------------------------------------
        // Sécuriter des moteurs
        m_robotDrive.SetExpiration(100_ms);

        std::thread visionThread(VisionThread);
        visionThread.detach();
    }

    void TeleopPeriodic() override {
        void buttonA();

        if (m_controller.GetAButtonPressed()){
            automode = true;
        }
        if (m_controller.GetBButtonPressed()){
            automode = false;
        }

        if(automode){
            double x_person = table->GetNumber("x_person", -1);

            if (x_person > 0) {
                double error = x_person - 320;  // Centre de l'image (640x480)
                static double lastTurn = 0.0;  // Sauvegarde de la dernière valeur
                double targetTurn = error * 0.005;  
                double turn;
                turn = 0.8 * lastTurn + 0.2 * targetTurn;  // Filtrage exponentiel
                lastTurn = turn;
                m_robotDrive.ArcadeDrive(0.3, 0.4*turn);
            } else {
                m_robotDrive.ArcadeDrive(0.0, 0.0);
            }
        }
        
        // Défnition des variables de controle
            double pr_speed = 0.4;
            double pr_rotation = 0.5;

            // Definition du code de controle
            double forward = m_controller.GetRightTriggerAxis();
            double backward = m_controller.GetLeftTriggerAxis();
            double turn = m_controller.GetLeftX();

            // Définition de la variable speed pour avancer
            double speed = forward - backward;

            // Definition du code  pour l'action du robot ( avancer, reculer et touner)
            m_robotDrive.ArcadeDrive(pr_speed*speed, pr_rotation*turn);

            //Définition du code pour faire la rotion de la caméra si la vitesse du robot deviens négative
            if (speed < 0)
            {
                servo_cam.SetAngle(0);
            } else {
                servo_cam.SetAngle(137);
            }

            // Reglage vitesse
            double pov;
            pov = m_controller.GetPOV();

            if (pov == 90 && pr_speed < 1)
            {
                pr_speed += 0.01;
            
            }
            if (pov == 270 && pr_speed > 0.1)
            {
                pr_speed -= 0.01;
            }
        
    }

    private:
        std::shared_ptr<nt::NetworkTable> table;
        //Définition des moteurs Venom
        pwf::CANVenom CANVenom_left{2};
        pwf::CANVenom CANVenom_right{1};

        frc::XboxController m_controller{0};
        
        frc::Servo servo_cam{1};
        //Définition de comment les moteurs doivent se comporter
        frc::DifferentialDrive m_robotDrive{
        CANVenom_left, CANVenom_right};
        bool automode = false;

    static void VisionThread() {
      
      //Définis que la variable camera est maintenant utiliser
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

      //Définis que la camera a une définition de 640x480
      camera.SetResolution(160, 120);

      //Définis que la camera a une compressention de 
      //camera.SetPixelFormat();

      //Définis le nbre fps de la camera
      camera.SetFPS(30);

      // Met en action un serveur cvSink et indique a la camera de capturer la vidéo
      cs::CvSink cvSink = frc::CameraServer::GetVideo();

      // Met en place le service de renvois de video au dashboard avec un rectangle dessiner
      cs::CvSource outputStream =
          frc::CameraServer::PutVideo("Rectangle", 640, 480);

      // Défini un claque statique pour un rectangle
      cv::Mat mat;

      while (true) {
        // si jamais il y a un probleme dans la capture de l'image
        // depuis la camera, renvoie un code d'erreur
        if (cvSink.GrabFrame(mat) == 0) {

          // Envoie l'erreur dans les logs
          outputStream.NotifyError(cvSink.GetError());

          // Continue
          continue;

        }

        // Place un rectangle dans la vidéo
        rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
                  cv::Scalar(255, 255, 255), 5);

        // Donne au dashboard une nouvelle image comme un nouveau calque
        outputStream.PutFrame(mat);

      }
    }
};

#ifndef RUNNING_FRC_TESTS
int main() { 
    return frc::StartRobot<Robot>(); 
}
#endif