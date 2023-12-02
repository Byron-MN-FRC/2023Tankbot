// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.photonvision.PhotonCamera; 

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;

  private final MotorController m_leftMotor = new WPI_TalonSRX(5);
  private final MotorController m_rightMotor = new WPI_TalonSRX(7);

      // Constants such as camera and target height stored. Change per robot and goal!
      final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(23);
      final double TARGET_HEIGHT_METERS = Units.inchesToMeters(21.5);
  
      // Angle between horizontal and the camera.
      final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
      // How far from the target we want to be
      final double GOAL_RANGE_METERS = Units.feetToMeters(2);
  
      // Change this to match the name of your camera
      PhotonCamera camera = new PhotonCamera("photonvision");
  
      // PID constants should be tuned per robot
      final double LINEAR_P = 0.1;
      final double LINEAR_D = 0.0;
      PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
      final double ANGULAR_P = 0.1;
      final double ANGULAR_D = 0.0;
      PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  
//tommy innit
  @Override
  public void robotInit() {

    m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftStick = new Joystick(0);
  }

 // @Override
  //public void teleopPeriodic() {
   // m_myRobot.tankDrive(-m_leftStick.getY(), -m_leftStick.getY());
    //m_myRobot.tankDrive(-m_leftStick.getTwist(), m_leftStick.getTwist());
  //}
  @Override

    public void teleopPeriodic() {

        double forwardSpeed;

        double rotationSpeed;


        forwardSpeed = -m_leftStick.getY()*.5;


        if (m_leftStick.getTriggerPressed()) {

            // Vision-alignment mode

            // Query the latest result from PhotonVision

            var result = camera.getLatestResult();


            if (result.hasTargets()) {

                // Calculate angular turn power

                // -1.0 required to ensure positive PID controller effort _increases_ yaw

                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

            } else {

                // If we have no targets, stay still.

                rotationSpeed = 0;

            }

        } else {

            // Manual Driver Mode

            rotationSpeed = m_leftStick.getTwist()*.5;

        }


        // Use our forward/turn speeds to control the drivetrain

        m_myRobot.arcadeDrive(forwardSpeed, rotationSpeed);

    }

}

//:3