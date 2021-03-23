// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;
import java.lang.annotation.Target;

import frc.robot.subsystems.ComplexDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class AutoCommands extends CommandBase {
        @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
        private final ComplexDrivetrain m_drive;
        public double m_rightCommand;
        public double m_leftCommand;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoCommands(ComplexDrivetrain subsystem) {
        m_drive = subsystem;
        m_rightCommand = 0.0;
        m_leftCommand = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }
  public void driveToDistance(double dist, double driveSpeed) {
    m_drive.setRightZero();
    m_drive.setLeftZero();
    Timer.delay(0.2);
    while ((Math.abs(m_drive.getLeftDistance())+Math.abs(m_drive.getRightDistance()))/2 < dist) {
      m_drive.tankDrive(driveSpeed*-1, driveSpeed*-1.03);
      SmartDashboard.putNumber("Average Distance DTD", m_drive.getLeftDistance()+m_drive.getRightDistance()); // Display
      SmartDashboard.putNumber("Adjusted Total Distance To Go", dist-((Math.abs(m_drive.getLeftDistance())+Math.abs(m_drive.getRightDistance()))/2)); // Display

    }
    stopMoving();
  }
  // public void zeroGyro() {
  //   ahrs.zeroYaw();
  // }
  //   public void turnWithGyro(double degrees, double turnSpeed) {
  //   /* Display 6-axis Processed Angle Data                                      */
  //   SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
  //   SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
  //   SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
  //   SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
  //   SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
  // }
  // radius in in feet
  public void turnWithRadius(double radius, double degrees, double turnSpeed) {
          double radiansToTurn = Math.PI*(degrees/180); // Converting to radians
          double rightSideArcLen = radiansToTurn*(radius - Constants.kRobotWidth); // Find arc length
          double leftSideArcLen = radiansToTurn*radius;
        
          double rightTarget = m_drive.getRightDistance()+rightSideArcLen;
          double leftTarget = m_drive.getLeftDistance()+leftSideArcLen;

          SmartDashboard.putNumber("Right Arc Length", rightSideArcLen); // Display
          SmartDashboard.putNumber("Left Arc Length", leftSideArcLen);

          if (radius==1 && leftSideArcLen<0) {
            while (m_drive.getLeftDistance()>leftTarget) { //distance is encoder value converted to radians
              SmartDashboard.putNumber("Current Right Distance", m_drive.getRightDistance());
              SmartDashboard.putNumber("Current Left Distance", m_drive.getLeftDistance());
              SmartDashboard.putNumber("Right Target", rightTarget);
              SmartDashboard.putNumber("Left Target", leftTarget);
              m_drive.tankDrive(turnSpeed, turnSpeed*-1);
            }
            stopMoving();
          } else {
            while (m_drive.getLeftDistance()<leftTarget) { //distance is encoder value converted to radians
              SmartDashboard.putNumber("Current Right Distance", m_drive.getRightDistance());
              SmartDashboard.putNumber("Current Left Distance", m_drive.getLeftDistance());
              SmartDashboard.putNumber("Right Target", rightTarget);
              SmartDashboard.putNumber("Left Target", leftTarget);
              m_drive.tankDrive(turnSpeed*-1, turnSpeed);
            }
            stopMoving();
        }
          
      }

      // radius in in feet
  // public void turnWithRadius(double radius, double degrees, double turnSpeed) {
  //   double radiansToTurn = Math.PI*(degrees/180); // Converting to radians
  //   double rightSideArcLen = radiansToTurn*radius; // Find arc length
  //   double leftSideArcLen = radiansToTurn*(radius - Constants.kRobotWidth);

  //   m_drive.setLeftZero();
  //   m_drive.setRightZero();
  
  //   double rightTarget = m_drive.getRightDistance()+rightSideArcLen;
  //   double leftTarget = m_drive.getLeftDistance()+leftSideArcLen;

  //   SmartDashboard.putNumber("Right Arc Length", rightSideArcLen); // Display
  //   SmartDashboard.putNumber("Left Arc Length", leftSideArcLen);

  //   while (!(m_drive.getLeftDistance()>=leftTarget-0.05&&m_drive.getLeftDistance()<=leftTarget+0.05)) {
  //     m_drive.tankDrive(leftTarget-m_drive.getLeftDistance(), rightTarget-m_drive.getRightDistance());
  //   }

  //   stopMoving();
  // }

  public void stopMoving() {
		m_drive.setLeft(0);
		m_drive.setRight(0);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
