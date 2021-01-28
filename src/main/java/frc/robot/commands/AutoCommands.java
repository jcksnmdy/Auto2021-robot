// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;

import frc.robot.subsystems.ComplexDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;


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
  // radius in in icnhes
  public void turnWithRadius(double radius, double degrees) {
          double radiansToTurn = 2*Math.PI*(degrees/360); // Converting to radians
          double rightSideArcLen = radiansToTurn*radius; // Find arc length
          double leftSideArcLen = radiansToTurn*(radius - Constants.kRobotWidth);
        
          double rightSpeed = rightSideArcLen/200; // speed
          double leftSpeed = leftSideArcLen/200;

          SmartDashboard.putNumber("Right Speed to Turn", rightSpeed); // Display
          SmartDashboard.putNumber("Left Speed to Turn", leftSpeed);
          SmartDashboard.putNumber("Distance", m_drive.getDistance());

        //   while (m_drive.getDistance() < radiansToTurn){ //distance is encoder value converted to radians
        //     m_drive.setRight(rightSpeed); // go right speed
        //     m_drive.setLeft(leftSpeed); // go left speed
        //   }
          
        //   m_drive.setRight(0);
        //   m_drive.setLeft(0);
  }

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
