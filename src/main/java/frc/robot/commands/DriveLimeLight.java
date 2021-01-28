// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ComplexDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class DriveLimeLight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ComplexDrivetrain m_drive;
	private final NetworkTable m_limelight;
	private NetworkTableEntry m_pipeline;
	public double m_rightCommand;
	public double m_leftCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveLimeLight(ComplexDrivetrain subsystem) {
    m_drive = subsystem;
		m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
		m_pipeline = m_limelight.getEntry("pipeline");
		m_pipeline.setNumber(2);
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
    double tx = m_limelight.getEntry("tx").getDouble(0);
    double ty = m_limelight.getEntry("ty").getDouble(0);

    double aimAdjust = tx/3 * Constants.Limelight.kPAngle;
    double driveAdjust = tx/3 * Constants.Limelight.kPDistance;
		// Still needs to get target area
		m_rightCommand = aimAdjust*-1; 
		m_leftCommand =  aimAdjust*-1;

		SmartDashboard.putNumber("Right", m_leftCommand);
		SmartDashboard.putNumber("Left", m_rightCommand);

		m_drive.setLeft(m_leftCommand);
		m_drive.setRight(m_rightCommand);
  }

  public void stopLimeLight() {
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
