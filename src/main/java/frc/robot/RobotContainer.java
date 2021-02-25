// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.ComplexDrivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveLimeLight;
import frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.lang.invoke.VarHandle;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Subsystems
	public static ComplexDrivetrain drivetrain = new ComplexDrivetrain();
	public static Shooting shooter = new Shooting();
	public static Index index = new Index();
	public static Intake intake = new Intake();
	private DigitalInput m_limitSwitch = new DigitalInput(1);
	private boolean limelightOn = false;

	private boolean intakeOn = false;
	// Controllers
	public static Joystick joystick = new Joystick(0);
	public static JoystickButton circle = new JoystickButton(joystick, 3);
	public static JoystickButton leftTrigger = new JoystickButton(joystick, 7);
	public static JoystickButton rightTrigger = new JoystickButton(joystick, 8);
	public static JoystickButton xbutton = new JoystickButton(joystick, 2);
	public static JoystickButton slowTurn = new JoystickButton(joystick, 12);
	// public static DriveToDistance driveToDistance = new DriveToDistance(10,
	// drivetrain);
	public static DriveLimeLight driveLimelight = new DriveLimeLight(drivetrain);
	public static AutoCommands autoCommands = new AutoCommands(drivetrain);

	public static double feetFromEncoder = 0.0;

  private RunCommand m_curvatureDrive = new RunCommand(() -> {
		if (slowTurn.get()) {
			drivetrain.curvatureDrive(((joystick.getRawAxis(1)) / 3), ((joystick.getRawAxis(2)) / 3));
		} else {
			drivetrain.curvatureDrive(((joystick.getRawAxis(1)) / 1.5), ((joystick.getRawAxis(2) / 2.5)));
		}
  }, drivetrain);
  
  private RunCommand runIndexer = new RunCommand(() -> {
		rightTrigger.whenReleased(new InstantCommand(() -> index.stopIndexer(), index));
		if (circle.get()) {
			index.reverse();
		}
		if (rightTrigger.get()) {
			index.runIndexer();
		} else if (m_limitSwitch.get()) {
			index.stopIndexer();
		}
  }, index);
  
  private RunCommand runShooter = new RunCommand(() -> {
		if (leftTrigger.get()) {
			shooter.shootLimeLight();
		} else if (xbutton.get()) {
			shooter.shoot(-0.75);
		} else {
			shooter.stopShooting();
		}
	}, shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick, 4).whenPressed(new InstantCommand(() -> {
			if (intakeOn == true) {
				intake.stopIntake();
				index.stopIndexer();
				intakeOn = false;
			} else {
				intake.runIntake();
				// index.runIndexer();
				intakeOn = true;
			}

    }, intake, index));
    
		// X (on ps4) -
		new JoystickButton(joystick, 1).toggleWhenPressed(new RunCommand(() -> {
			if (limelightOn == false){
				limelightOn = true;
				driveLimelight.execute();
			} else {
				limelightOn = false;
				driveLimelight.stopLimeLight();
			}
		}, drivetrain));

		new JoystickButton(joystick, 12).whenPressed(new RunCommand(() -> {                         // Radius, degrees
			autoCommands.turnWithRadius(65, 170);
		}, drivetrain));

		new JoystickButton(joystick, 12).whenReleased(new RunCommand(() -> {    
			autoCommands.stopMoving();
		}, drivetrain));
		
		// Togle lights - BUTTON->
		new JoystickButton(joystick, 13).whenPressed(new InstantCommand(() -> {
			NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry ledMode = m_limelight.getEntry("ledMode");
			if (ledMode.getDouble(0) == 0) {
				ledMode.setDouble(1); // Force OFF
			} else if (ledMode.getDouble(0) == 1) {
				ledMode.setDouble(3); // Force ON
			} else if (ledMode.getDouble(0) == 3) {
				ledMode.setDouble(1); // Force OFF
			} else {
				ledMode.setDouble(1); // Force OFF by default
			}
		}));
  }

  private void configureDefaultCommands() {
	drivetrain.setDefaultCommand(m_curvatureDrive);
	// colorsensor.setDefaultCommand(colorSensor);
	shooter.setDefaultCommand(runShooter);
	// intake.setDefaultCommand(runBoth);
	// driveLimelight.setDefaultCommand(runLimelight);
	index.setDefaultCommand(runIndexer);
	}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
// 	  // Create a voltage constraint to ensure we don't accelerate too fast
// 	  var autoVoltageConstraint =
// 	  new DifferentialDriveVoltageConstraint(
// 	      new SimpleMotorFeedforward(
// 		  0.22,
// 		  0.001,
// 		  0.01),
// 		  Constants.Drivetrain.mkinematics,
// 	      1);
  
//       // Create config for trajectory
//       TrajectoryConfig config =
// 	  new TrajectoryConfig(
// 		  AutoConstants.kMaxSpeedMetersPerSecond,
// 		  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
// 	      // Add kinematics to ensure max speed is actually obeyed
// 	      .setKinematics(Constants.Drivetrain.mkinematics)
// 	      // Apply the voltage constraint
// 	      .addConstraint(autoVoltageConstraint);
  
//       // An example trajectory to follow.  All units in meters.
//       Trajectory exampleTrajectory =
// 	  TrajectoryGenerator.generateTrajectory(
// 	      // Start at the origin facing the +X direction
// 	      new Pose2d(0, 0, new Rotation2d(270)),
// 	      // Pass through these two interior waypoints, making an 's' curve path
// 	      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
// 	      // End 3 meters straight ahead of where we started, facing forward
// 	      new Pose2d(3, 0, new Rotation2d(270)),
// 	      // Pass config
// 	      config);
  
//       RamseteCommand ramseteCommand =
// 	  new RamseteCommand(
// 	      exampleTrajectory,
// 	      drivetrain::getPose,
// 	      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
// 	      new SimpleMotorFeedforward(
// 		  0.22,
// 		  0.001,
// 		  0.01),
// 		Constants.Drivetrain.mkinematics,
// 	      drivetrain::getWheelSpeeds,
// 	      new PIDController(
// 		      Constants.Drivetrain.kPIDConfig.P,
// 		      0,
// 		      0),
// 	      new PIDController(
// 		      Constants.Drivetrain.kPIDConfig.P,
// 		      0,
// 		      0),
// 	      // RamseteCommand passes volts to the callback
// 	      drivetrain::tankDrive,
// 	      drivetrain);
  
//       // Reset odometry to the starting pose of the trajectory.
//       drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
  
//       // Run path following command, then stop at the end.
//       return ramseteCommand.andThen(() -> drivetrain.curvatureDrive(0.0, 0.0));
    // An ExampleCommand will run in autonomous
    return new RunCommand(() -> {
		System.out.println("Straight");
		autoCommands.driveToDistance(6.34);
		// System.out.println("Turn Right");
		// autoCommands.turnWithRadius(1, 45);
	}, drivetrain).withTimeout(5);
    /*
	boolean var = true;
	if (var == true) {
		return new SequentialCommandGroup(new RunCommand(() -> {
			System.out.println("Straight");
				autoCommands.driveToDistance(6.34);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Turn Right");
				autoCommands.turnWithRadius(1, 45);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.driveToDistance(5.6);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, -90);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.driveToDistance(7.9);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, 45);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.driveToDistance(13.83);
			}, drivetrain).withTimeout(4));
	} else {
		return new SequentialCommandGroup(new RunCommand(() -> {
			System.out.println("Straight");
				autoCommands.driveToDistance(8.84);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, -73);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.driveToDistance(7.9);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, 90);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.driveToDistance(11);
			}, drivetrain).withTimeout(4));
	}	
*/

/*

	boolean var2 = true;

	if (var2 == true) {
		return new SequentialCommandGroup(new RunCommand(() -> {
				System.out.println("Turn Left");
				autoCommands.turnWithRadius(1, -45);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Straight");
				autoCommands.driveToDistance(7);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, 90);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Straight");
				autoCommands.driveToDistance(7);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, -90);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Straight");
				autoCommands.driveToDistance(7);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, 45);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Straight");
				autoCommands.driveToDistance(10);
			}, drivetrain).withTimeout(4));
	} else {
		return new SequentialCommandGroup(new RunCommand(() -> {
				System.out.println("Turn Left");
				autoCommands.turnWithRadius(1, -30);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Straight");
				autoCommands.driveToDistance(14);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, -10);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				System.out.println("Straight");
				autoCommands.driveToDistance(7);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.turnWithRadius(1, 90);
			}, drivetrain).withTimeout(4), new RunCommand(() -> {
				autoCommands.driveToDistance(12);
			}, drivetrain).withTimeout(4));
		}

*/

//     return new SequentialCommandGroup(new ParallelCommandGroup(new RunCommand(() -> {
// 			shooter.shoot(-0.54);
// 		}, shooter), new RunCommand(() -> {
// 			index.runIndexer();
// 		}, index)).withTimeout(10), new RunCommand(() -> {
// 			drivetrain.curvatureDrive(-0.25, 0);
//     }, drivetrain).withTimeout(1)).withTimeout(15);
  }
}
