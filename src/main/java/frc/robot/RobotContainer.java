// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.drivetrain.JoystickDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final Arm m_arm = new Arm();
  
  private final Intake m_intake = new Intake();

  // The auto selected from a dashboard
  private SendableChooser<Command> m_autoChooser;

  // The field with the robot location
  private Field2d m_Field2d = new Field2d();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Zero the heaeding of the robot at the beginning for field relative drive
    m_robotDrive.zeroHeading();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
    m_robotDrive.setDefaultCommand(new JoystickDrive(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriverControllerDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriverControllerDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriverControllerDeadband),
        m_robotDrive));
    
   m_arm.setDefaultCommand(new RunCommand(() -> m_arm.setSprocketSpeed(
    MathUtil.applyDeadband(-m_operatorController.getLeftY()*0.3, OIConstants.kOperatorControllerDeadband)
   ), m_arm));

   //Buttons to preset positions
    
  m_operatorController.a().whileTrue(new RunCommand(()->m_arm.sprocketToPosition(ArmConstants.SprocketAPosition)));


  m_operatorController.y().whileTrue(new RunCommand(()->m_arm.sprocketToPosition(ArmConstants.SprocketYPosition)));


  m_operatorController.x().whileTrue(new RunCommand(()->m_arm.sprocketToPosition(ArmConstants.SprocketXPosition)));

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_autoChooser = AutoBuilder.buildAutoChooser();
    // Place the sendable chooser data onto the dashboard
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Place the robots location onto the field
    m_Field2d.setRobotPose(m_robotDrive.getPose());
    SmartDashboard.putData("Robot Position", m_Field2d);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Apply slow-down for swerve drive
    // Double Clutch
    m_driverController.leftBumper().whileTrue(
      new JoystickDrive(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriverControllerDeadband) * DriveConstants.kDoubleClutchTranslation,
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriverControllerDeadband) * DriveConstants.kDoubleClutchTranslation,
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriverControllerDeadband) * DriveConstants.kDoubleClutchRotation,
        m_robotDrive)
    );
    // Single Clutch
    m_driverController.rightBumper().whileTrue(
      new JoystickDrive(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriverControllerDeadband) * DriveConstants.kSingleClutchTranslation,
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriverControllerDeadband) * DriveConstants.kSingleClutchTranslation,
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriverControllerDeadband) * DriveConstants.kSingleClutchRotation,
        m_robotDrive)
    );


    

    //Zero Gyro (x + left trigger + right trigger)
    m_driverController.x()
      .and(m_driverController.leftTrigger(OIConstants.kDriverControllerTriggerThreshold))
      .and(m_driverController.rightTrigger(OIConstants.kDriverControllerTriggerThreshold))
      .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    //Intake Control
    m_operatorController.leftTrigger(OIConstants.kOperatorControllerTriggerThreshold).onTrue(
      new InstantCommand(
        () -> m_intake.setMotorSpeed(IntakeConstants.kIntakeMotorSpeed)
      )
    ).onFalse(
      new InstantCommand(
        () -> m_intake.setMotorSpeed(0)
      )
    );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  //This is for updating all elastic info, and is called in robotPeriodic in Robot.java
  public void updateInterface(){
    m_Field2d.setRobotPose(m_robotDrive.getPose());
  }
}
