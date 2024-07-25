// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.limelight.LimelightPoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          DriveConstants.kFrontLeftTurningMotorReversed,
          DriveConstants.kFrontLeftTurningEncoderOffset);
  
  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kFrontRightTurningMotorReversed,
          DriveConstants.kFrontRightTurningEncoderOffset);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          DriveConstants.kRearLeftTurningMotorReversed,
          DriveConstants.kRearLeftTurningEncoderOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          DriveConstants.kRearRightTurningMotorReversed,
          DriveConstants.kRearRightTurningEncoderOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });
  // Pose estimator class, tracks pose using odometry, but also uses data from the limelight to correct errors
  SwerveDrivePoseEstimator m_PoseEstimator = 
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d() //Initial pose
          );
  
  // Initializes limelight pose estimators
  LimelightPoseEstimator limelight1Estimator = new LimelightPoseEstimator(VisionConstants.limelight1,new Transform2d());
  LimelightPoseEstimator limelight2Estimator = new LimelightPoseEstimator(VisionConstants.limelight2,new Transform2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (ChassisSpeeds chassisSpeeds) -> drive(chassisSpeeds, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kPTranslationController, 0.0, 0.0), // Translation PID constants
            new PIDConstants(AutoConstants.kPRotationController, 0.0, 0.0), // Rotation PID constants
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Math.sqrt(
              Math.pow(DriveConstants.kWheelBase / 2, 2) + 
              Math.pow(DriveConstants.kTrackWidth / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
        ),
        () -> {return false;}, // Whether or not the paths should be flipped for being on other teams
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the PoseEstimator in the periodic block
    m_PoseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    // Add limelight1 measurements to pose estimate
    Optional<LimelightPoseEstimator.LimelightPose> limelight1Pose = limelight1Estimator.getPose();
    if(limelight1Pose.isPresent()){
      m_PoseEstimator.addVisionMeasurement(limelight1Pose.get().getPose(), limelight1Pose.get().getTimestamp());
      System.out.println(Constants.VisionConstants.limelight1 + " JSON DUMP AT " + limelight1Pose.get().getTimestamp() + ":");
      System.out.println(LimelightHelpers.getJSONDump(Constants.VisionConstants.limelight1));
    }
    // Add limelight2 measurements to pose estimate
    Optional<LimelightPoseEstimator.LimelightPose> limelight2Pose = limelight2Estimator.getPose();
    if(limelight1Pose.isPresent()){
      m_PoseEstimator.addVisionMeasurement(limelight2Pose.get().getPose(), limelight2Pose.get().getTimestamp());
      System.out.println(Constants.VisionConstants.limelight2 + " JSON DUMP AT " + limelight2Pose.get().getTimestamp() + ":");
      System.out.println(LimelightHelpers.getJSONDump(Constants.VisionConstants.limelight2));
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

  /**
    * Returns the robot's pose, calculated based on data from the Limelight sensors
    * 
    * @return The pose.
    */
  public Pose2d getLimelightPose(String limelightName, boolean isBlueTeam) {
    return isBlueTeam ? LimelightHelpers.getBotPose2d_wpiBlue(limelightName): LimelightHelpers.getBotPose2d_wpiRed(limelightName);
  }

  /**
   * Resets the PoseEstimator to the specified pose.
   *
   * @param pose The pose to which to set the PoseEstimator.
   */
  public void resetPose(Pose2d pose) {
    m_PoseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean useOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0], useOpenLoop);
    m_frontRight.setDesiredState(swerveModuleStates[1], useOpenLoop);
    m_rearLeft.setDesiredState(swerveModuleStates[2], useOpenLoop);
    m_rearRight.setDesiredState(swerveModuleStates[3], useOpenLoop);
  }

  /**
   * Method to drive the robot using robot relative chassis speeds
   *
   * @param chassisSpeeds Robot relative chassis speeds
   */
  public void drive(ChassisSpeeds chassisSpeeds, boolean useOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0], useOpenLoop);
    m_frontRight.setDesiredState(swerveModuleStates[1], useOpenLoop);
    m_rearLeft.setDesiredState(swerveModuleStates[2], useOpenLoop);
    m_rearRight.setDesiredState(swerveModuleStates[3], useOpenLoop);
  }

  /**
   * Method that gets the robot relative chassis speeds
   * 
   */
  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[]{
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState(),
      });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean useOpenloop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], useOpenloop);
    m_frontRight.setDesiredState(desiredStates[1], useOpenloop);
    m_rearLeft.setDesiredState(desiredStates[2], useOpenloop);
    m_rearRight.setDesiredState(desiredStates[3], useOpenloop);
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
