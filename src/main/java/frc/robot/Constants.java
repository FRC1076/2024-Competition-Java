// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 12;
    public static final int kRearLeftTurningMotorPort = 14;
    public static final int kRearRightTurningMotorPort = 13;

    public static final int kFrontLeftTurningEncoderPort = 21;
    public static final int kFrontRightTurningEncoderPort = 22;
    public static final int kRearLeftTurningEncoderPort = 24;
    public static final int kRearRightTurningEncoderPort = 23;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;

    public static final Rotation2d kFrontLeftTurningEncoderOffset = Rotation2d.fromDegrees(77.9);
    public static final Rotation2d kFrontRightTurningEncoderOffset = Rotation2d.fromDegrees(-61.2);
    public static final Rotation2d kRearLeftTurningEncoderOffset = Rotation2d.fromDegrees(125.9);
    public static final Rotation2d kRearRightTurningEncoderOffset = Rotation2d.fromDegrees(113.5); //-316.2

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(19.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27.5);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left Module
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right Module
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Rear Left Module
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //Rear Right Module

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxRotationalSpeedRadiansPerSecond = 2 * Math.PI;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20 * Math.PI;

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.1016; // Four Inch Wheels
    public static final double kDriveRatio = 6.75; // The ratio of a L2 SDS MK4 Module is 6.75 : 1

    // Also accounts for the gear ratio of the Swerve Module, L2 as of writing
    public static final double kDriveEncoderDistancePerRotation = 
        (kWheelDiameterMeters * Math.PI) / kDriveRatio;

    
    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;
    

    public static final double kPModuleTurningController = 0.4;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;
    public static final double kDriverControllerTriggerThreshold = 0.7;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kPTranslationController = 5;
    public static final double kPRotationController = 5;
  }
}
