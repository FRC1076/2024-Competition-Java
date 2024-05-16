// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//CANCoder is deprecated after 2024, make sure to switch to Phoenix 6 API later
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = // new PIDController(ModuleConstants.kPModuleTurningController, 0, 0);
  
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param turningEncoderChannel The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
     int driveMotorChannel,
     int turningMotorChannel,
     int turningEncoderChannel,
     boolean driveEncoderReversed,
     boolean turningEncoderReversed,
     boolean turningMotorReversed,
     Rotation2d turningEncoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_turningEncoder = new CANCoder(turningEncoderChannel);

    // Set the distance per rotation for the drive encoder.
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerRotation);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderDistancePerRotation);

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setInverted(driveEncoderReversed);
    m_driveMotor.setInverted(driveEncoderReversed);

    m_turningMotor.setInverted(turningMotorReversed);

    // This configuration includes offset, if the cancoder is reversed, and the pulse to rotation conversion
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = ModuleConstants.kTurningEncoderDistancePerPulse;
    config.unitString = "rad";
    config.sensorDirection = turningEncoderReversed;
    config.magnetOffsetDegrees = turningEncoderOffset.getDegrees();
    // Apply the configuration to the cancoder
    m_turningEncoder.configAllSettings(config);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getAbsolutePosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    final double driveOutput;
    if(!isOpenLoop){
      // Calculate the drive output from the drive PID controller.
      driveOutput =
          m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
    }
    else{
      // Divide the drive output by the max speed to scale it from -1 to 1
      driveOutput =
          state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
    }

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    // no need to rezero an absolute encoder
    // m_turningEncoder.reset();
  }
}
