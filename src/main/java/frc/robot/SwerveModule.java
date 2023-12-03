// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;


public class SwerveModule {
  private static final double kWheelRadius = 0.0508; // 2in
  private static final double kDriveGearRatio = 6.75;
  private static final double kTurningGearRatio = 150.0/7;

  private static final double kModuleMaxAngularVelocity = 1;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final String m_name;
  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final AnalogInput m_absoluteEncoder;
  private final double m_absoluteOffset;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.5,
          0,
          0.001,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));


  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private boolean a_isRunnable = true;


  public SwerveModule(String name, int driveMotorID, int turnMotorID, int absoluteEncoderPort,
                      double absoluteEncoderOffset) {
    m_name = name;

    m_driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turnMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius);
    m_driveEncoder.setVelocityConversionFactor(2 * Math.PI / 60 / kDriveGearRatio * kWheelRadius); // meters per second

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningMotor.setInverted(true); // TESTING FRONT LEFT
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI / kTurningGearRatio); // degrees (divide by gear ratio)

    m_absoluteEncoder = new AnalogInput(absoluteEncoderPort);
    m_absoluteOffset = absoluteEncoderOffset;

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the position of the absolute encoder.
   * @param truncate Whether the function should truncate the output.
   * @return The position of the encoder, in radians
   */
  public double getAbsoluteEncoderRad(boolean truncate) {
    double angle = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2 * Math.PI;
    angle -= m_absoluteOffset;
    if (truncate)
      return (int)(angle*100)/100.0;
    return angle;
  }
  public double getAbsoluteEncoderRad() {
    return getAbsoluteEncoderRad(true);
  }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Quit if the swerve module is not able to be run
    if (!a_isRunnable) {
      // the SETS are PROBABLY extraneous
      // m_driveMotor.set(0);
      // m_turningMotor.set(0);
      return;
    }

    // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState state = desiredState;
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    // m_driveMotor.setVoltage(driveOutput);
    // m_turningMotor.setVoltage(turnOutput);
  }

  public void setRunnable(boolean isRunnable) {
    a_isRunnable = isRunnable;
  }

  public boolean getRunnable() {
    return a_isRunnable;
  }

  public void reset() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);

    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public void updateDashboard(String name) {
    SmartDashboard.putNumber(name + " DriveEncoder Pos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(name + " TurningEncoder Pos", m_turningEncoder.getPosition());

    SmartDashboard.putNumber(name + " DriveEncoder Vel", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(name + " TurningEncoder Vel", m_turningEncoder.getVelocity());

    SmartDashboard.putNumber(name + " AbsoluteEncoder Pos", getAbsoluteEncoderRad());
  }
}