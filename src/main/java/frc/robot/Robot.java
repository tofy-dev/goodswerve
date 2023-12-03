// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final PS4Controller m_controller = new PS4Controller(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final SendableChooser<String> m_modChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> m_driveMotorReversedChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> m_turnMotorReversedChooser = new SendableChooser<>();


  @Override
  public void robotInit() {
    m_modChooser.setDefaultOption("None", "NA");
    m_modChooser.addOption("Front Left", "FL");
    m_modChooser.addOption("Front Right", "FR");
    m_modChooser.addOption("Back Left", "BL");
    m_modChooser.addOption("Back Right", "BR");
    SmartDashboard.putData("modChoices", m_modChooser);

    m_driveMotorReversedChooser.setDefaultOption("No", false);
    m_driveMotorReversedChooser.addOption("Yes", true);
    SmartDashboard.putData("driveMotorReversed", m_driveMotorReversedChooser);

    m_turnMotorReversedChooser.setDefaultOption("No", false);
    m_turnMotorReversedChooser.addOption("Yes", true);
    SmartDashboard.putData("turnMotorReversed", m_turnMotorReversedChooser);
  }
  
  // This is probably extraneous
  // @Override
  // public void disabledInit() {
  //   System.out.println("Disabling...");
  //   disableMovement();
  // }


  // ==========================================================================================================================
  // AUTONOMOUS CODE

  @Override
  public void autonomousInit() {
    m_swerve.reset();
    auto_disableMovement();

    String modSelected = m_modChooser.getSelected();
    boolean driveMotorReversed = m_driveMotorReversedChooser.getSelected();
    boolean turnMotorReversed = m_turnMotorReversedChooser.getSelected();

    SwerveModule swerveMod = m_swerve.getSwerveMod(modSelected);
    swerveMod.setRunnable(true);
    swerveMod.m_driveMotor.setInverted(driveMotorReversed);
    swerveMod.m_turningMotor.setInverted(turnMotorReversed);
  }

  @Override
  public void autonomousPeriodic() {
    m_swerve.updateDashboard();
    m_swerve.drive(0.2, 0, 0, false);
  }


  // ==========================================================================================================================
  // TESTING CODE

  @Override
  public void testInit() {
    autonomousInit();
  }
  @Override
  public void testPeriodic() {
    autonomousPeriodic();
  }


  // ==========================================================================================================================
  // TELEOP CODE

  @Override
  public void teleopInit() {
    m_swerve.reset();
  }

  @Override
  public void teleopPeriodic() {
    // m_swerve.drive(0.01, 0, 0, false);
    // driveWithJoystick(true);
    m_swerve.updateDashboard();
  }


  // ==========================================================================================================================
  // HELPER FUNCTIONS

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because PS4 controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. PS4 controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). PS4 controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public void auto_disableMovement() {
    m_swerve.getSwerveMod("FL")
            .setRunnable(false);
    m_swerve.getSwerveMod("FR")
            .setRunnable(false);
    m_swerve.getSwerveMod("BL")
            .setRunnable(false);
    m_swerve.getSwerveMod("BR")
            .setRunnable(false);
  }
  // ==========================================================================================================================
}