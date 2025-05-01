// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopSwervecom;
import frc.robot.subsystems.Swerve.Swerve;


public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void autonomousInit() {
    /*緊急用の呼び出しにつき、後で必ず直すこと。 */
    new TeleopSwervecom(
    robotContainer.getSwerve(),
    () -> 0.0,  // forwardSupplier（使わない）
    () -> 0.0,  // strafeSupplier（使わない）
    () -> 0.0,  // turnSupplier（使わない）
    new Joystick(0) // dummy joystick
    ).schedule();

  }
}