// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.navigation.*;

public class SwerveDriveCmd extends Command {
  SwerveDriveSubsystem driveSubsystem;
  // IO_Subsystem ioSubsystem;
  // Joystick joy;
  NavigationSubsystem navigationSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  /** Creates a new DriveCommand. */
  public SwerveDriveCmd(SwerveDriveSubsystem driveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction,
      NavigationSubsystem navigationSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // this.ioSubsystem = io;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.navigationSubsystem = navigationSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double Deadzone = 0.1; // Constants.ControllerDeadzone;

    double x = -xSpdFunction.get();
    double y = ySpdFunction.get();
    double speed = Math.sqrt(x * x + y * y) * .7; // Constants.DriveSpeedMultiplier;
    double angle = Math.atan2(y, x);
    double gyroAngle = navigationSubsystem.angleRad();
    if (fieldOrientedFunction.get()) {
      gyroAngle = 0;
    }

    // if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
    // double rotate = joy.getRightX();
    // if (Math.abs(rotate) > Deadzone) {
    // driveSubsystem.rotate(Constants.RotateSpeedMultiplier * rotate);
    // } else {
    // driveSubsystem.stop();
    // }
    // } else {
    // driveSubsystem.directionalDrive(speed, angle - gyroAngle);
    // }

    double r = -turningSpdFunction.get();
    // if (Math.abs(r) >= Deadzone) {
    // driveSubsystem.rotate(.3 * r);
    // } else
    if (Math.abs(x) < Deadzone && Math.abs(y) < Deadzone && Math.abs(r) < Deadzone) {
      driveSubsystem.stop();
    } else {
      driveSubsystem.directionalDrive(speed, angle - gyroAngle, .6 * r); // Constants.RotateSpeedMultiplier * r
    }
    driveSubsystem.showData();
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("GyroAngle", navigationSubsystem.angle());
    SmartDashboard.putNumber("GyroAngleRAD", gyroAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
