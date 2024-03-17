// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.IO_Subsystem;
import frc.robot.Navigation.*;

public class SwerveDriveCmd extends Command {
  SwerveDriveSubsystem driveSubsystem;
  IO_Subsystem ioSubsystem;
  //Joystick joy;
  NavigationSubsystem navigationSubsystem;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  /** Creates a new DriveCommand. */
  public SwerveDriveCmd(SwerveDriveSubsystem driveSubsystem, 
          IO_Subsystem io,
          NavigationSubsystem navigationSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.ioSubsystem = io;
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
    final double Deadzone = 0.2;  //Constants.ControllerDeadzone;

    double x = xSpeedLimiter.calculate(-ioSubsystem.getDriveY());
    double y = ySpeedLimiter.calculate(ioSubsystem.getDriveX());
    double speed = Math.sqrt(x * x + y * y) * IO_Subsystem.convToSpeedMult(); // Constants.DriveSpeedMultiplier;
    double angle = Math.atan2(y, x);
    double gyroAngle = navigationSubsystem.angleRad();
    if(ioSubsystem.getTrigger()) {gyroAngle = 0;}
    //gyroAngle = 0;  // Temp override of field orientated

    // if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
    //   double rotate = joy.getRightX();
    //   if (Math.abs(rotate) > Deadzone) {
    //     driveSubsystem.rotate(Constants.RotateSpeedMultiplier * rotate);
    //   } else {
    //     driveSubsystem.stop();
    //   }
    // } else {
    //   driveSubsystem.directionalDrive(speed, angle - gyroAngle);
    // }

    double r = rotLimiter.calculate(-ioSubsystem.getDriveTwist());
    //if (Math.abs(r) >= Deadzone) {
    //  driveSubsystem.rotate(.3 * r);
    //} else
    if (Math.abs(x) < Deadzone && Math.abs(y) < Deadzone && Math.abs(r) < Deadzone) {
      driveSubsystem.stop();
    } else {
      SmartDashboard.putNumber("to Angle", angle);
      driveSubsystem.directionalDrive(speed, angle - gyroAngle, IO_Subsystem.convToSpeedMult() * r);    // Constants.RotateSpeedMultiplier * r
      //driveSubsystem.drive(x, y, r, !ioSubsystem.getTrigger(), .02);
    }
    driveSubsystem.showData();
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("GyroAngle", Math.round(navigationSubsystem.angle()*10)/10);
    SmartDashboard.putNumber("GyroAngleRAD", Math.round(gyroAngle*1000)/1000);
    
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
