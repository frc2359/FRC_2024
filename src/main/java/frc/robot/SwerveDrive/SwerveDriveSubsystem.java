// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO2.Gyro;
import frc.robot.RobotMap.DriveConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d locFL = new Translation2d(0.381, 0.381);
  private final Translation2d locFR = new Translation2d(0.381, -0.381);
  private final Translation2d locBL = new Translation2d(-0.381, 0.381);
  private final Translation2d locBR = new Translation2d(-0.381, -0.381);

  SwerveModule sdmBL = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset);
  SwerveModule sdmBR = new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          DriveConstants.kBackRightDriveAbsoluteEncoderPort,
          DriveConstants.kBackRightDriveAbsoluteEncoderOffset);
  SwerveModule sdmFR = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffset);
  SwerveModule sdmFL = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset);
  
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          locFL, locFR, locBL, locBR);

  public SwerveModulePosition[] getPositions()
  {
    return new SwerveModulePosition[] {
    sdmFL.getMotorEncoderPosition(), sdmFR.getMotorEncoderPosition(),
    sdmBL.getMotorEncoderPosition(), sdmBR.getMotorEncoderPosition()
  };
  }

  /** Creates a new TestSubsystem. */
  public SwerveDriveSubsystem() {
  }

  @Override
  public void periodic() {
    // System.out.println("BL:" + blMotor.rawPosition());
    // System.out.println("BR:" + brMotor.rawPosition());
    // System.out.println("FL:" + flMotor.rawPosition());
    // System.out.println("FR:" + frMotor.rawPosition());

    }

  public void drive(double speed, double rotate) {
    sdmBL.drive(speed, rotate);
    sdmBR.drive(speed, rotate);
    sdmFR.drive(speed, rotate);
    sdmFL.drive(speed, rotate);
  }


  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Gyro.getRaw().getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    sdmFL.setDesiredState(swerveModuleStates[0]);
    sdmFR.setDesiredState(swerveModuleStates[1]);
    sdmBL.setDesiredState(swerveModuleStates[2]);
    sdmBR.setDesiredState(swerveModuleStates[3]);
  }

  public void directionalDrive(double speed, double angle) {
    sdmBL.directionalDrive(speed, angle);
    sdmBR.directionalDrive(speed, angle);
    sdmFR.directionalDrive(speed, angle);
    sdmFL.directionalDrive(speed, angle);
  }

  static class Vec {
    double phi;
    double r;
    Vec(double r, double phi) {
      this.phi = phi; this.r = r;
    }
    Vec add(Vec a) {
      double x = this.r * Math.cos(this.phi);
      double y = this.r * Math.sin(this.phi);
      x += a.r * Math.cos(a.phi);
      y += a.r * Math.sin(a.phi);
      return new Vec(
        Math.sqrt(x * x + y * y),
        Math.atan2(y, x)
      );
    }
  }

  public void directionalDrive(double speed, double angle, double rotation) {
    //rotation = 0; //temp
   // Vec bl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
   // Vec br = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
   // Vec fr = new Vec(speed, angle).add(new Vec(rotation, Math.PI / 4));
   // Vec fl = new Vec(speed, angle).add(new Vec(rotation, -Math.PI / 4));
    Vec bl = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
    Vec br = new Vec(speed, angle).add(new Vec(rotation, 1 * Math.PI / 4));
    Vec fr = new Vec(speed, angle).add(new Vec(rotation, -1 * Math.PI / 4));
    Vec fl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
   // Vec bl = new Vec(speed, angle).add(new Vec(rotation, 1 * Math.PI / 4));
   // Vec br = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
   // Vec fr = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
   // Vec fl = new Vec(speed, angle).add(new Vec(rotation, -1 * Math.PI / 4));
    sdmBL.directionalDrive(bl.r, bl.phi);
    sdmBR.directionalDrive(br.r, br.phi);
    sdmFR.directionalDrive(fr.r, fr.phi);
    sdmFL.directionalDrive(fl.r, fl.phi);
  }

  public void resetMotors() {
    sdmBL.reset();
    sdmBR.reset();
    sdmFR.reset();
    sdmFL.reset();
  }

  public void stop() {
    sdmBL.stop();
    sdmBR.stop();
    sdmFR.stop();
    sdmFL.stop();
  }

  public void rotate(double speed) {
    //sdmFR.directionalDrive(speed, Math.PI / 4);
    //sdmBR.directionalDrive(speed, 3 * Math.PI / 4);
    //sdmBL.directionalDrive(speed, -3 * Math.PI / 4);
    //sdmFL.directionalDrive(speed, -Math.PI / 4);

    //sdmFR.directionalDrive(speed, -3 * Math.PI / 4);
    sdmFR.directionalDrive(speed, -1 * Math.PI / 4);
    //sdmBR.directionalDrive(speed, 3 * Math.PI / 4);
    sdmBR.directionalDrive(speed,  1 * Math.PI / 4);
    //sdmBL.directionalDrive(speed,  Math.PI / 4);
    sdmBL.directionalDrive(speed,  3 * Math.PI / 4);
    //sdmFL.directionalDrive(speed, -Math.PI / 4);
    sdmFL.directionalDrive(speed, -3 * Math.PI / 4);
  }

  public void carDrive(double rotationFactor, double speed) {
    final double HALF_WHEEL_DISTANCE = DriveConstants.Physical.kTrackWidth / 2;
    double distance = 1 / (rotationFactor + 1e-7);

    speed *= Math.copySign(1, distance);

    double rl = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance - HALF_WHEEL_DISTANCE) * (distance - HALF_WHEEL_DISTANCE));
    double rr = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance + HALF_WHEEL_DISTANCE) * (distance + HALF_WHEEL_DISTANCE));

    double flAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double frAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double blAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);
    double brAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);

    double kl = 1, kr = 1;
    if (distance < 0) {
      kl = rr / rl;
    } else {
      kr = rl / rr;
    }

    sdmFR.directionalDrive(kr*speed, frAngle);
    sdmBR.directionalDrive(kr*speed, brAngle);
    sdmBL.directionalDrive(kl*speed, blAngle);
    sdmFL.directionalDrive(kl*speed, flAngle);
  }

  public void rotateTo(double radians) {
    
  }

  /**Show swerve data */
  public void showData() {
      //SmartDashboard.putNumber("Robot Fused Heading", Gyro.getFusedHeading());
      //SmartDashboard.putNumber("Robot Heading", getCalculatedHeading());
      //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
      SmartDashboard.putNumber("FL Abs", rndNum(sdmFL.getAbsoluteEncoder(),3));
      SmartDashboard.putNumber("FR Abs", rndNum(sdmFR.getAbsoluteEncoder(), 3));
      SmartDashboard.putNumber("BL Abs", rndNum(sdmBL.getAbsoluteEncoder(),3));
      SmartDashboard.putNumber("BR Abs", rndNum(sdmBR.getAbsoluteEncoder(),3));
      SmartDashboard.putNumber("FL Rad", rndNum(sdmFL.getAbsoluteEncoder(),3));
      SmartDashboard.putNumber("FR Rad", rndNum(sdmFR.getAbsoluteEncoder(),3));
      SmartDashboard.putNumber("FR Deg", rndNum(sdmFR.getAbsoluteEncoder(),3));
      SmartDashboard.putNumber("FL turn", sdmFL.getTurningPosition());
      SmartDashboard.putNumber("FR turn", sdmFR.getTurningPosition());
      SmartDashboard.putNumber("BL turn", sdmBL.getTurningPosition());
      SmartDashboard.putNumber("BR turn", sdmBR.getTurningPosition());
      //SmartDashboard.putNumber("FL Sensor Pos", sdmFL.getDrivePosition());
      //SmartDashboard.putNumber("FL Sensor Vel", sdmFL.getDriveVelocity());
  }

  private double rndNum (double inNum, int place) {
    double mult = 1;
    for (int i=0; i<place; i++) {
      mult *= 10;
    }
    return Math.round((inNum * mult )) / mult;
  }
}
