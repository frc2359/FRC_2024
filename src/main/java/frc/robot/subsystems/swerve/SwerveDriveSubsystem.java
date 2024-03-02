// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.IO2;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.subsystems.swerve.navigation.NavigationSubsystem;

public class SwerveDriveSubsystem extends SubsystemBase {
  static SwerveDriveModule sdmBL = new SwerveDriveModule(DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 0.099);
  
  static SwerveDriveModule sdmBR = new SwerveDriveModule(DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightDriveAbsoluteEncoderPort, -0.251);
  
  static SwerveDriveModule sdmFR = new SwerveDriveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveAbsoluteEncoderPort, -0.013);
  
  static SwerveDriveModule sdmFL = new SwerveDriveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 0.486);

  
  
      public SwerveModulePosition[] getPositions() {
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
      this.phi = phi;
      this.r = r;
    }

    Vec add(Vec a) {
      double x = this.r * Math.cos(this.phi);
      double y = this.r * Math.sin(this.phi);
      x += a.r * Math.cos(a.phi);
      y += a.r * Math.sin(a.phi);
      return new Vec(
          Math.sqrt(x * x + y * y),
          Math.atan2(y, x));
    }
  }

  public void directionalDrive(double speed, double angle, double rotation) {
    // Vec bl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
    // Vec br = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
    // Vec fr = new Vec(speed, angle).add(new Vec(rotation, Math.PI / 4));
    // Vec fl = new Vec(speed, angle).add(new Vec(rotation, -Math.PI / 4));
    Vec bl = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
    Vec br = new Vec(speed, angle).add(new Vec(rotation, 1 * Math.PI / 4));
    Vec fr = new Vec(speed, angle).add(new Vec(rotation, -1 * Math.PI / 4));
    Vec fl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
    sdmBL.directionalDrive(bl.r, bl.phi);
    sdmBR.directionalDrive(br.r, br.phi);
    sdmFR.directionalDrive(fr.r, fr.phi);
    sdmFL.directionalDrive(fl.r, fl.phi);
  }

  //TURN THIS INTO A HASHMAP
  public void setStates(SwerveModuleState stFl, SwerveModuleState stFr, SwerveModuleState stBl, SwerveModuleState stBr) {
    sdmFL.setDesiredState(stFl);
    sdmFR.setDesiredState(stFr);
    sdmBL.setDesiredState(stBl);
    sdmBR.setDesiredState(stBr);
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
    // sdmFR.directionalDrive(speed, Math.PI / 4);
    // sdmBR.directionalDrive(speed, 3 * Math.PI / 4);
    // sdmBL.directionalDrive(speed, -3 * Math.PI / 4);
    // sdmFL.directionalDrive(speed, -Math.PI / 4);

    // sdmFR.directionalDrive(speed, -3 * Math.PI / 4);
    sdmFR.directionalDrive(speed, -1 * Math.PI / 4);
    // sdmBR.directionalDrive(speed, 3 * Math.PI / 4);
    sdmBR.directionalDrive(speed, 1 * Math.PI / 4);
    // sdmBL.directionalDrive(speed, Math.PI / 4);
    sdmBL.directionalDrive(speed, 3 * Math.PI / 4);
    // sdmFL.directionalDrive(speed, -Math.PI / 4);
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

    sdmFR.directionalDrive(kr * speed, frAngle);
    sdmBR.directionalDrive(kr * speed, brAngle);
    sdmBL.directionalDrive(kl * speed, blAngle);
    sdmFL.directionalDrive(kl * speed, flAngle);
  }

  public void rotateTo(double radians) {

  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public static ChassisSpeeds getSpeeds() {
    return NavigationSubsystem.m_kinematics.toChassisSpeeds(sdmFL.getModuleState(), sdmFR.getModuleState(), sdmBL.getModuleState(), sdmBR.getModuleState()); //m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc
  }

  /** Show swerve data */
  public void showData() {
    // SmartDashboard.putNumber("Robot Fused Heading", Gyro.getFusedHeading());
    // SmartDashboard.putNumber("Robot Heading", getCalculatedHeading());
    // SmartDashboard.putString("Robot Location",
    // getPose().getTranslation().toString());
    SmartDashboard.putNumber("FL Abs", sdmFL.getAbsoluteEncoder());
    SmartDashboard.putNumber("FR Abs", sdmFR.getAbsoluteEncoder());
    SmartDashboard.putNumber("BL Abs", sdmBL.getAbsoluteEncoder());
    SmartDashboard.putNumber("BR Abs", sdmBR.getAbsoluteEncoder());
    SmartDashboard.putNumber("FL Rad", sdmFL.getAbsoluteEncoder() * 2 * Math.PI);
    SmartDashboard.putNumber("FR Rad", sdmFR.getAbsoluteEncoder() * 2 * Math.PI);
    SmartDashboard.putNumber("FL turn", sdmFL.getTurningPosition());
    SmartDashboard.putNumber("FR turn", sdmFR.getTurningPosition());
    SmartDashboard.putNumber("BL turn", sdmBL.getTurningPosition());
    SmartDashboard.putNumber("BR turn", sdmBR.getTurningPosition());
    // SmartDashboard.putNumber("FL Sensor Pos", sdmFL.getDrivePosition());
    // SmartDashboard.putNumber("FL Sensor Vel", sdmFL.getDriveVelocity());
  }
}
