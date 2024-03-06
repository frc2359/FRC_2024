// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.navigation;

import frc.robot.IO;
import frc.robot.IO.Gyro.GyroType;
import frc.robot.RobotMap.DriveConstants;

import java.util.HashMap;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NavigationSubsystem extends SubsystemBase {
  public AHRS gyro = IO.Gyro.getNavX();

  public double angle;
  // Locations for the swerve drive modules relative to the robot center.
  static Translation2d m_frontLeftLoc = new Translation2d(DriveConstants.Physical.kWheelBase / 2,
      DriveConstants.Physical.kTrackWidth / 2);
  static Translation2d m_frontRightLoc = new Translation2d(DriveConstants.Physical.kWheelBase / 2,
      -DriveConstants.Physical.kTrackWidth / 2);
  static Translation2d m_backLeftLoc = new Translation2d(-DriveConstants.Physical.kWheelBase / 2,
      DriveConstants.Physical.kTrackWidth / 2);
  static Translation2d m_backRightLoc = new Translation2d(-DriveConstants.Physical.kWheelBase / 2,
      -DriveConstants.Physical.kTrackWidth / 2);

  // Creating my kinematics object using the module locations
  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);

  double pitch;
  private static Supplier<SwerveModulePosition[]> modulePositions;

  public double x;
  public double y;
  public double z;
  public double flx;
  public double fly;
  double[] fl = { flx, fly };
  public double frx;
  public double fry;
  double[] fr = { frx, fry };
  public double blx;
  public double bly;
  double[] bl = { blx, bly };
  public double brx;
  public double bry;
  double[] br = { brx, bry };

  double sflx;
  double sfly;
  double sfrx;
  double sfry;
  double sblx;
  double sbly;
  double sbrx;
  double sbry;

  public double fla;
  public double fra;
  public double bla;
  public double bra;

  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;

  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem(Supplier<SwerveModulePosition[]> modulePositions) {
    NavigationSubsystem.modulePositions = modulePositions;
    Shuffleboard.getTab("Navigation").add(gyro);

    /*
     * Shuffleboard.getTab("Navigation").addDoubleArray("position", () -> {
     * return new double[] {pose.getX(), pose.getY(),
     * pose.getRotation().getDegrees()};
     * });
     */
    Shuffleboard.getTab("Swerve Coordinates");

    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Left", () -> {
      return new double[] { sflx, sfly };
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Right", () -> {
      return new double[] { sfrx, sfry };
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Left", () -> {
      return new double[] { sblx, sbly };
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Right", () -> {
      return new double[] { sbrx, sbry };
    });

    odometry = new SwerveDriveOdometry(
        m_kinematics, IO.Gyro.getRotation2D(), modulePositions.get(), new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public double angle() {
    return this.angle;
  }

  public static void updateOdometry(HashMap<String, SwerveModulePosition> positions) {
    SwerveModulePosition posFl = positions.get("front_left");
    SwerveModulePosition posFr = positions.get("front_right");
    SwerveModulePosition posBl = positions.get("back_left");
    SwerveModulePosition posBr = positions.get("back_right");

    odometry.update(getRotation2d(), new SwerveModulePosition[] {posFl, posFr, posBl, posBr});
  }

  public double angleRad() { // navx reads clockwise as positive, in degrees, continuous from 0 to 360
    double angleRad = -IO.Gyro.getAngle();
    if (angleRad > 180) {
      angleRad -= 360;
    }
    if (angleRad < -180) {
      angleRad += 360;
    }
    return angleRad / 180 * Math.PI;
  }

  /** Gets the rotation of the robot in range (0, 360) and returns as a Rotation2d type so the robot can understand
   * @return Rotation2d representation of the robot rotation
   */
  public static Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(IO.Gyro.getAngle(), 360));
  }

  public static Pose2d getPose() {
    return pose;
  }

  public static void resetPose(Pose2d pose) {
    pose = new Pose2d(0.0, 0.0, new Rotation2d());
    odometry.resetPosition(IO.Gyro.getRotation2D(), modulePositions.get(), pose);
  }

  public void resetOrientation() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    angle = angleRad();
    pitch = IO.Gyro.getPitch();
    // pose = odometry.update(gyro.getRotation2d(), modulePositions);
    // x = IO.Gyro.getDisplacementX();
    // y = IO.Gyro.getDisplacementY();
    // z = IO.Gyro.getDisplacementZ();

    SwerveModulePosition[] positions = modulePositions.get();

    SwerveModulePosition fl = positions[0];
    double fld = fl.distanceMeters;
    fla = fl.angle.getRadians();
    SwerveModulePosition fr = positions[1];
    double frd = fr.distanceMeters;
    fra = fr.angle.getRadians();
    SwerveModulePosition bl = positions[2];
    double bld = bl.distanceMeters;
    bla = bl.angle.getRadians();
    SwerveModulePosition br = positions[3];
    double brd = br.distanceMeters;
    bra = br.angle.getRadians();

    flx = fld * Math.cos(fla);
    fly = fld * Math.sin(fla);
    frx = frd * Math.cos(fra);
    fry = frd * Math.sin(fra);
    blx = bld * Math.cos(bla);
    bly = bld * Math.sin(bla);
    brx = brd * Math.cos(bra);
    bry = brd * Math.sin(bra);

    sflx += flx;
    sfly += fly;
    sfrx += frx;
    sfry += fry;
    sblx += blx;
    sbly += bly;
    sbrx += frx;
    sbry += fry;
  }

  public Double[][] getCoordinates() {
    // Returns 4 1D arrays, each representing the x and y of a module. Index 0, 1,
    // 2, 3 represent
    // the front left, front right, back left, and back right modules respectively.
    Double[][] coordinates = {
        { sflx, sfly },
        { sfrx, sfry },
        { sblx, sbly },
        { sbrx, sbry }
    };
    return coordinates;
  }

}
