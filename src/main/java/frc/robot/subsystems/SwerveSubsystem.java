package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.IO.GyroType;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    
    public SwerveDriveKinematics kinematics;

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
                    DriveConstants.Physical.kDriveKinematics,
                    new Rotation2d(0),
                    new SwerveModulePosition[] {
                        frontRight.getPosition(),
                        frontLeft.getPosition(),
                        backRight.getPosition(),
                        backLeft.getPosition()}
                    );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                IO.zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public double getCalculatedHeading() {
        return -1 * Math.IEEEremainder(IO.getAngle(GyroType.kNAVX), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getCalculatedHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),
                                new SwerveModulePosition[] {
                                    frontRight.getPosition(),
                                    frontLeft.getPosition(),
                                    backRight.getPosition(),
                                    backLeft.getPosition()},
                                pose);
    }

    @Override
    public void periodic() {
        backLeft.invertDrive(true);
        odometer.update(getRotation2d(),
                        new SwerveModulePosition[] {
                            frontRight.getPosition(),
                            frontLeft.getPosition(),
                            backRight.getPosition(),
                            backLeft.getPosition()});
        
        
        
        showData();
    }

    /**Set drive mode between brake and coast */
    public boolean setDriveMode(boolean isBrakeMode) {
        frontRight.setDriveMode(isBrakeMode);
        frontLeft.setDriveMode(isBrakeMode);
        backRight.setDriveMode(isBrakeMode);
        backLeft.setDriveMode(isBrakeMode);
        
        SmartDashboard.putBoolean("SwerveBrakeMode?", isBrakeMode);

        DriveConstants.currentBrakeMode = isBrakeMode;

        return isBrakeMode;
    }
    /**Show swerve data */
    public void showData() {
        SmartDashboard.putNumber("Robot Fused Heading", IO.getFusedHeading());
        SmartDashboard.putNumber("Robot Heading", getCalculatedHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("FL Abs", frontLeft.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("FR Abs", frontRight.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("BL Abs", backLeft.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("BR Abs", backRight.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("FL Rad", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FR Rad", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BL Rad", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BR Rad", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FL turn", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FR turn", frontRight.getTurningPosition());
        SmartDashboard.putNumber("BL turn", backLeft.getTurningPosition());
        SmartDashboard.putNumber("BR turn", backRight.getTurningPosition());
        SmartDashboard.putNumber("FL Sensor Pos", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("FL Sensor Vel", frontLeft.getDriveVelocity());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates
    (SwerveModuleState[] desiredStates) {
        
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, (DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setAutoModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, (DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        frontLeft.setAutoDesiredState(desiredStates[0]);
        frontRight.setAutoDesiredState(desiredStates[1]);
        backLeft.setAutoDesiredState(desiredStates[2]);
        backRight.setAutoDesiredState(desiredStates[3]);
    }
    
    public double convToSpeedMult() {
        double spdMultiplier = ((IO.getSpeedDial() + 1) * 0.25) + 0.5;
        SmartDashboard.putNumber("SpeedDriveMult", spdMultiplier);
        return spdMultiplier;
    }
}
