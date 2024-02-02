package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.Gyro;
import frc.robot.IO.OI;
import frc.robot.IO.Gyro.GyroType;
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

    private final SwerveModule[] modules = new SwerveModule[]{frontRight, frontLeft, backRight, backLeft};

    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
                    DriveConstants.Physical.kDriveKinematics,
                    new Rotation2d(0),
                    new SwerveModulePosition[] {
                        frontRight.getPosition(),
                        frontLeft.getPosition(),
                        backRight.getPosition(),
                        backLeft.getPosition()
                    });

    public SwerveSubsystem() {
        // Configure autonomous
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getSpeeds,
            this::driveRobotRelative,
            AutoConstants.PATH_FOLLOWER_CONFIG,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );


        new Thread(() -> {
            try {
                Thread.sleep(1000);
                Gyro.zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public double getCalculatedHeading() {
        return -1 * Math.IEEEremainder(Gyro.getAngle(GyroType.kNAVX), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getCalculatedHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
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

    /** Periodically runs */
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
        SmartDashboard.putNumber("Robot Fused Heading", Gyro.getFusedHeading());
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

    /** Get "states" of each swerve module. A state is the angle and speed at which the module is moving.
     * 
     * @return states of each module 
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Sets "states" of each swerve module. A state is the angle and speed at which the module is moving. */
    public void setModuleStates (SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, (DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Sets drive mode to be relative to the robot instead of the field. */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /*
    // Not entirely sure what the function below does...
    public void setAutoModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, (DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
        frontLeft.setAutoDesiredState(desiredStates[0]);
        frontRight.setAutoDesiredState(desiredStates[1]);
        backLeft.setAutoDesiredState(desiredStates[2]);
        backRight.setAutoDesiredState(desiredStates[3]);
    }
    */
    
    public double convToSpeedMult() {
        double spdMultiplier = ((OI.Driver.getSpeedDial() + 1) * 0.25) + 0.5;
        SmartDashboard.putNumber("SpeedDriveMult", spdMultiplier);
        return spdMultiplier;
    }


    
}
