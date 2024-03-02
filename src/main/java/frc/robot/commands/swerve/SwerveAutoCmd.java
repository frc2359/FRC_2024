package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.navigation.NavigationSubsystem;

public class SwerveAutoCmd {
    public void configure() {
        AutoBuilder.configureHolonomic(
                NavigationSubsystem::getPose, // Robot pose supplier
                NavigationSubsystem::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                SwerveDriveSubsystem::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
}

/**
 * public Command getAutonomousCommand() {
 * // 1. Create trajectory settings
 * TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
 * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
 * DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
 * trajectoryConfig.setKinematics(DriveConstants.Physical.kDriveKinematics);
 * 
 * // 2. Generate trajectory
 * Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
 * new Pose2d(0, 0, new Rotation2d(0)),
 * List.of(
 * new Translation2d(1, 0),
 * new Translation2d(1, -1)),
 * new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
 * trajectoryConfig);
 * 
 * // 3. Define PID controllers for tracking trajectory
 * PIDController xController = new PIDController(AutoConstants.kPXController, 0,
 * 0);
 * PIDController yController = new PIDController(AutoConstants.kPYController, 0,
 * 0);
 * ProfiledPIDController thetaController = new ProfiledPIDController(
 * AutoConstants.kPThetaController, 0, 0,
 * AutoConstants.kThetaControllerConstraints);
 * thetaController.enableContinuousInput(-Math.PI, Math.PI);
 * 
 * // 4. Construct command to follow trajectory
 * SwerveControllerCommand swerveControllerCommand = new
 * SwerveControllerCommand(
 * trajectory,
 * swerveSubsystem::getPose,
 * DriveConstants.Physical.kDriveKinematics,
 * xController,
 * yController,
 * thetaController,
 * swerveSubsystem::setModuleStates,
 * swerveSubsystem);
 * 
 * // 5. Add some init and wrap-up, and return everything
 * return new SequentialCommandGroup(
 * new InstantCommand(() ->
 * swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
 * swerveControllerCommand,
 * new InstantCommand(() -> swerveSubsystem.stopModules()));
 * }
 */