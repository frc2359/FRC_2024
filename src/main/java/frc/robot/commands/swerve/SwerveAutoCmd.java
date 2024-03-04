package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.navigation.NavigationSubsystem;

public class SwerveAutoCmd {
    
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