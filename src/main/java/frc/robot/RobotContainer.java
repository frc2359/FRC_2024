package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.OIConstants;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.navigation.NavigationSubsystem;

import static frc.robot.IO.OI;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;


public class RobotContainer {

    //private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer() {
        // Subsystems
        SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();
        //IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        NavigationSubsystem navigationSubsystem = new NavigationSubsystem(driveSubsystem::getPositions);
        driveSubsystem.setDefaultCommand(new SwerveDriveCmd(
                driveSubsystem,
                () -> OI.Driver.getDriveY(),
                () -> -OI.Driver.getDriveX(),
                () -> OI.Driver.getDriveTwist(),
                () -> !OI.Driver.getTrigger(),
                navigationSubsystem));


    /**Returns the current instance of the swerve subsystem */
    //final public SwerveDriveSubsystem getSwerveSubsystem() {
    //    return driveSubsystem;
    //}
    
/**
    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        trajectoryConfig.setKinematics(DriveConstants.Physical.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.Physical.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
    */
}
}
