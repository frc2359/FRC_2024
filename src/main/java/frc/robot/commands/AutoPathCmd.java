package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.IO;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPathCmd extends SequentialCommandGroup {
    private final String kinematics = null;

    SwerveSubsystem swerveSubsystem;

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(SwerveSubsystem swerveSubsystem, PathPlannerPath path, boolean isFirstPath) {
        this.swerveSubsystem = swerveSubsystem;
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                swerveSubsystem.resetOdometry(path.getInitialTargetHolonomicPose());
            }
            }),
            new FollowPathHolonomic(
                path, 
                swerveSubsystem::getPose, // Pose supplier
                swerveSubsystem::setAutoModuleStates, // Module states consumer
                DriveConstants.Physical.kDriveKinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerveSubsystem // Requires this drive subsystem
            )
        );
    }
}
