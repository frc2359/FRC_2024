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
        Command c = new Command() {
            
        };
        return c;
    }
}
