package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPathCmd extends SequentialCommandGroup {

    SwerveSubsystem swerveSubsystem;

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(SwerveSubsystem swerveSubsystem, PathPlannerPath path, boolean isFirstPath) {
        Command c = new Command() {
            
        };
        return c;
    }
}
