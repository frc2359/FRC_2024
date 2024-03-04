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
import frc.robot.commands.swerve.SwerveDriveCmd;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.navigation.NavigationSubsystem;

import static frc.robot.IO.OI;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

    // private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer() {
        // Subsystems
        SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();
        // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        NavigationSubsystem navigationSubsystem = new NavigationSubsystem(driveSubsystem::getPositions);
        driveSubsystem.setDefaultCommand(
                new SwerveDriveCmd(
                        driveSubsystem,
                        () -> OI.Driver.getDriveY() * OI.Driver.convToSpeedMult(),
                        () -> -OI.Driver.getDriveX() * OI.Driver.convToSpeedMult(),
                        () -> OI.Driver.getDriveTwist() * OI.Driver.convToSpeedMult(),
                        () -> !OI.Driver.getTrigger(),
                        navigationSubsystem));


        /** Returns the current instance of the swerve subsystem */
        // public SwerveDriveSubsystem getSwerveSubsystem() {
        // return driveSubsystem;
        // }

    }
}
