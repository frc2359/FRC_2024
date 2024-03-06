package frc.robot;

import frc.robot.commands.swerve.SwerveJoystickCmd;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import static frc.robot.IO.OI;

public class RobotContainer {

    // private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public RobotContainer() {
        // Subsystems
        SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();
        // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        driveSubsystem.setDefaultCommand(
                new SwerveJoystickCmd(
                        driveSubsystem,
                        () -> OI.Driver.getDriveY() * OI.Driver.convToSpeedMult(),
                        () -> -OI.Driver.getDriveX() * OI.Driver.convToSpeedMult(),
                        () -> OI.Driver.getDriveTwist() * OI.Driver.convToSpeedMult(),
                        () -> !OI.Driver.getTrigger()));

    }
}
