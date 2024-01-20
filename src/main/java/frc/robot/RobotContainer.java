package frc.robot;

import java.util.HashMap;
import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoPathCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final SlewRateLimiter xLimiter, yLimiter;

//     private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -IO.getDriveY() * swerveSubsystem.convToSpeedMult(),
                () -> -IO.getDriveX() * swerveSubsystem.convToSpeedMult(),
                () -> -IO.getDriveTwist() * swerveSubsystem.convToSpeedMult(),
                () -> !IO.getTrigger()));

        this.xLimiter = new SlewRateLimiter(RobotMap.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(RobotMap.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        //configureButtonBindings();
    }

    /**Returns the current instance of the swerve subsystem */
    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    /**Runs inputted path from the helper application "Path Planner (from Microsoft Store)
     * @param pathName is the name of the path set in PathPlanner
     * @param maxV is the velocity on the path
     * @param maxAccel is the acceleration of the path
     * @return the command for the path the follow */
    public Command runPath(String pathName, int maxV, int maxAccel) {
        final AutoPathCmd command = new AutoPathCmd();
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(pathName, new PathConstraints(maxV, maxAccel));
        return command.followTrajectoryCommand(swerveSubsystem, examplePath, true);
    }

    /**Runs inputted path (that includes events) from the helper application "Path Planner (from Microsoft Store)
     * @param pathName is the name of the path set in PathPlanner
     * @param maxV is the velocity on the path
     * @param maxAccel is the acceleration of the path
     * @param events is a Hash Map of events that are to be run (in the format ("Key", event()))
     * @return the command for the path the follow */
    public Command runPathWithEvents(String pathName, int maxV, int maxAccel, HashMap<String, Command> events) {
        // final AutoPathCmd command = new AutoPathCmd();

        PathPlannerTrajectory examplePath = PathPlanner.loadPath(pathName, new PathConstraints(maxV, maxAccel));
        
        FollowPathWithEvents command = new FollowPathWithEvents(
                runPath(pathName, maxV, maxAccel), examplePath.getMarkers(), events
                );

        return command;
        // return command.followTrajectoryCommand(swerveSubsystem, examplePath, true);
    }
    

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

    public void balance() {
        double pitchAngleRadians = IO.getPitch() * (Math.PI / 180.0);
        // double yAxisRate = Math.sin(pitchAngleRadians);
        double yAxisRate = 0;

        
        double rollAngleRadians = IO.getRoll() * (Math.PI / 180.0);
        double xAxisRate = Math.sin(rollAngleRadians);

        // 2. Apply deadband
        // xAxisRate = Math.abs(xAxisRate) > RobotMap.OIConstants.kDriverDeadband ? xAxisRate : 0.0;
        // yAxisRate = Math.abs(yAxisRate) > RobotMap.OIConstants.kDriverDeadband ? yAxisRate : 0.0;

        // 3. Make the driving smoother
        xAxisRate = xLimiter.calculate(xAxisRate) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * 0.9;
        yAxisRate = yLimiter.calculate(yAxisRate) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * 0.8;

        SmartDashboard.putNumber("xRate", xAxisRate);
        SmartDashboard.putNumber("yRate", yAxisRate);


        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xAxisRate, yAxisRate, 0);
         
      
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.Physical.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

}
