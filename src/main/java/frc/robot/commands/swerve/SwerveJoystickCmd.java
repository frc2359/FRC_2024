package frc.robot.commands.swerve;

import frc.robot.IO;
import frc.robot.RobotMap;
import frc.robot.RobotMap.DriveConstants;

import java.util.HashMap;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.navigation.NavigationSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveDriveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    // private final ADXRS450_Gyro gyroNew = new ADXRS450_Gyro();
    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // double pitchAngleDegrees    = gyro.getPitch();
    // double rollAngleDegrees     = gyro.getRoll();


    public SwerveJoystickCmd(SwerveDriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(RobotMap.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(RobotMap.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(RobotMap.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > RobotMap.OIConstants.kDriverDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > RobotMap.OIConstants.kDriverDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > RobotMap.OIConstants.kDriverDeadband ? turningSpeed : 0.0;

        SmartDashboard.putNumber("xSpeed = ", xSpeed);
        SmartDashboard.putNumber("ySpeed = ", ySpeed);
        SmartDashboard.putNumber("Turning Speed = ", turningSpeed);

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, NavigationSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
     
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = NavigationSubsystem.m_kinematics.toSwerveModuleStates(chassisSpeeds);

        // states are created based on the order of their initialization from m_kinematics (fL, fR, bL, bR)
        // convert to Hash for easier readability
        HashMap<String, SwerveModuleState> states = new HashMap<>();
        states.put("front_left", moduleStates[0]);
        states.put("front_right", moduleStates[1]);
        states.put("back_left", moduleStates[2]);
        states.put("back_right", moduleStates[3]);


        // 6. Output each module states to wheels
        SwerveDriveSubsystem.setStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}