package frc.robot.subsystems;

import java.lang.Math;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {

    private final DutyCycleOut request;

    private final TalonFX driveMotor; // Use Falcon500 for drive
    private final CANSparkMax turningMotor; // Use NEO for turning

    private final RelativeEncoder turningEncoder; // Use SparkMax encoder for turning

    private final PIDController turningPidController;
    private final PIDController drivingPidController;

    private final CANcoder absoluteEncoder; // Use CANCoder for absolute position
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    /** Represents a single swerve module that contains two motors (drive and spin) and a wheel */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId, "rio");
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor
                .setNeutralMode(DriveConstants.INI_BRAKE_MODE_DRIVE ? NeutralModeValue.Brake : NeutralModeValue.Coast);

        request = new DutyCycleOut(0);

        // driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        drivingPidController = new PIDController(ModuleConstants.kPDriving, 0, 0);
        drivingPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public boolean setDriveMode(boolean isBrakeMode) {
        driveMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        turningMotor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        return isBrakeMode;
    }

    public IdleMode getDriveMode() {
        return turningMotor.getIdleMode();
    }

    /**
     * Set whether the drive motor is inverted
     * 
     * @param inv is true when you want to set it to inverted
     */
    public void invertDrive(boolean inv) {
        driveMotor.setInverted(inv);
    }

    public double getDrivePosition() {
        var rotorPosSignal = driveMotor.getRotorPosition().getValue();
        return ModuleConstants.kDriveEncoderRot2Meter * (rotorPosSignal / 2048);
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDriveVelocity() {
        double driveVelRpS = (driveMotor.getRotorVelocity().getValue() * 10) / 2048;
        double driveVelMpS = driveVelRpS * ModuleConstants.kDriveEncoderRot2Meter;
        return driveVelMpS;
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {  // -.5 to +.5
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // No offset
    }

    public double getAbsoluteEncoderRad() {
        double angle = Math.PI*2*(getAbsoluteEncoder() - absoluteEncoderOffset);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public void hardResetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        // SET MOTORS
        // ----------------------------------------------------------------------

        // percent out control
        driveMotor.setControl(request.withOutput(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

        // positon control
        /* 2048 ticks/rev * 10 Rotations in either direction */
        // double convToSensorCounts = (state.speedMetersPerSecond * (2048 /
        // (ModuleConstants.kWheelDiameterMeters * Math.PI))) / 8.14;
        // convToSensorCounts = convToSensorCounts >
        // DriveConstants.kPhysicalMaxSpeedMetersPerSecond ?
        // DriveConstants.kPhysicalMaxSpeedMetersPerSecond : convToSensorCounts;
        // driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond);

        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void setAutoDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        // SET MOTORS ----------------------------------------------------------------------

        // percent out control
        driveMotor.setControl(request.withOutput(state.speedMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.setControl(request.withOutput(0));
        turningMotor.set(0);
    }
}