package frc.robot.subsystems;

import static frc.robot.RobotMap.*;

import java.lang.Math;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {

    private final TalonFX driveMotor;               // Use Falcon500 for drive
    private final CANSparkMax turningMotor;         // Use NEO for turning

    //private final CANEncoder driveEncoder;        // Use encoder in Falcon500 for drive
    private final RelativeEncoder turningEncoder;   // Use SparkMax encoder for turning

    private final PIDController turningPidController;
    private final PIDController drivingPidController;

    private final CANcoder absoluteEncoder;         // Use CANCoder for absolute position
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private boolean isBrakeMode;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId, "rio");
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.setNeutralMode(DriveConstants.INI_BRAKE_MODE_DRIVE ? NeutralMode.Brake : NeutralMode.Coast);

        //driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        drivingPidController = new PIDController(ModuleConstants.kPDriving, 0, 0);
        drivingPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        // iniVelcotyMode();
    }

    /**Initialize velocity mode */
    public void iniVelcotyMode() {
        /* Factory default hardware to prevent unexpected behavior */
		driveMotor.getConfigurator().apply(new TalonFXConfiguration());

		/* Configure Sensor Source for Pirmary PID */
        
		driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, AutoConstants.kPIDLoopIdx,
        AutoConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		driveMotor.configNeutralDeadband(0.01, AutoConstants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		driveMotor.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // driveMotor.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, AutoConstants.kTimeoutMs);
		driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, AutoConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		driveMotor.configNominalOutputForward(0, AutoConstants.kTimeoutMs);
		driveMotor.configNominalOutputReverse(0, AutoConstants.kTimeoutMs);
		driveMotor.configPeakOutputForward(1, AutoConstants.kTimeoutMs);
		driveMotor.configPeakOutputReverse(-1, AutoConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - migrated to v6 as per docs https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/configuration-guide.html */
		var slot0Configs = new Slot0Configs();
        slot0Configs.kP = AutoConstants.kGains.kP;
        slot0Configs.kI = AutoConstants.kGains.kI;
        slot0Configs.kD = AutoConstants.kGains.kD;
		driveMotor.getConfigurator().apply(slot0Configs, AutoConstants.kGains.kF);


        /*
         * CONFIGURE MOTION MAGIC----------------------------------------------------
         */

        var talonFXConfigs = new TalonFXConfiguration();

        var motionMagicCongifs = talonFXConfigs.MotionMagic;
        
		/* Set acceleration and vcruise velocity - see documentation */
		driveMotor.configMotionCruiseVelocity(26000, AutoConstants.kTimeoutMs);
		driveMotor.configMotionAcceleration(10000, AutoConstants.kTimeoutMs);
        // 15000 vel, 6000 accel

        driveMotor.configMotionSCurveStrength(0);

		/* Zero the sensor once on robot boot up */
		driveMotor.setSelectedSensorPosition(0, AutoConstants.kPIDLoopIdx, AutoConstants.kTimeoutMs);
    }

    public boolean setDriveMode(boolean isBrakeMode) {
        driveMotor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        turningMotor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        this.isBrakeMode = isBrakeMode;
        return isBrakeMode;
    }

    public IdleMode getDriveMode() {
        return turningMotor.getIdleMode();
    }
    
    /**Set whether the drive motor is inverted 
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

    public double getAbsoluteEncoderDeg() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();   // No offset
    }

    public double getAbsoluteEncoderRad() {
        double angle = Math.toRadians(getAbsoluteEncoderDeg()) - absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
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

        // SET MOTORS ----------------------------------------------------------------------
        
        // percent out control
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        // positon control
        /* 2048 ticks/rev * 10 Rotations in either direction */
        // double convToSensorCounts = (state.speedMetersPerSecond * (2048 / (ModuleConstants.kWheelDiameterMeters * Math.PI))) / 8.14;
        // convToSensorCounts =  convToSensorCounts > DriveConstants.kPhysicalMaxSpeedMetersPerSecond ? DriveConstants.kPhysicalMaxSpeedMetersPerSecond : convToSensorCounts; 
		// System.out.println(state.speedMetersPerSecond);
        // driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond);

        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void setAutoDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        // SET MOTORS ----------------------------------------------------------------------
        
        // percent out control
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond);
        
        // positon control
        /* 2048 ticks/rev * 10 Rotations in either direction */
        // double convToSensorCounts = (state.speedMetersPerSecond * (2048 / (ModuleConstants.kWheelDiameterMeters * Math.PI))) / 8.14;
        // convToSensorCounts =  convToSensorCounts > DriveConstants.kPhysicalMaxSpeedMetersPerSecond ? DriveConstants.kPhysicalMaxSpeedMetersPerSecond : convToSensorCounts; 
		// System.out.println(state.speedMetersPerSecond);
        // driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond);

        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput,0);
        turningMotor.set(0);
    }
}