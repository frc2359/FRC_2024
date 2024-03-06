// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants.Physical;
import frc.robot.RobotMap.ModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SwerveDriveModule {
    private static final double ROTATION_LIMIT_SPEED = 0.7;
    private SwerveModuleState currentState = new SwerveModuleState();

    TalonFX driveMotor;
    CANSparkMax rotateMotor;
    CANcoder encoder;
    RelativeEncoder distanceEncoder;
    RelativeEncoder rotationEncoder;
    int id;
    double alpha;
    private PIDController pidRotate;

    public SwerveDriveModule(int driveId, int rotateId, int encoderId, double alpha) {
        // driveMotor = new CANSparkMax(driveId, Constants.motorType);
        driveMotor = new TalonFX(driveId, "rio");
        rotateMotor = new CANSparkMax(rotateId, MotorType.kBrushless); // Brushless NEO on SparkMax for turning
        encoder = new CANcoder(encoderId);

        // default reports in rotations, conversion factor later set to conv to radians
        // doesn't nescessarily need to be in the WPI
        rotationEncoder = rotateMotor.getEncoder(); 
        id = encoderId;
        this.alpha = alpha;
        this.pidRotate = new PIDController(0.63, 0, 0);

        CANcoderConfigurator configurator = encoder.getConfigurator();

        var defaultEncoderConfig = new CANcoderConfiguration();
        defaultEncoderConfig.MagnetSensor.MagnetOffset = 5; // rotations

        defaultEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        configurator.apply(defaultEncoderConfig);

        rotateMotor.setInverted(false);

        
        // sparkmax reports radians instead of rotations and accounts for gear ratio
        rotationEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    }

    public void drive(double speed, double rotate) {
        driveMotor.set(speed);
        rotateMotor.set(rotate);
        System.out.println(id + " " + distanceEncoder.getPosition());
    }

    public SwerveModulePosition getMotorEncoderPosition() {
        return new SwerveModulePosition(
                //drive motor reporting rotations only          rotation encoder radians is converted to a Rotation2d that the robot
                //                                              can read
                driveMotor.getRotorPosition().getValue(), new Rotation2d(rotationEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                (driveMotor.getRotorPosition().getValue() * ModuleConstants.kDriveEncoderRot2Meter), //convert from rotations to meters
                new Rotation2d(rotationEncoder.getPosition())); // reporting in radians and converted to Rotation2d obj
    }

    public void directionalDrive(double speed, double angle) {
        pidRotate.setSetpoint(0);
        double pos = -position() - angle;
        while (pos < -Math.PI)
            pos += 2 * Math.PI;
        while (pos >= Math.PI)
            pos -= 2 * Math.PI;
        // -PI =< pos < PI

        double direction = 1.0;
        if (pos < -Math.PI / 2) {
            direction = -1.0;
            pos += Math.PI;
        }
        if (pos > Math.PI / 2) {
            direction = -1.0;
            pos -= Math.PI;
        }
        double speedOfRotation = pidRotate.calculate(pos);
        speedOfRotation = MathUtil.clamp(speedOfRotation, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
        rotateMotor.set(speedOfRotation);

        driveMotor.set(speed * direction);
    }

    private double position() {
        // position [-0.5..0.5]
        double value = encoder.getAbsolutePosition().getValueAsDouble() - alpha;
        if (value < -0.5)
            value += 1.0;
        if (value >= 0.5)
            value -= 1.0;
        return value * 2 * Math.PI;
    }

    public double rawPosition() {

        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void reset() {
        pidRotate.setSetpoint(0);
        double s = pidRotate.calculate(-position());
        s = MathUtil.clamp(s, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
        rotateMotor.set(s);
    }

    /** Stops motors */
    public void stop() {
        driveMotor.stopMotor();
        rotateMotor.stopMotor();
    }

    public double getAbsoluteEncoder() { // -.5 to +.5
        return encoder.getAbsolutePosition().getValueAsDouble(); // No offset
    }

    /**
     * Returns Turning Encoder Value (of SparkMax, NOT CAN Coder) in Radians (via
     * conversion factor set in init)
     */
    public double getTurningPosition() {
        return rotationEncoder.getPosition();
    }

    public double getTurningVelocity() {
        return rotationEncoder.getVelocity();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.get(), new Rotation2d(rotationEncoder.getPosition()));
    }

    /** Tells the module, "Hey, I want the wheel to be oriented x degrees at y velocity" and it calculates the best way to
     * make that happen
     * @param state desired state as a SwerveModuleState type.
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, currentState.angle);

        driveMotor.set(state.speedMetersPerSecond / AutoConstants.kMaxSpeedMetersPerSecond); // sets % power as (desired v / max v)
        rotateMotor.set(pidRotate.calculate(getTurningPosition(), state.angle.getRadians()));
    }

}
