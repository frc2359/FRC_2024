// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SwerveModule {
  private static final double kWheelRadius = 0.0508; // determine
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Math.PI; // 1/2 rotation per second
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  
  private static final double ROTATION_LIMIT_SPEED = 0.7;

  TalonFX driveMotor;
  CANSparkMax rotateMotor;
  CANcoder encoder;
  RelativeEncoder driveEncoder;
  RelativeEncoder rotationEncoder;
  int id;
  double offset;
  private final PIDController pidDrive;
  private final PIDController pidRotate;
  final CANcoderConfigurator configurator;

  public SwerveModule(int driveId, int rotateId, int absEncoderId, double absEncoderOffset) {
    //driveMotor = new CANSparkMax(driveId, Constants.motorType);
    driveMotor = new TalonFX(driveId, "rio");
    rotateMotor = new CANSparkMax(rotateId, MotorType.kBrushless);    // Brushless NEO on SparkMax for turning
    rotateMotor.setInverted(false);  // check

    // Setup Drive Encoder
    //driveEncoder = driveMotor.getEncoder();
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder resolution.
    //driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Drive PID
    pidDrive = new PIDController(.63, 0, 0);
  
    // Setup Absolute Encoder
    encoder = new CANcoder(absEncoderId);
    id = absEncoderId;
    configurator = encoder.getConfigurator();
    var defaultAbsEncoderConfig = new CANcoderConfiguration();
    defaultAbsEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    defaultAbsEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // Clockwise_Positive;
    this.offset = absEncoderOffset;  // may not need to save
    defaultAbsEncoderConfig.MagnetSensor.MagnetOffset = absEncoderOffset; // in rotations
    configurator.apply(defaultAbsEncoderConfig);

    // Setup Rotation Encoder
    rotationEncoder = rotateMotor.getEncoder();
    pidRotate = new PIDController(0.63, 0, 0);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    //private final SimpleMotorFeedforward rotateFeedforward = new SimpleMotorFeedforward(1, 0.5);


  }

  public void drive(double speed, double rotate) {
    driveMotor.set(speed);
    rotateMotor.set(rotate);
    System.out.println(id + " " + driveEncoder.getPosition());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getMotorEncoderPosition() {
    return new SwerveModulePosition(
      driveMotor.getRotorPosition().getValue(), new Rotation2d(rotationEncoder.getPosition()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        //m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance())
        driveMotor.getRotorVelocity().getValue(), new Rotation2d(rotationEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(rotationEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = pidDrive.calculate(driveMotor.getRotorVelocity().getValue(), state.speedMetersPerSecond);
    //
    final double driveFeedforward = 0;  //m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        pidRotate.calculate(rotationEncoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward = 0;
        //rotateFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    rotateMotor.setVoltage(turnOutput + turnFeedforward);
  }
  
  public void directionalDrive(double speed, double angle) {
    pidRotate.setSetpoint(0);
    double pos = angle - position();
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
    double value = encoder.getAbsolutePosition().getValueAsDouble(); // - offset;
    if (value < -0.5)
      value += 1.0;
    if (value >= 0.5)
      value -= 1.0;
    return  -1 * value * 2 * Math.PI;
  }

  public double rawPosition() {

    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public void reset() {
    pidRotate.setSetpoint(0);
    double s = pidRotate.calculate(position());
    s = MathUtil.clamp(s, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(s);
  }

  public void stop() {
    driveMotor.stopMotor();
    rotateMotor.stopMotor();
  }

  public double getAbsoluteEncoder() {  // -.5 to +.5
    return encoder.getAbsolutePosition().getValueAsDouble(); // No offset
  }

  public double getTurningPosition() {
    return rotationEncoder.getPosition();
  }

  public double getTurningVelocity() {
    return rotationEncoder.getVelocity();
}
 
}
