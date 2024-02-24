package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface RobotMap {
    public static final class DevMode {
        public static final boolean isTelemetryEnabled = false;
    }

    /** Configuration constants for Limelight */
    public static final class LimelightConsants {
        public static final int kPipelineLedSettings = 0;
        public static final int kLedOff = 1;
        public static final int kLedBlink = 2;
        public static final int kLedOn = 3;
    }

    /** Constants for a Swerve Module (needs restructuring to separate TeleOp consts and Drivetrain consts) */
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio =  (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);  // check
        public static final double kTurningMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);  // check
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kPDriving = 0.5;
    }

    /** Constants for the Shooter */
    public static final class CollectShooterConstants {
        public static final double kP = 6e-5; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000015; 
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        public static final double maxRPM = 5700;
    }

    /** Constants for the Teleop and Drivetrain (needs restructuring to separate TeleOp consts and Drivetrain consts) */
    public static final class DriveConstants {
        public static final boolean INI_BRAKE_MODE_DRIVE = true;

        public static final boolean kBrakeMode = true;
        public static final boolean kCoastMode = false;

        public static boolean currentBrakeMode = INI_BRAKE_MODE_DRIVE;

        /**Physical constants that represent physical meansurements, motor offsets, etc */
        public static final class Physical {
            /** Distance between left and right wheels */
            public static final double kTrackWidth = Units.inchesToMeters(24.5);
            // CHECK THIS -- Center of Wheels are 2.75" from edgle, so TrackWidth and WheekBase should be 30-5.5 or 24.5 inches        
            
            /** Distance between front and back wheels */
            public static final double kWheelBase = Units.inchesToMeters(24.5);

            public static final double kBaseRadius = Math.sqrt(Math.pow((kTrackWidth / 2), 2) * Math.pow((kWheelBase / 2), 2));
            
            /** Kinematics configured with the location of each module in relation to the center */
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),    // front right
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),     // front left
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   // rear right
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));   // rear left
        }

        // MOTOR CONSTANTS ----------------------------------------------------------
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 1;

        
        public static final int kFrontLeftTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 5;


        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // ENCODER CONSTANTS ---------------------------------------------------------
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed =false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;


        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(90);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(270);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(270);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(90);

        /* Thursday night, 2/15 testing
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-90);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(-160);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(-180);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(10);
        */
        // SPEED CONSTANTS ---------------------------------------------------------
        public static final double kPhysicalMaxSpeedMetersPerSecond = 8;  // 13.5?
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond - 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 7;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

        
    }

    /** Constants used for autonomous */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final int MAX_PATH_SPEED_AUTO = 3;
        public static final int MAX_PATH_ACCEL_AUTO = 2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(kMaxSpeedMetersPerSecond, DriveConstants.Physical.kBaseRadius, new ReplanningConfig());

        /** Configuration constants for MotionMagic */
        public static final class MotionMagic {
            /**
             * Which PID slot to pull gains from
             */
            public static final int kSlotIdx = 0;

            /**
             * Talon FX supports multiple (cascaded) PID loops. For
             * now we just want the primary one.
             */
            public static final int kPIDLoopIdx = 0;

            /**
             * set to zero to skip waiting for confirmation, set to nonzero to wait and
             * report to DS if action fails.
             */
            public static final int kTimeoutMs = 30;

            /**
             * Gains used in Motion Magic, to be adjusted accordingly
             * Gains(kp, ki, kd, kf, izone, peak output); kP and kF have been converted.
             */
            public static final Gains kGains = new Gains(0.04, 0.0, 0.0, 0.04, 0, 1.0);
        }
    }

    /**
     *  Class that organizes gains used when assigning values to slots
     */
    public static final class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int kIzone;
        public final double kPeakOutput;
        
        public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = _kIzone;
            kPeakOutput = _kPeakOutput;
        }
    }

    /** Configuration constants for Operator Input */
    public static final class OIConstants {
        public static final int DRIVE_PORT = 0; //USB IO Port  -- joystick init in robotcontainer, too.  Switch if needed
        public static final int LIFT_PORT = 1;
        public static final int BOX_PORT = 2;

        public static final boolean SEPARATE_CONTROLS = true;

        public static final double TURN_SPEED_MULT = 1;
        public static final double DRIVE_SPEED_MULT = 1;

        public static final double kDriverDeadband = 0.2;
    }

    /** Configuration constants for LEDs */
    public static final class LEDConstants {
        public static final int PWM_LEDS = 0;
        public static final int STATE_LEDS_OFF = 0;
        public static final int STATE_LEDS_INIT = 1;
        public static final int STATE_LEDS_STATUS = 2;
        public static final int STATE_LEDS_COLOR = 3;
        public static final int STATE_LEDS_PIECE = 4;
        public static final int STATE_LEDS_AUTO = 5;
        public static final int STATE_LEDS_COUNTDOWN = 6;

        public static final int PIECE_TYPE_UNKNOWN = 0;
        public static final int PIECE_TYPE_CUBE = 1;
        public static final int PIECE_TYPE_CONE = 2;
    }

    /** States for Collector - Shooter */
    public static final class State_CS {
        /** This is the unkown state for the collector shooter */
        public static final int UNKNOWN = 0;
        public static final int OFF = 1;
        public static final int COLLECTOR_INTAKE = 2;
        public static final int SHOOTER_INTAKE = 3;
        public static final int NOTE_READY = 4;
        public static final int PREPARE_TO_SHOOT = 5;
        public static final int SHOOT = 6;
        public static final int EJECT_NOTE = 7;
        public static final int ALIGN_NOTE = 8;  
        public static final int MANUAL = 99;
    }

    //** Types for Collector - Shooter */
    public static final class CS {
        public static final int kTargetNone = 0;
        public static final int kTargetAmp = 1;
        public static final int kTargetSpeaker = 2;
        public static final double kEjectSlow = -.25;
        public static final double kEjectFast = -.75;
    }


    //** Button Box Buttons */
    public static final class ButtonBOX {
        public static final int INTAKE_COLLECTOR = 5;
        public static final int INTAKE_SHOOTER = 4;
        public static final int INTAKE_OFF = 6;
        public static final int SHOOT_AMP = 9;
        public static final int SHOOT_SPEAKER = 12;
        public static final int INTAKE_EJECT = 7;
        public static final int INTAKE_EJECT_FAST = 8;
    }
}
