package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.OIConstants;
import frc.robot.RobotMap.DevMode;
import frc.robot.RobotMap.RobotSettings;

import edu.wpi.first.wpilibj.DigitalInput;

import java.util.HashMap;
import com.kauailabs.navx.frc.AHRS;

/**
 * Handles sending and recieving information that comes externally from the
 * robot
 * <p>
 * This ensures that two instantiations of the same device does not occur, which
 * often leads to errors.
 * </p>
 */

public class IO {
    // Joystick (4axis tilt) at port DRIVE_PORT (def @ RobotMap)
    private static Joystick driver = new Joystick(OIConstants.DRIVE_PORT);
    // Xbox Controller at port LIFT_PORT (def @ RobotMap)
    private static XboxController liftCont = new XboxController(OIConstants.LIFT_PORT);
    // Generic HID (collections of buttons at port BOX_PORT
    private static GenericHID buttonBox = new GenericHID(OIConstants.BOX_PORT);
    // power dist hub on robot
    private static PowerDistribution pdh = new PowerDistribution();
    // sets up network access to access limelight
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    private static DigitalInput white = new DigitalInput(RobotSettings.kWhite);
    private static DigitalInput yellow = new DigitalInput(RobotSettings.kYellow);
    private static DigitalInput red = new DigitalInput(RobotSettings.kRed);

    /** Note Sensor Class */
    public static class Sensor {

        private static final int numSensors = 5;

        private static DigitalInput[] sensorNote = new DigitalInput[] { new DigitalInput(5), new DigitalInput(6),
                new DigitalInput(7),
                new DigitalInput(8), new DigitalInput(9) };

        public static boolean isNoteDetected() {
            boolean flag = false;
            for (int i = 1; i <= numSensors; i++) {
                if (getNoteSensor(i))
                    flag = true;
            }
            SmartDashboard.putBoolean("Note Det.", flag);
            return flag;

            // return getNoteSensor(0);
        }

        public static boolean getNoteSensor(int sns) {
            if (sns < 1 || sns > numSensors) {
                return false;
            } else {
                return !sensorNote[sns - 1].get();
            }
        }
    }

    /** Functions that modify values to filter or normalize them */
    public static class Modifiers {
        /**
         * Adds a deadband to functions
         * 
         * @param value    value to apply a deadband to
         * @param deadband the distance from 0 (both positive and negative) that still
         *                 returns 0
         */
        public static double withDeadband(double value, double deadband) {
            return (Math.abs(value) < deadband ? 0 : value);
        }
    }

    /**
     * Provides information for the Robot Status, such as PDP stats, DS stats,
     * connection information, etc
     */
    public static class Status {
        public static double getBattVoltage() {
            return pdh.getVoltage();
        }

        public static boolean isTeamRed() {
            return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }
    }

    /** Helper functions for the Limelight functionality */
    public static class Limelight {
        /*
         * The limelight uses NetworkTables. Think of it like a spreadsheet that is sent
         * over the local robot network
         * (simiar to internet). If you're more savvy, you can read these like JSON.
         * There are key-value pairs that would be sent over ethernet that you can read.
         */

        private static NetworkTableEntry tx = limelightTable.getEntry("tx");
        private static NetworkTableEntry ty = limelightTable.getEntry("ty");
        private static NetworkTableEntry tz = limelightTable.getEntry("tz");
        private static NetworkTableEntry tarea = limelightTable.getEntry("ta");

        /**
         * Get AprilTag values
         * 
         * @return HashMap of botpose, target camera, and target robot values.
         */
        public static HashMap<String, double[]> getAprilTagValues() {
            HashMap<String, double[]> m = new HashMap<String, double[]>();
            m.put("botpose", limelightTable.getEntry("botpose").getDoubleArray(new double[6]));
            m.put("target_camera", limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]));
            m.put("target_robot", limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]));

            if (DevMode.isTelemetryEnabled) {
                SmartDashboard.putNumberArray("botpose", m.get("botpose"));
                SmartDashboard.putNumberArray("t_c", m.get("target_camera"));
                SmartDashboard.putNumberArray("t_r", m.get("target_robot"));
            }

            return m;
        }

        public static void setLed(int setpoint) {
            limelightTable.getEntry("ledMode").setNumber(setpoint);
        }

        /**
         * Returns a hashmap of the numbers returned from the limelight
         * 
         * @return Hashmap of entries for x ("tx"), y ("ty"), z ("tz"), and area
         *         ("tarea").
         */
        public static HashMap<String, Double> getLimelightValues() {
            HashMap<String, Double> m = new HashMap<String, Double>();
            m.put("tx", tx.getDouble(0.0));
            m.put("ty", ty.getDouble(0.0));
            m.put("tz", tz.getDouble(0.0));
            m.put("tarea", tarea.getDouble(0.0));
            return m;
        }
    }

    /** Helper functions for Gyro inputs and outputs */
    public static class Gyro {
        private static final AHRS navx = new AHRS(SPI.Port.kMXP);

        public static AHRS getNavX() {
            return navx;
        }

        /**
         * The type of Gyro, NavX or ADX(attached to robot)
         * <p>
         * <i>Needs to be configured so that if the NavX fails, there is a fallback to
         * ADX.</i>
         * </p>
         */
        public static class GyroType {
            public static final boolean kNAVX = false;
            public static final boolean kADXRS = true;
        }

        /**
         * Returns the "fused" (9-axis) heading from NavX.
         * <p>
         * The 9-axis heading is the fusion of the yaw angle, the tilt-corrected
         * compass heading, and magnetic disturbance detection. Note that the
         * magnetometer calibration procedure is required in order to
         * achieve valid 9-axis headings.
         * <p>
         * The 9-axis Heading represents the sensor's best estimate of current heading,
         * based upon the last known valid Compass Angle, and updated by the change in
         * the
         * Yaw Angle since the last known valid Compass Angle. The last known valid
         * Compass
         * Angle is updated whenever a Calibrated Compass Angle is read and the sensor
         * has recently rotated less than the Compass Noise Bandwidth (~2 degrees).
         * 
         * @return Fused Heading in Degrees (range 0-360)
         */
        public static double getFusedHeading() {
            return navx.getFusedHeading();
        }

        /** Zeroes heading for robot */
        public static void zeroHeading() {
            navx.reset();
            navx.setAngleAdjustment(180);
            // adx.reset();
        }

        public static void zeroYaw() {
            navx.zeroYaw();
        }

        public static boolean isRotating(int gyroType) {
            return navx.isRotating();
        }

        public static double getPitch() {
            return navx.getPitch();
        }

        public static double getRoll() {
            return navx.getRoll();
        }

        public static double getAngle() {
            return navx.getAngle();
        }

        public static double getAngleRad() {
            return navx.getAngle() * (Math.PI / 180);
        }

        /**
         * Returns the displacement (in meters) of the X axis since resetDisplacement()
         * was last invoked [Experimental].
         * 
         * NOTE: This feature is experimental. Displacement measures rely on
         * double-integration
         * of acceleration values from MEMS accelerometers which yield "noisy" values.
         * The
         * resulting displacement are not known to be very accurate, and the amount of
         * error
         * increases quickly as time progresses.
         * 
         * @return Displacement since last reset (in meters).
         */
        public static float getDisplacementX() {
            return navx.getDisplacementX();
        }

        /**
         * Returns the displacement (in meters) of the Y axis since resetDisplacement()
         * was last invoked [Experimental].
         * 
         * NOTE: This feature is experimental. Displacement measures rely on
         * double-integration
         * of acceleration values from MEMS accelerometers which yield "noisy" values.
         * The
         * resulting displacement are not known to be very accurate, and the amount of
         * error
         * increases quickly as time progresses.
         * 
         * @return Displacement since last reset (in meters).
         */
        public static float getDisplacementY() {
            return navx.getDisplacementY();
        }

        /**
         * Returns the displacement (in meters) of the Z axis since resetDisplacement()
         * was last invoked [Experimental].
         * 
         * NOTE: This feature is experimental. Displacement measures rely on
         * double-integration
         * of acceleration values from MEMS accelerometers which yield "noisy" values.
         * The
         * resulting displacement are not known to be very accurate, and the amount of
         * error
         * increases quickly as time progresses.
         * 
         * @return Displacement since last reset (in meters).
         */
        public static float getDisplacementZ() {
            return navx.getDisplacementZ();
        }

        public static double getYaw(boolean gyroType) {
            return navx.getYaw();
        }

        /** @return true if the NavX is available */
        public static boolean isNavXAvail() {
            return navx.isConnected();
        }

        public static Rotation2d getRotation2D() {
            return navx.getRotation2d();
            // return (gyroType == GyroType.kNAVX ? navx : adx).getRotation2d();
        }

        /** get the navx measured accel in m/s^2 */
        public static double getAccelerationMetersPerSecond() {
            return navx.getRawAccelX();
        }
    }

    /** Helper functions for the Operator Input */
    public static class OI {
        public static class RobotControls {
            public static boolean getDIO(int port) {
                if (port == RobotSettings.kRed || port == RobotSettings.kWhite || port == RobotSettings.kYellow) {
                    switch (port) {
                        case RobotSettings.kWhite:
                            return !white.get();
                        case RobotSettings.kRed:
                            return !red.get();
                        case RobotSettings.kYellow:
                            return !yellow.get();
                    }
                }
                return false;

            }
        }

        public static class Driver {
            /**
             * Checks Button
             * 
             * @param btn is the targeted button
             */
            public static boolean getButton(int btn) {
                return driver.getRawButtonPressed(btn);
            }

            /**
             * Get selected axis
             * 
             * @param ax is the axis you selected
             */
            public static double getRawAxis(int ax) {
                return driver.getRawAxis(ax);
            }

            /** Get the lower dial, values from -1 to 1 */
            public static double getSpeedDial() {
                return driver.getRawAxis(3);
            }

            public static double convToSpeedMult() {
                double spdMultiplier = ((getSpeedDial() + 1) * 0.25) + 0.5;
                SmartDashboard.putNumber("SpeedDriveMult", spdMultiplier);
                return spdMultiplier;
            }

            /** Checks X Axis <b>FOR THE DRIVE CONTROLLER</b> */
            public static double getDriveX() {
                return Math.abs(driver.getX()) > 0.1 ? driver.getX() : 0;
            }

            /** Checks Y Axis <b>FOR THE DRIVE CONTROLLER</b> */
            public static double getDriveY() {
                return Math.abs(driver.getY()) > 0.1 ? driver.getY() : 0;
            }

            /** Checks stick angle <b>FOR THE DRIVE CONTROLLER</b> */
            public static double getDriveDirection() {
                return driver.getDirectionRadians();
            }

            /** Checks stick magnitude <b>FOR THE DRIVE CONTROLLER</b> */
            public static double getDriveMagnitude() {
                return driver.getMagnitude();
            }

            /** Checks stick twist <b>FOR THE DRIVE CONTROLLER</b> */
            public static double getDriveTwist() {
                if (DevMode.isTelemetryEnabled) {
                    SmartDashboard.putNumber("Twist", driver.getTwist());
                }
                return Math.abs(driver.getTwist()) > 0.5 ? driver.getTwist() * 0.5 : 0;
            }

            /** Checks Trigger <b>FOR THE DRIVE CONTROLLER</b> */
            public static boolean getTrigger() {
                return driver.getTrigger();
            }

            /** Checks POV (little hat guy on top) <b>FOR THE DRIVE CONTROLLER</b> */
            public static double getPOV() {
                // return liftCont.getLeftTriggerAxis() - liftCont.getRightTriggerAxis();
                return driver.getPOV();
            }

            /**
             * Checks if POV (little hat guy on top) is rotated to an angle <b>FOR THE DRIVE
             * CONTROLLER</b>
             * 
             * @param angle is the desired angle to check for
             */
            public static boolean isPOVToAngle(double angle) {
                return driver.getPOV() == angle;
            }
        }

        /**
         * The Operator HID - currently configured for a button box that is used by an
         * operator
         */
        public static class OperatorHID {
            /**
             * Checks Button <b>FOR THE BUTTON BOX</b>
             * 
             * @param btn is the targeted button
             */
            public static boolean getButton(int btn) {
                return (buttonBox != null ? buttonBox.getRawButtonPressed(btn) : false);
            }
        }

        /**
         * The Operator Xbox Controller - currently configured for an Xbox Controller
         */
        public static class OperatorXbox {
            /** Checks Left Y Axis */
            public static double getLeftY() {
                return liftCont.getLeftY();
            }

            /** Checks Left X Axis */
            public static double getLeftX() {
                return liftCont.getLeftX();
            }

            /** Checks Right X Axis */
            public static double getRightX() {
                return liftCont.getRightX();
            }

            /** Checks left joystick pressed */
            public static boolean isLeftAxisPressed() {
                return liftCont.getLeftStickButtonPressed();
            }

            /** Checks left joystick pressed */
            public static boolean isRightAxisPressed() {
                return liftCont.getRightStickButtonPressed();
            }

            /** Checks X */
            public static boolean isXPressed() {
                return liftCont.getXButtonPressed();
            }

            /** Checks Y */
            public static boolean isYPressed() {
                return liftCont.getYButtonPressed();
            }

            /** Checks POV */
            public static int getLiftPOV() {
                return liftCont.getPOV();
            }

            /** Checks A */
            public static boolean isAPressed() {
                return liftCont.getAButtonPressed();
            }

            /** Checks B */
            public static boolean isBPressed() {
                return liftCont.getBButtonPressed();
            }

            /** Checks Left Bumper */
            public static boolean isLeftBumpPressed() {
                return liftCont.getLeftBumperPressed();
            }

            /** Checks Right Bumper */
            public static boolean isRightBumpPressed() {
                return liftCont.getRightBumperPressed();
            }
        }

    }

}
