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
import static frc.robot.RobotMap.*;
import java.util.HashMap;
import com.kauailabs.navx.frc.AHRS;

public class IO {
    private static Joystick driver = new Joystick(OIConstants.DRIVE_PORT); // Joystick (4axis tilt) at port DRIVE_PORT (def @ RobotMap)
    private static XboxController liftCont = new XboxController(OIConstants.LIFT_PORT); // Xbox Controllrt at port LIFT_PORT (def @ RobotMap)
    private static GenericHID buttonBox = new GenericHID(OIConstants.BOX_PORT); // Generic HID (collections of buttons) at port BOX_PORT
    private static PowerDistribution pdh = new PowerDistribution(); // power dist hub on robot
    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight"); // sets up network access to access limelight

    /** Provides information for the Robot Status, such as PDP stats, DS stats, connection information, etc */
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
         * The limelight uses NetworkTables. Think of it like a spreadsheet that is sent over the local robot network
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
            SmartDashboard.putNumberArray("botpose", m.get("botpose"));
            SmartDashboard.putNumberArray("t_c", m.get("target_camera"));
            SmartDashboard.putNumberArray("t_r", m.get("target_robot"));
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

        public static class GyroType {
            public static final boolean kNAVX = false;
            public static final boolean kADXRS = true;
        }

        public static double getFusedHeading() {
            return navx.getFusedHeading();
        }

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

        public static double getAngle(boolean gyroType) {
            return navx.getAngle();
            // return (gyroType == GyroType.kNAVX ? navx : adx).getAngle();
        }

        public static double getYaw(boolean gyroType) {
            // if (gyroType == GyroType.kNAVX) {
            return navx.getYaw();
            // } else {
            // return (adx.getAngle() % 360) - 180;
            // }
        }

        public static boolean isNavXAvail() {
            return navx.isConnected();
        }

        public static Rotation2d getRotation2D(boolean gyroType) {
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
        /**
         * Checks Button <b>FOR THE DRIVE CONTROLLER</b>
         * 
         * @param btn is the targeted button
         */
        public static boolean getDriverButton(int btn) {
            return driver.getRawButtonPressed(btn);
        }

        /**
         * Checks Button <b>FOR THE BUTTON BOX</b>
         * 
         * @param btn is the targeted button
         */
        public static boolean getHIDButton(int btn) {
            return (buttonBox != null ? buttonBox.getRawButtonPressed(btn) : false);
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

        /** Checks Left Y Axis <b>FOR THE LIFT CONTROLLER</b> */
        public static double getLiftControlLeftY() {
            return liftCont.getLeftY();
        }

        /** Checks Left X Axis <b>FOR THE LIFT CONTROLLER</b> */
        public static double getLiftControlLeftX() {
            return liftCont.getLeftX();
        }

        public static double getLiftControlRightX() {
            return liftCont.getRightX();
        }

        /** Checks left joystick pressed <b>FOR THE LIFT CONTROLLER</b> */
        public static boolean isLeftAxisPressed() {
            return liftCont.getLeftStickButtonPressed();
        }

        /** Checks left joystick pressed <b>FOR THE LIFT CONTROLLER</b> */
        public static boolean isRightAxisPressed() {
            return liftCont.getRightStickButtonPressed();
        }

        /** Checks X <b>FOR THE LIFT CONTROLLER</b> */
        public static boolean isXPressed() {
            return liftCont.getXButtonPressed();
        }

        /** Checks Y <b>FOR THE LIFT CONTROLLER</b> */
        public static boolean isYPressed() {
            return liftCont.getYButtonPressed();
        }

        /** Checks POV <b>FOR THE LIFT CONTROLLER</b> */
        public static int getLiftPOV() {
            return liftCont.getPOV();
        }

        /** Checks A <b>FOR THE LIFT CONTROLLER</b> */
        public static boolean isAPressed() {
            return liftCont.getAButtonPressed();
        }

        /** Checks B <b>FOR THE LIFT CONTROLLER</b> */
        public static boolean isBPressed() {
            return liftCont.getBButtonPressed();
        }

        public static boolean isLeftBumpPressed() {
            return liftCont.getLeftBumperPressed();
        }

        public static boolean isRightBumpPressed() {
            return liftCont.getRightBumperPressed();
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
            SmartDashboard.putNumber("Twist", driver.getTwist());
            return Math.abs(driver.getTwist()) > 0.5 ? driver.getTwist() * 0.5 : 0;
        }

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

    
}
