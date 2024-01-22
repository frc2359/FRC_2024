package frc.robot.subsystems;

//subsystem interface
import edu.wpi.first.wpilibj2.command.Subsystem;

//limelight
import edu.wpi.first.networktables.*;

public class Limelight implements Subsystem{


    public static double getLimelightXAngle() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = limelight.getEntry("tx");
        return tx.getDouble(0.0);
    }

    public static double getLimelightYAngle() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = limelight.getEntry("ty");
        return ty.getDouble(0.0);
    }

    //NEED TO KNOW MOUNT ANGLE, MOUNT HEIGHT, AND APRIL TAG TARGET HEIGHT
    // public static double calculateDistance(double limelightMountAngleDegrees, double limelightLensHeightInches, double goalHeightInches) {
    //     double angleToGoalDegrees = limelightMountAngleDegrees + getLimelightYAngle();
    //     double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);
    //     return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    // }

    public static double calculateDistance(double YAngle, double limeHeight, double XAngle){
        double distance = limeHeight/Math.tan(YAngle);
        if(XAngle < 0){
            // turn right, turn left for if greater than 0 (excluding margin of error)
        }
        return distance;
    }

}