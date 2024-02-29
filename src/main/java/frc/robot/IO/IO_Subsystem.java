// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DevMode;
import frc.robot.RobotMap.OIConstants;

public class IO_Subsystem extends SubsystemBase {

    // Joystick (4axis tilt) at port DRIVE_PORT (def @ RobotMap)
    private static Joystick driver = new Joystick(OIConstants.DRIVE_PORT);
  
    //public static class Driver {
            /**
             * Checks Button
             * 
             * @param btn is the targeted button
             */
            public boolean getButton(int btn) {
                return driver.getRawButtonPressed(btn);
            }

            /**
             * Get selected axis
             * 
             * @param ax is the axis you selected
             */
            public double getRawAxis(int ax) {
                return driver.getRawAxis(ax);
            }

            /** Get the lower dial, values from -1 to 1 */
            public double getSpeedDial() {
                return driver.getRawAxis(3);
            }

            /** Checks X Axis <b>FOR THE DRIVE CONTROLLER</b> */
            public double getDriveX() {
                return Math.abs(driver.getX()) > 0.1 ? driver.getX() : 0;
            }

            /** Checks Y Axis <b>FOR THE DRIVE CONTROLLER</b> */
            public double getDriveY() {
                return Math.abs(driver.getY()) > 0.1 ? driver.getY() : 0;
            }

            /** Checks stick angle <b>FOR THE DRIVE CONTROLLER</b> */
            public double getDriveDirection() {
                return driver.getDirectionRadians();
            }

            /** Checks stick magnitude <b>FOR THE DRIVE CONTROLLER</b> */
            public double getDriveMagnitude() {
                return driver.getMagnitude();
            }

            /** Checks stick twist <b>FOR THE DRIVE CONTROLLER</b> */
            public double getDriveTwist() {
                if(DevMode.isTelemetryEnabled) {
                    SmartDashboard.putNumber("Twist", driver.getTwist());
                }
                return Math.abs(driver.getTwist()) > 0.5 ? driver.getTwist() * 0.5 : 0;
            }

            /** Checks Trigger <b>FOR THE DRIVE CONTROLLER</b> */
            public boolean getTrigger() {
                return driver.getTrigger();
            }

            /** Checks POV (little hat guy on top) <b>FOR THE DRIVE CONTROLLER</b> */
            public double getPOV() {
                // return liftCont.getLeftTriggerAxis() - liftCont.getRightTriggerAxis();
                return driver.getPOV();
            }

            /**
             * Checks if POV (little hat guy on top) is rotated to an angle <b>FOR THE DRIVE
             * CONTROLLER</b>
             * 
             * @param angle is the desired angle to check for
             */
            public boolean isPOVToAngle(double angle) {
                return driver.getPOV() == angle;
            }
       // }

}
