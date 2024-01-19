package frc.robot.subsystems;


//import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.RobotMap.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Telemetry extends SubsystemBase {
    private PowerDistribution pD;
    private DriverStation dS;

    public double getBatVoltage() {
        return pD.getVoltage();
    }
    
    
}
