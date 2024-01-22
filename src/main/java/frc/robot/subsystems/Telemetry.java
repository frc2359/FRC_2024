package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Telemetry extends SubsystemBase {
    private PowerDistribution pD;

    public double getBatVoltage() {
        return pD.getVoltage();
    }
    
    
}
