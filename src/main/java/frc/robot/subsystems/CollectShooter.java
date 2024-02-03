package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.OI.OperatorXbox;

public class CollectShooter extends SubsystemBase{

    /** CAN IDs -- MOVE TO ROBOTMAP */
    private final int kCANCollector = 1;
    private final int kCANTop = 2;    
    private final int kCANBot = 3;


    /** Collector CAN Spark Flex Motor */
    private CANSparkFlex collector = new CANSparkFlex(kCANCollector, MotorType.kBrushless);    
    
    /** Upper shooter CAN Spark Flex Motor */
    private CANSparkFlex shootTop = new CANSparkFlex(kCANTop, MotorType.kBrushless);
    
    /** Lower shooter CAN Spark Flex Motor */
    private CANSparkFlex shootBottom = new CANSparkFlex(kCANBot, MotorType.kBrushless);

    /** Initialize systems to set constants and defaults */
    public void init() {
        collector.restoreFactoryDefaults();        
        shootTop.restoreFactoryDefaults();
        shootBottom.restoreFactoryDefaults();
        collector.clearFaults();
        shootTop.clearFaults();
        shootBottom.clearFaults();
    }

    /** Basic run function for the shooter wheels -- needs to be moved to a command     */
    public void runShooter() {
        collector.set(OperatorXbox.getLeftX());
        shootTop.set(OperatorXbox.getRightX());        
        shootBottom.set(-OperatorXbox.getRightX());

    }

}
