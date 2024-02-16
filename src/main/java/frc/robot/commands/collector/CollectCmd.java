package frc.robot.commands.collector;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IO.OI.OperatorXbox;
import frc.robot.subsystems.CollectShooter;
import frc.robot.IO.Modifiers;

public class CollectCmd extends Command{

    private CollectShooter collectShooter;

    /** This is a command to run the collector. Eventually will need to implement light sensor, and maybe prepare for shooting. TBD later. */
    public CollectCmd(CollectShooter collectShooter) {
        this.collectShooter = collectShooter;
        super.addRequirements(collectShooter);
    }

    /** This is run periodically until isFinished returns true, and is where you would place things you would like to represent an "activated" state. */
    @Override
    public void execute() {
        collectShooter.setCollectorPctPower(0.6);
    }

    /** This determines the "finished" condition. When this returns true, the command is taken off the scheduler, and stops running. */
    @Override
    public boolean isFinished() {
        collectShooter.setShooterPctPower(-.1);
        return (OperatorXbox.isAPressed() || false); // 2nd part to be transferred to the DIO sensor
    }

    /** This is run after isFinished returns true, and is where you would place things you would like to represent a "deactivated" state. */
    @Override
    public void end(boolean interrupted) {
        collectShooter.setCollectorPctPower(0);
    }

    
}
