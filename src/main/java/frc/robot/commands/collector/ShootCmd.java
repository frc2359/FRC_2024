package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.IO.OI.OperatorXbox;
import frc.robot.subsystems.CollectShooter;

public class ShootCmd extends Command {
    CollectShooter collectShooter;

    /** This is a command to run the collector. Eventually will need to implement light sensor, and maybe prepare for shooting. TBD later. */
    public ShootCmd(CollectShooter collectShooter) {
        this.collectShooter = collectShooter;
        super.addRequirements(collectShooter);
    }

    /** Ramps shooter to velocity 1000 to prepare for shoot */
    public Command rampShooters() {
        return collectShooter.runOnce(() -> {
            collectShooter.setShooterVelocity(1000);
        });
    }

    public Command shoot() {
        Command shoot = collectShooter.runOnce(() -> {
            collectShooter.setCollectorPctPower(1);
        });
        return shoot;
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
        collectShooter.setShooterPctPower(0);
        collectShooter.setShooterVelocity(0);
    }
}
