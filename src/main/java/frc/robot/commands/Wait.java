package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Wait extends Command {

    private long delayTime;
    private long startTime;

    public Wait (long delay) {
        this.delayTime = delay;
    }


    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }
    
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= delayTime;
    }

    @Override
    public void end(boolean interruped) {

    }
}
