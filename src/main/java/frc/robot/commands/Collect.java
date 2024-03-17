package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CollectShooter;

public class Collect extends Command {

    private CollectShooter collectorShooter;
    private int intakeSource;
    
    public Collect (CollectShooter cs, int intake) {
        this.collectorShooter = cs;
        this.intakeSource = intake;
    }


    @Override
    public void initialize() {
        cs.intake(intakeSource);
    }
    
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interruped) {
        
    }
}
