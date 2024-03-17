package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveDrive.SwerveDriveSubsystem;

public class DriveTimed extends Command {

    private SwerveDriveSubsystem drive;
    private double speed;
    private double angle;
    private long delayTime;
    private long startTime;



    public DriveTimed (SwerveDriveSubsystem ds, double spd, double ang, long delay) {
        this.drive = ds;
        this.speed = spd;
        this.angle = ang;
        this.delayTime = delay;
    }


    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }
    
    @Override
    public void execute() {
        drive.directionalDrive(speed, angle, 0);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) >= delayTime;
    }

    @Override
    public void end(boolean interruped) {
        drive.directionalDrive(0,angle,0);
    }
}
