package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        m_robotContainer.getSwerveSubsystem().setDriveMode(true);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}


    @Override
    public void disabledPeriodic() {}

    /** This function is called one time before autonomousPeriodic is run. */
    @Override
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called one time before operator takes control. */
    @Override
    public void teleopInit() {
        m_robotContainer.getSwerveSubsystem().setDriveMode(true);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

}