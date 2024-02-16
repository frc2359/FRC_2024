package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CollectShooter;

public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;
    private CollectShooter collectShooter = new CollectShooter();

    private int csState;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        m_robotContainer.getSwerveSubsystem().setDriveMode(true);

        collectShooter.init();
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
        // collectShooter.runShooter();
        csState = collectShooter.stateMachine();
        SmartDashboard.putNumber("cs State", csState);
        SmartDashboard.putBoolean("Sens 1",IO.Sensor.getNoteSensor(0));
        SmartDashboard.putBoolean("Note Det.", IO.Sensor.isNoteDetected());
    }

}