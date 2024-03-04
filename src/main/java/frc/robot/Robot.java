package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO2;
import frc.robot.IO2.OI;
import frc.robot.IO2.OI.RobotControls;
import frc.robot.RobotMap.CollectShooterConstants.State_CS;
import frc.robot.RobotMap.LEDConstants;
import frc.robot.subsystems.CollectShooter;
import frc.robot.subsystems.LEDs;

public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;
    private CollectShooter collectShooter = new CollectShooter();
    private LEDs leds = new LEDs();

    private int csState;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

       // m_robotContainer.getSwerveSubsystem().setDriveMode(true);

        collectShooter.init();

        leds.init();
        leds.initLEDs();

        //m_robotContainer.getSwerveSubsystem().hardResetEncoders(); //run once and comment out when done
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
        leds.runLEDs();
    }

    @Override
    public void disabledInit() {
        //leds.setCol(192,192,0,true);
        //leds.setCol(255,95,21,true);
        leds.setCol(255,95,0,true);    
    }


    @Override
    public void disabledPeriodic() {
         /* ------------------------------ ROBOT BUTTONS ----------------------------- */
        if(RobotControls.getDIO(RobotMap.RobotButtons.kWhite)) {
            leds.setState(LEDConstants.STATE_LEDS_STATUS);
        }
        if (RobotControls.getDIO(RobotMap.RobotButtons.kYellow)) {
            SmartDashboard.putBoolean("DIO_W", RobotControls.getDIO(RobotMap.RobotButtons.kWhite));
            IO2.Gyro.zeroHeading();
        }
        if (RobotControls.getDIO(RobotMap.RobotButtons.kRed)) {
           // m_robotContainer.getSwerveSubsystem().setDriveMode(false);
        }
        
    }

    /** This function is called one time before autonomousPeriodic is run. */
    @Override
    public void autonomousInit() {
        System.out.println(Math.atan2(1,1));
        System.out.println(Math.atan2(-1,1));
        System.out.println(Math.atan2(-1,-1));
        System.out.println(Math.atan2(1,-1));
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called one time before operator takes control. */
    @Override
    public void teleopInit() {
        //m_robotContainer.getSwerveSubsystem().setDriveMode(true);
        leds.setCol(9,255,0,false);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // collectShooter.runShooter();
        csState = collectShooter.stateMachine();
        //if (csState != State_CS.OFF) {
            leds.setState(LEDConstants.STATE_LEDS_COLLECT_SHOOT,csState);
        //}
        SmartDashboard.putNumber("cs State", csState);
        SmartDashboard.putBoolean("Sens 1",IO2.Sensor.getNoteSensor(1));
        SmartDashboard.putBoolean("Sens 3",IO2.Sensor.getNoteSensor(3));
        SmartDashboard.putBoolean("Sens 5",IO2.Sensor.getNoteSensor(5));
        SmartDashboard.putBoolean("Note Det.", IO2.Sensor.isNoteDetected());
    }

    @Override
    public void testPeriodic() {
        //m_robotContainer.getSwerveSubsystem().showData();
    }
}