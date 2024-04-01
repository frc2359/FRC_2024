package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO2;
import frc.robot.IO2.Camera;
import frc.robot.IO2.OI;
import frc.robot.IO2.OI.RobotControls;
import frc.robot.RobotMap.ButtonBOX;
import frc.robot.RobotMap.CollectShooterConstants.CS;
import frc.robot.RobotMap.CollectShooterConstants.State_CS;
import frc.robot.RobotMap.LEDConstants;
import frc.robot.IO.IO_Subsystem;
import frc.robot.Navigation.NavigationSubsystem;
import frc.robot.SwerveDrive.SwerveDriveSubsystem;
import frc.robot.SwerveDrive.SwerveDriveCmd;
import frc.robot.subsystems.CollectShooter;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.LEDs;

public class Robot extends TimedRobot {
    //private RobotContainer m_robotContainer;
    // Subsystems
    private SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();
    private CollectShooter collectShooter = new CollectShooter();
    private LEDs leds = new LEDs();
    private IO_Subsystem ioSubsystem = new IO_Subsystem();
    private NavigationSubsystem navigationSubsystem = new NavigationSubsystem(driveSubsystem::getPositions);
    private LifterSubsystem lifter = new LifterSubsystem();

    private int csState;

    private int countAuto = 0;

    private boolean autoSide = true; // goes to robot left during auto

    private static int autoSelector = 1;
    private static final String autoLeft = "Left";
    private static final String autoRight = "Right";
    private static final String shootOnly = "Shoot Only";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private static final String kPosSpkrCenter = "S.Ctr";
    private static final String kPosSpkrAmp = "S.Amp";
    private static final String kPosSpkrSource = "S.Source";
    private static final String kPosWallSource = "W.Source";
    private static final String kPosWallAmp = "W.Amp";

    private final SendableChooser<String> chooseAutoStartPos = new SendableChooser<>();
    private String autoStartPos = "";
    private final SendableChooser<String> chooseNumPiecs = new SendableChooser<>();
    private String autoPiecesString = "3";
    private final double autoPieces = 3;
    private final double autoWaitTime = 0;
    //private final String autoDir = "Left";


    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        //m_robotContainer = new RobotContainer();

       // m_robotContainer.getSwerveSubsystem().setDriveMode(true);

        collectShooter.init();

        lifter.init();

        leds.init();
        leds.initLEDs();

        Camera.initCamera();

        driveSubsystem.setDefaultCommand(new SwerveDriveCmd(driveSubsystem, ioSubsystem, navigationSubsystem));

        //m_robotContainer.getSwerveSubsystem().hardResetEncoders(); //run once and comment out when done
        
        if(IO2.Status.isTeamRed()) {
            m_chooser.setDefaultOption("Left Auto", autoLeft);
            m_chooser.addOption("Right Auto", autoRight);
            m_chooser.addOption("Shoot Only", shootOnly);
        }
        else {
            m_chooser.setDefaultOption("Left Auto", autoLeft);
            m_chooser.addOption("Right Auto", autoRight);
            m_chooser.addOption("Shoot Only", shootOnly);
        }
        SmartDashboard.putData("Auto Select", m_chooser);

        chooseAutoStartPos.setDefaultOption("Spkr-Center", kPosSpkrCenter);
        chooseAutoStartPos.addOption("Spkr-Amp side", kPosSpkrAmp);
        chooseAutoStartPos.addOption("Spkr-Src side", kPosSpkrSource);
        chooseAutoStartPos.addOption("Wall-by Amp", kPosWallAmp);
        chooseAutoStartPos.addOption("Wall-by Src", kPosSpkrSource);
        SmartDashboard.putData("Start Pos", chooseAutoStartPos);
 
        chooseNumPiecs.setDefaultOption("3", "3");
        chooseNumPiecs.addOption("0", "0");
        chooseNumPiecs.addOption("1", "1");
        chooseNumPiecs.addOption("2", "2");
        chooseNumPiecs.addOption("4", "4");
        SmartDashboard.putData("Num Pieces", chooseNumPiecs);
        
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
        //CommandScheduler.getInstance().run();
        leds.executePeriodic();
    }

    @Override
    public void disabledInit() {

    }


    @Override
    public void disabledPeriodic() {
         /* ------------------------------ ROBOT BUTTONS -----------------------------
          * White  = Robot Status
          * Yellow = reset Gyro heading
          * Red    = set motors to coast mode
          * Green  = TBD
          * Blue   = show LED alignment pattern
          */
        if(RobotControls.getDIO(RobotMap.RobotButtons.kWhite)) {
            leds.setState(LEDConstants.STATE_LEDS_STATUS);
        } else if (RobotControls.getDIO(RobotMap.RobotButtons.kBlue)) {
            leds.setState(LEDConstants.STATE_LEDS_ALIGN);
        } else if (RobotControls.getDIO(RobotMap.RobotButtons.kGreen)) {
            leds.testLEDs();
        } else {
            //leds.SetColor(128,45,0,true);
        }
        //SmartDashboard.putBoolean("DIO_W", RobotControls.getDIO(RobotMap.RobotButtons.kWhite));
        if (RobotControls.getDIO(RobotMap.RobotButtons.kYellow)) {
            IO2.Gyro.zeroHeading();
        }
        if (RobotControls.getDIO(RobotMap.RobotButtons.kRed)) {
           //m_robotContainer.getSwerveSubsystem().setDriveMode(false);
        }
        if (RobotControls.getDIO(RobotMap.RobotButtons.kGreen)) {
            //leds.setState(LEDConstants.STATE_LEDS_ALIGN);
        }

    }

    /** This function is called one time before autonomousPeriodic is run. */
    @Override
    public void autonomousInit() {
        countAuto = 0;
        collectShooter.off();
        //m_autoSelected = m_chooser.getSelected();
        //System.out.println("Auto selected: " + m_autoSelected);
        //SmartDashboard.getNumber("auto", autoSelector);
        //SmartDashboard.get
        m_autoSelected = m_chooser.getSelected();
        autoStartPos = chooseAutoStartPos.getSelected();
        autoPiecesString = chooseNumPiecs.getSelected();
        
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        // Allow the robot to perform different auto paths
        // This is obtained from the SmartDashboard
        switch (autoSelector) {
            case 1: //auto towards amp red, away from amp blue (right)
                autoSide = false;
                shootThree();
                break;

            case 2: // auto away from amp red, towards amp blue  (left)
                autoSide = true;
                shootThree(); 

                break;

            case 3: //only shoot
                if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                }

            case 4: //shoot 2
                if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
                    collectShooter.intakeCollector();
                } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);  
                } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
                    driveSubsystem.directionalDrive(.25, Math.PI, 0);  
                } else if (countAuto == 250) {                                          // shoot
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                }

                break;
        }

        countAuto++;
        if(IO2.Status.isTeamRed()) {
            switch (m_autoSelected) {
            case shootOnly:
              // Put custom auto code here
              if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                }
              break;

            case autoRight: 
            if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
                    collectShooter.intakeCollector();
                } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);  
                } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
                    driveSubsystem.directionalDrive(.25, Math.PI, 0);  
                } else if (countAuto == 250) {                                          // shoot
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto > 350 && countAuto < 380) {                        // drive away from speaker
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);
                } else if (countAuto > 380 && countAuto < 470) {                        
                    driveSubsystem.directionalDrive(.237,Math.PI/2, 0); // drive right
                }
                    if (countAuto == 470) {
                    collectShooter.intakeCollector();
                }
                if (countAuto > 470 && countAuto < 540) {                               // drive away from wall to pick up note
                    driveSubsystem.directionalDrive(.257, 0,0);
                }
                if (countAuto >= 540 && countAuto <= 596) {                             // drive left to align with speaker
                    driveSubsystem.directionalDrive(.345, -Math.PI/2,0); 
                }
                if (countAuto == 596 ) {                                                // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto > 596 && countAuto <= 649){                               // drive towards speaker
                    driveSubsystem.directionalDrive(.48, Math.PI, 0);  
                }
                if (countAuto >= 649) {                                                 // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto == 650) {                                                 // shoot
                    leds.setCol(0,255,0,false);
                collectShooter.shootSpeaker();
                }
              break;

            case autoLeft:
            default:
              // Put default auto code here
              if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
                    collectShooter.intakeCollector();
                } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);  
                } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
                    driveSubsystem.directionalDrive(.25, Math.PI, 0);  
                } else if (countAuto == 250) {                                          // shoot
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto > 350 && countAuto < 380) {                        // drive away from speaker
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);
                } else if (countAuto > 380 && countAuto < 470) {                        
                    driveSubsystem.directionalDrive(.237,-Math.PI/2, 0); // drive left
                }
                    if (countAuto == 470) {
                    collectShooter.intakeCollector();
                }
                if (countAuto > 470 && countAuto < 540) {                               // drive away from wall to pick up note
                    driveSubsystem.directionalDrive(.257, 0,0);
                }
                if (countAuto >= 540 && countAuto <= 596) {                             // drive  rightto align with speaker
                    driveSubsystem.directionalDrive(.345, Math.PI/2,0); 
                }
                if (countAuto == 596 ) {                                                // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto > 596 && countAuto <= 649){                               // drive towards speaker
                    driveSubsystem.directionalDrive(.48, Math.PI, 0);  
                }
                if (countAuto >= 649) {                                                 // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto == 650) {                                                 // shoot
                    leds.setCol(0,255,0,false);
                collectShooter.shootSpeaker();
                }
              break;
          }
        }
        else {
            switch (m_autoSelected) {
            case shootOnly:
              // Put custom auto code here
              if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                }
              break;

            case autoLeft: 
            if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
                    collectShooter.intakeCollector();
                } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);  
                } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
                    driveSubsystem.directionalDrive(.25, Math.PI, 0);  
                } else if (countAuto == 250) {                                          // shoot
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto > 350 && countAuto < 380) {                        // drive away from speaker
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);
                } else if (countAuto > 380 && countAuto < 470) {                        
                    driveSubsystem.directionalDrive(.237,-Math.PI/2, 0); // drive left
                }
                    if (countAuto == 470) {
                    collectShooter.intakeCollector();
                }
                if (countAuto > 470 && countAuto < 540) {                               // drive away from wall to pick up note
                    driveSubsystem.directionalDrive(.257, 0,0);
                }
                if (countAuto >= 540 && countAuto <= 596) {                             // drive  rightto align with speaker
                    driveSubsystem.directionalDrive(.345, Math.PI/2,0); 
                }
                if (countAuto == 596 ) {                                                // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto > 596 && countAuto <= 649){                               // drive towards speaker
                    driveSubsystem.directionalDrive(.48, Math.PI, 0);  
                }
                if (countAuto >= 649) {                                                 // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto == 650) {                                                 // shoot
                    leds.setCol(0,255,0,false);
                collectShooter.shootSpeaker();
                }
              break;

            case autoRight:
            default:
              // Put default auto code here
              if (countAuto < 1) {                                                  // Wait 1 Sec
                    leds.setCol(255,95,0,false);
                } else if (countAuto == 1) {                                           // Shoot
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
                    collectShooter.intakeCollector();
                } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);  
                } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
                    driveSubsystem.directionalDrive(.25, Math.PI, 0);  
                } else if (countAuto == 250) {                                          // shoot
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,255,0,false);
                    collectShooter.shootSpeaker();
                } else if (countAuto > 350 && countAuto < 380) {                        // drive away from speaker
                    driveSubsystem.directionalDrive(.25, 0, 0);
                    leds.setCol(0,0,255,false);
                } else if (countAuto > 380 && countAuto < 470) {                        
                    driveSubsystem.directionalDrive(.237,Math.PI/2, 0); // drive right
                }
                    if (countAuto == 470) {
                    collectShooter.intakeCollector();
                }
                if (countAuto > 470 && countAuto < 540) {                               // drive away from wall to pick up note
                    driveSubsystem.directionalDrive(.257, 0,0);
                }
                if (countAuto >= 540 && countAuto <= 596) {                             // drive left to align with speaker
                    driveSubsystem.directionalDrive(.345, -Math.PI/2,0); 
                }
                if (countAuto == 596 ) {                                                // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto > 596 && countAuto <= 649){                               // drive towards speaker
                    driveSubsystem.directionalDrive(.48, Math.PI, 0);  
                }
                if (countAuto >= 649) {                                                 // stop motors
                    driveSubsystem.directionalDrive(0, 0, 0);
                    leds.setCol(0,0,0,false);
                }
                if (countAuto == 650) {                                                 // shoot
                    leds.setCol(0,255,0,false);
                collectShooter.shootSpeaker();
                }
              break;
          }
        }

        

        
        

        /*if(IO2.Status.isTeamRed()) {
            switch (m_autoSelected) {
            case shootOnly:

            break;

            case autoLeft:

            break;

            case autoRight:
            default:
              // Put default auto code here
              break;
          }

        }
        else {
            switch (m_autoSelected) {
            case shootOnly:

            break;

            case autoLeft:

            break;
            
            case autoRight:
            default:
              // Put default auto code here
              break;
          }

        }*/


        // TEST MOTOR DIRECTION
        /* 
        if (countAuto < 250) {
            driveSubsystem.directionalDrive(.3, 0, 0);
            leds.setCol(255,0,0,false);
        } else if (countAuto < 500) {
           driveSubsystem.directionalDrive(.3, -1 * Math.PI / 2, 0);
           leds.setCol(0,255,0,false);
        } else if (countAuto < 750) {
           driveSubsystem.directionalDrive(.3, -3 * Math.PI / 4, 0);
           leds.setCol(0,0,255,false);
        } else {
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,0,0,false);
        }
        */
        SmartDashboard.putNumber("AutoC", countAuto);
        csState = collectShooter.executePeriodic();
        leds.setState(LEDConstants.STATE_LEDS_COLLECT_SHOOT,csState);

        // Delayed Move
        // Wait 1 Sec; Shoot; Wait to 12 Secs and drive for 2 secs
        /* 
        if (countAuto < 50) {
            leds.setCol(255,95,0,false);
        } else if (countAuto == 50) {
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto == 12*50) {
            driveSubsystem.directionalDrive(.25, -1 * Math.PI / 4, 0);
        } else if (countAuto > 14*50) {
           driveSubsystem.directionalDrive(0,0, 0);
           leds.setCol(0,0,0,false);
        }
        // */

        
        //  SHOOT AND DRIVE BACK (Source side with Turn)
        /*
        if (countAuto < 50) {                                   // wait 1 second
            leds.setCol(255,95,0,false);
        } else if (countAuto == 50) {                           // Shoot
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto == 300) {                          // drive back
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,255,0,false);
        } else if (countAuto > 400 && countAuto < 600) {
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);  
        } else if (countAuto >= 600 ) {
           driveSubsystem.directionalDrive(.1, Math.PI / 4, 0);
        } else if (countAuto > 602 & countAuto < 700) {
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,0,false);
         } else if (countAuto > 700) {
            driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,0,0,false);
         }
        */

        
        // TWO PIECE AUTO FROM CENTER
         /*if (countAuto < 50) {                                                  // Wait 1 Sec
            leds.setCol(255,95,0,false);
        } else if (countAuto == 50) {                                           // Shoot
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto == 200) {                                          // After shot wait 3 seconds; intake on
           collectShooter.intakeCollector();
        } else if (countAuto > 200 && countAuto < 300) {                        // drive backward for 2 seconds
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);  
        } else if (countAuto > 300 && countAuto < 400) {                        // drive back to speaker             
            driveSubsystem.directionalDrive(.27, Math.PI, 0);  
        } else if (countAuto == 400) {                                          // shoot
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto > 500 && countAuto < 530) {                        // drive away from speaker
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);
        } else if (countAuto > 530 && countAuto < 630) {
            driveSubsystem.directionalDrive(.25,-Math.PI/2, 0);
        }
            if (countAuto == 630) {
            collectShooter.intakeCollector();
        }
        if (countAuto > 630 && countAuto < 700) {
            driveSubsystem.directionalDrive(.257, 0,0);
        }
        if (countAuto >= 700 && countAuto <= 750) {
            driveSubsystem.directionalDrive(.3, Math.PI/2,0);
        }
        if (countAuto >= 750 ) {            // stop motors
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,0,0,false);
        }*/

        /*if (countAuto < 1) {                                                  // Wait 1 Sec
            leds.setCol(255,95,0,false);
        } else if (countAuto == 1) {                                           // Shoot
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto == 150) {                                          // After shot wait 3 seconds; intake on
           collectShooter.intakeCollector();
        } else if (countAuto > 150 && countAuto < 250) {                        // drive backward for 2 seconds
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);  
        } else if (countAuto > 250 && countAuto < 350) {                        // drive back to speaker             
            driveSubsystem.directionalDrive(.27, Math.PI, 0);  
        } else if (countAuto == 350) {                                          // shoot
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto > 450 && countAuto < 480) {                        // drive away from speaker
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);
        } else if (countAuto > 480 && countAuto < 570) {
            driveSubsystem.directionalDrive(.25,-Math.PI/2, 0);
        }
            if (countAuto == 570) {
            collectShooter.intakeCollector();
        }
        if (countAuto > 570 && countAuto < 640) {
            driveSubsystem.directionalDrive(.257, 0,0);
        }
        if (countAuto >= 640 && countAuto <= 696) {
            driveSubsystem.directionalDrive(.35, Math.PI/2,0);
        }
        if (countAuto == 696 ) {            // stop motors
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,0,0,false);
        }
        if (countAuto > 696 && countAuto <= 744){
            driveSubsystem.directionalDrive(.48, Math.PI, 0);  
        }
        if (countAuto >= 744) {
            driveSubsystem.directionalDrive(0, 0, 0);
            leds.setCol(0,0,0,false);
        }*/

        //functional auto code

        /*
        if (countAuto < 1) {                                                  // Wait 1 Sec
            leds.setCol(255,95,0,false);
        } else if (countAuto == 1) {                                           // Shoot
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
           collectShooter.intakeCollector();
        } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);  
        } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
            driveSubsystem.directionalDrive(.25, Math.PI, 0);  
        } else if (countAuto == 250) {                                          // shoot
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        } else if (countAuto > 350 && countAuto < 380) {                        // drive away from speaker
           driveSubsystem.directionalDrive(.25, 0, 0);
           leds.setCol(0,0,255,false);
        } else if (countAuto > 380 && countAuto < 470) {                        
            if (autoSide) {                                                     // drive to robot left
                driveSubsystem.directionalDrive(.237,-Math.PI/2, 0);
            }
            else {                                                              //drive to robot right
                driveSubsystem.directionalDrive(.237,Math.PI/2, 0);
            }
            
        }
            if (countAuto == 470) {
            collectShooter.intakeCollector();
        }
        if (countAuto > 470 && countAuto < 540) {                               // drive away from wall to pick up note
            driveSubsystem.directionalDrive(.257, 0,0);
        }
        if (countAuto >= 540 && countAuto <= 596) {                             // drive to align with speaker
            if (autoSide) {                                                     // drive to robot right
                driveSubsystem.directionalDrive(.345, Math.PI/2,0); 
            }
            else {                                                              // drive to robot left
                driveSubsystem.directionalDrive(.345, -Math.PI/2,0);
            }
        }
        if (countAuto == 596 ) {                                                // stop motors
           driveSubsystem.directionalDrive(0, 0, 0);
           leds.setCol(0,0,0,false);
        }
        if (countAuto > 596 && countAuto <= 649){                               // drive towards speaker
            driveSubsystem.directionalDrive(.48, Math.PI, 0);  
        }
        if (countAuto >= 649) {                                                 // stop motors
            driveSubsystem.directionalDrive(0, 0, 0);
            leds.setCol(0,0,0,false);
        }
        if (countAuto == 650) {                                                 // shoot
            leds.setCol(0,255,0,false);
           collectShooter.shootSpeaker();
        }
        */

    }

    private void shootThree() {
            if (countAuto < 1) {                                                  // Wait 1 Sec
            leds.setCol(255,95,0,false);
            } else if (countAuto == 1) {                                           // Shoot
            leds.setCol(0,255,0,false);
            collectShooter.shootSpeaker();
            } else if (countAuto == 50) {                                          // After shot wait 1 seconds; intake on
            collectShooter.intakeCollector();
            } else if (countAuto > 50 && countAuto < 150) {                        // drive backward for 2 seconds
            driveSubsystem.directionalDrive(.25, 0, 0);
            leds.setCol(0,0,255,false);  
            } else if (countAuto > 150 && countAuto < 250) {                        // drive back to speaker             
                driveSubsystem.directionalDrive(.25, Math.PI, 0);  
            } else if (countAuto == 250) {                                          // shoot
            driveSubsystem.directionalDrive(0, 0, 0);
            leds.setCol(0,255,0,false);
            collectShooter.shootSpeaker();
            } else if (countAuto > 350 && countAuto < 380) {                        // drive away from speaker
            driveSubsystem.directionalDrive(.25, 0, 0);
            leds.setCol(0,0,255,false);
            } else if (countAuto > 380 && countAuto < 470) {                        
                if (autoSide) {                                                     // drive to robot left
                    driveSubsystem.directionalDrive(.237,-Math.PI/2, 0);
                }
                else {                                                              //drive to robot right
                    driveSubsystem.directionalDrive(.237,Math.PI/2, 0);
                }
                
            }
                if (countAuto == 470) {
                collectShooter.intakeCollector();
            }
            if (countAuto > 470 && countAuto < 540) {                               // drive away from wall to pick up note
                driveSubsystem.directionalDrive(.257, 0,0);
            }
            if (countAuto >= 540 && countAuto <= 596) {                             // drive to align with speaker
                if (autoSide) {                                                     // drive to robot right
                    driveSubsystem.directionalDrive(.345, Math.PI/2,0); 
                }
                else {                                                              // drive to robot left
                    driveSubsystem.directionalDrive(.345, -Math.PI/2,0);
                }
            }
            if (countAuto == 596 ) {                                                // stop motors
            driveSubsystem.directionalDrive(0, 0, 0);
            leds.setCol(0,0,0,false);
            }
            if (countAuto > 596 && countAuto <= 649){                               // drive towards speaker
                driveSubsystem.directionalDrive(.48, Math.PI, 0);  
            }
            if (countAuto >= 649) {                                                 // stop motors
                driveSubsystem.directionalDrive(0, 0, 0);
                leds.setCol(0,0,0,false);
            }
            if (countAuto == 650) {                                                 // shoot
                leds.setCol(0,255,0,false);
            collectShooter.shootSpeaker();
            }
        }

    /** This function is called one time before operator takes control. */
    @Override
    public void teleopInit() {
        //m_robotContainer.getSwerveSubsystem().setDriveMode(true);
        leds.setCol(9,255,0,false);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        // collectShooter.runShooter();
        csState = collectShooter.executePeriodic();

        //if (csState != State_CS.OFF) {
            leds.setState(LEDConstants.STATE_LEDS_COLLECT_SHOOT,csState);
        //}
        SmartDashboard.putNumber("CS State", csState);
        SmartDashboard.putBoolean("Sens 1",IO2.Sensor.getNoteSensor(1));
        SmartDashboard.putBoolean("Sens 2",IO2.Sensor.getNoteSensor(2));
        SmartDashboard.putBoolean("Sens 3",IO2.Sensor.getNoteSensor(3));
        SmartDashboard.putBoolean("Sens 4",IO2.Sensor.getNoteSensor(4));
        SmartDashboard.putBoolean("Sens 5",IO2.Sensor.getNoteSensor(5));
        SmartDashboard.putBoolean("Note Det.", IO2.Sensor.isNoteDetected());

        lifter.executePeriodic();
    }

    @Override
    public void testPeriodic() {
        //m_robotContainer.getSwerveSubsystem().showData();
        // collectShooter.setShooterVelocity(1200);

    }
}