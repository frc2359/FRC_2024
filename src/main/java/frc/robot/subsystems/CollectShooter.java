package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.CollectShooterConstants.State_CS;
import frc.robot.IO2;
import frc.robot.IO2.OI;
import frc.robot.IO2.OI.OperatorHID;
import frc.robot.IO2.OI.OperatorXbox;
import frc.robot.IO2.Modifiers;
import frc.robot.RobotMap.ButtonBOX;
import frc.robot.RobotMap.CollectShooterConstants.CS;
import frc.robot.RobotMap.CollectShooterConstants.PIDConstants;
import frc.robot.RobotMap.CollectShooterConstants.CANID;



public class CollectShooter extends SubsystemBase{

    /** Class Variables */
    private int stateCurrent = State_CS.UNKNOWN;    // Collector/Shooter Current State
    private double speedCollector = 0;
    private double speedShooter = 0;
    private int csTarget = CS.kTargetNone;
    private double joyCollector = 0;
    private double joyShooter = 0;
    private int countLoop = 0; 

    /** Collector CAN Spark Flex Motor */
    private CANSparkMax collector = new CANSparkMax(CANID.kCANCollector, MotorType.kBrushless);
    
    /** Upper shooter CAN Spark Flex Motor */
    private CANSparkFlex shootTop = new CANSparkFlex(CANID.kCANTop, MotorType.kBrushless);
    
    /** Lower shooter CAN Spark Flex Motor */
    private CANSparkFlex shootBottom = new CANSparkFlex(CANID.kCANBot, MotorType.kBrushless);

    private SparkPIDController shootTopPID;
    private SparkPIDController shootBottomPID;

    private RelativeEncoder encoderTop;
    private RelativeEncoder encoderBottom;

    public final static int kIntakeGround = 1;
    public final static int kIntakeShooter = 2;

    /** Initialize systems to set constants and defaults */
    public void init() {
        stateCurrent = State_CS.OFF;

        collector.restoreFactoryDefaults();   
        collector.clearFaults();
        collector.setIdleMode(IdleMode.kBrake);

        shootTop.restoreFactoryDefaults();
        shootTop.clearFaults();
        shootTop.setIdleMode(IdleMode.kBrake);
        encoderTop = shootTop.getEncoder();
        encoderTop.setVelocityConversionFactor(1);

        shootBottom.restoreFactoryDefaults();
        shootBottom.clearFaults();
        shootBottom.setIdleMode(IdleMode.kBrake);
        encoderBottom = shootBottom.getEncoder();
        // encoderBottom.setInverted(true);
        encoderBottom.setVelocityConversionFactor(1);

        //shootBottom.setInverted(true);
        
        shootTopPID = shootTop.getPIDController();     
        shootBottomPID = shootBottom.getPIDController();   

        // set PID coefficients
        // shootTopPID.setSmartMotionMaxVelocity(6000, 0);
        // shootTopPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        shootTopPID.setP(PIDConstants.kP);
        shootTopPID.setI(PIDConstants.kI);
        shootTopPID.setD(PIDConstants.kD);
        shootTopPID.setIZone(PIDConstants.kIz);
        shootTopPID.setFF(PIDConstants.kFF);
        shootTopPID.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
        // shootTopPID.

        // shootBottomPID.setSmartMotionMaxVelocity(6000, 0);
        // shootBottomPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        shootBottomPID.setP(PIDConstants.kP);
        shootBottomPID.setI(PIDConstants.kI);
        shootBottomPID.setD(PIDConstants.kD);
        shootBottomPID.setIZone(PIDConstants.kIz);
        shootBottomPID.setFF(PIDConstants.kFF);
        shootBottomPID.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
    }

    /** Set percent power for the collector motor */
    public void setCollectorPctPower(double percent) {
        collector.set(percent);
        // collectorSpark.set(percent);
    }

    // public void te() {
    //     setShooterVelocity(2500);
    //     // shootBottomPID.get/
        

    // }

    public void setPIDFfFromSmartDashbaord(double p, double i, double d, double ff) {
        double pSet = SmartDashboard.getNumber("CS P", PIDConstants.kP);        
        double iSet = SmartDashboard.getNumber("CS I", PIDConstants.kI);
        double dSet = SmartDashboard.getNumber("CS D", PIDConstants.kD);
        double ffSet = SmartDashboard.getNumber("CS Ff", PIDConstants.kFF);

        if(pSet != shootBottomPID.getP()) {
            shootBottomPID.setP(pSet);
            shootBottomPID.setI(iSet);
            shootBottomPID.setD(dSet);
            shootBottomPID.setFF(ffSet);
            shootTopPID.setP(pSet);
            shootTopPID.setI(iSet);
            shootTopPID.setD(dSet);
            shootTopPID.setFF(ffSet);
        }
    }
    
    public void setShooterVelocity(double velocity) {
        shootTopPID.setReference(velocity, CANSparkFlex.ControlType.kVelocity);
        shootBottomPID.setReference(-velocity, CANSparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber("TopRPM", encoderTop.getVelocity());
        SmartDashboard.putNumber("BotRPM",encoderBottom.getVelocity());
    }

    /** Set percent power for the shooter motors */
    public void setShooterPctPower(double percent) {
        shootTop.set(Modifiers.withDeadband(OperatorXbox.getRightX(), 0.1));        
        shootBottom.set(-Modifiers.withDeadband(OperatorXbox.getRightX(), 0.1));
  }

    /** Basic run function for the shooter */
    public void runShooter() {
        // collectorSpark.set(Modifiers.withDeadband(OperatorXbox.getLeftX(), 0.1));
        collector.set(Modifiers.withDeadband(OperatorXbox.getLeftX(), 0.1));
        shootTop.set(Modifiers.withDeadband(OperatorXbox.getRightX(), 0.1));        
        shootBottom.set(-Modifiers.withDeadband(OperatorXbox.getRightX(), 0.1));

    }

    private void setCollectorSpeed(double spdNew) {
        speedCollector = spdNew;
        collector.set(-speedCollector);
    }

    private void setShooterSpeed(double spdNew) {
        speedShooter = spdNew;
        shootTop.set(speedShooter);
        shootBottom.set(-speedShooter);
    }

    public void off() {
        setState(State_CS.OFF);
    }

    public void intakeCollector () {
        setState(State_CS.COLLECTOR_INTAKE);
    }

    public void intakeShooter () {
        setState(State_CS.SHOOTER_INTAKE);
    }

    public void intake(int intakeSource) {
        if (intakeSource == kIntakeGround) {
            intakeCollector();
        } else if (intakeSource == kIntakeShooter) {
            intakeShooter();
        }
    }
    public void shoot(int targ) {
        csTarget = targ;
        setState(State_CS.PREPARE_TO_SHOOT);
    }

    public void shootSpeaker() {
        csTarget = CS.kTargetSpeaker;
        setState(State_CS.PREPARE_TO_SHOOT);
    }

    public void shootAmp() {
        csTarget = CS.kTargetAmp;
        setState(State_CS.PREPARE_TO_SHOOT);
    }

    public Command collect() {
        return this.run(() -> {
            setCollectorPctPower(1);
        });
    }

    private void setState(int newState) {
        stateCurrent = newState;
        countLoop = 0;
    }

    // Process the current state of the shooter
    // Execute code relevant to the state, and transition to a new state (if needed)
    // Return the value of the state machine after all code has executed
    public int executePeriodic() {

        SmartDashboard.putNumber("Current State", stateCurrent);

        joyCollector = -1 * Modifiers.withDeadband(OperatorXbox.getLeftX(), 0.1);
        joyShooter = Modifiers.withDeadband(OperatorXbox.getRightX(), 0.1);   
        if (joyCollector != 0 || joyShooter != 0) {
            setState(State_CS.MANUAL);
        }

        countLoop ++;
        if (countLoop > 1000) {countLoop = 0;}
        SmartDashboard.putNumber("cnt Loop", countLoop);

        SmartDashboard.putNumber("TopRPM", encoderTop.getVelocity());
        SmartDashboard.putNumber("BotRPM",encoderBottom.getVelocity());


        /** if joystick1`<>0 or joystick2 <>0 then currentState = State_CD.MANUAL */

        switch (stateCurrent) {
            case State_CS.UNKNOWN:

                // init the robot

                // transition to a new state
                //currentState = State_CS.OFF;
                break;

            case State_CS.MANUAL:
                setCollectorSpeed(joyCollector);
                setShooterSpeed(joyShooter);

                if (joyCollector == 0 && joyShooter == 0) {
                    setState(State_CS.OFF);
                    break;
                }
                break;

            case State_CS.OFF:

                // collector motor to zero power
                setCollectorSpeed(0);
                
                // shooter motor to zero power
                setShooterSpeed(0);

                if (OperatorHID.getButton(ButtonBOX.INTAKE_COLLECTOR) || OperatorXbox.isYPressed()) {
                    setState(State_CS.COLLECTOR_INTAKE);
                    break;
                }
                
                if (OperatorHID.getButton(ButtonBOX.INTAKE_SHOOTER) || OperatorXbox.isAPressed()) { 
                     setState(State_CS.SHOOTER_INTAKE);
                    break;
                }
                
                if(IO2.Sensor.isNoteDetected()) {
                    setState(State_CS.ALIGN_NOTE);
                }

                break;

            case State_CS.COLLECTOR_INTAKE:
 
                // code to run the intake motors until the note is detected goes here
                setCollectorSpeed(.8);
                //setCollectorSpeed(.5);   // temp slow down - while testing sensors
                
                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                if (IO2.Sensor.isNoteDetected()) {
                    setState(State_CS.ALIGN_NOTE);
                    break;
                }

                if (OperatorHID.getButton(ButtonBOX.INTAKE_OFF) || OperatorXbox.isXPressed()) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.OFF);
                    break;
                }
                if (OperatorHID.getButton(ButtonBOX.INTAKE_EJECT) || OperatorXbox.isBPressed()) {        // Eject Button Pressed     
                    setState(State_CS.EJECT_NOTE);
                    break;
                }
                
                break;

            case State_CS.SHOOTER_INTAKE:

                // collect a note from the source
                // code to run the shooter motors in reverse (slowly maybe)
                setShooterSpeed(-.2);
                setCollectorSpeed(-.4);

                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                if (IO2.Sensor.isNoteDetected()) {
                    setState(State_CS.ALIGN_NOTE);
                }

                if (OperatorHID.getButton(ButtonBOX.INTAKE_OFF) || OperatorXbox.isXPressed()) {        // Off Button Pressed
                    setState(State_CS.OFF);
                    break;
                }

                break;

            case State_CS.ALIGN_NOTE:
                // if the note is no longer detected move to OFF state
                if (!IO2.Sensor.isNoteDetected()) {
                    setState(State_CS.OFF);
                    break;
                }

                // if the note is in the correct position in the chute, it is ready to be processed
                if (IO2.Sensor.isNoteDetected() && !IO2.Sensor.getNoteSensor(1) && !IO2.Sensor.getNoteSensor(5)
                    && IO2.Sensor.getNoteSensor(2) && IO2.Sensor.getNoteSensor(4)) {
                    setState(State_CS.NOTE_READY);
                    break;
                }

                if (OperatorHID.getButton(ButtonBOX.INTAKE_EJECT) || OperatorXbox.isBPressed()) {        // Eject Button Pressed     
                    setCollectorSpeed(CS.kEjectFast);
                    setState(State_CS.EJECT_NOTE);
                    break;
                }
 
                // if the note is not all the way in, then move it up slowly
                 if (IO2.Sensor.getNoteSensor(2) && !IO2.Sensor.getNoteSensor(4)) {
                    setCollectorSpeed(.3);
                 }   

                // if the note is too far in, then move it down slowly
                 if (!IO2.Sensor.getNoteSensor(2) && IO2.Sensor.getNoteSensor(4)) {
                    setCollectorSpeed(-.3);
                 }  

                break;

            case State_CS.NOTE_READY:

                // stop the collector motor (in case you collected from the ground)
                setCollectorSpeed(0);
                
                // stop the shooter motor (in case you were collecting from the source)
                setShooterSpeed(0);

                // if the note is no longer detected move to OFF state
                if (!IO2.Sensor.isNoteDetected() || OperatorHID.getButton(ButtonBOX.INTAKE_OFF) || OperatorXbox.isXPressed()) {
                    setState(State_CS.OFF);
                    break;
                }

                if (OperatorHID.getButton(ButtonBOX.SHOOT_SPEAKER) || OperatorXbox.isRightBumpPressed()) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.PREPARE_TO_SHOOT);
                    break;
                }
                
                if (OperatorHID.getButton(ButtonBOX.SHOOT_AMP) || OperatorXbox.isLeftBumpPressed()) {        // Shoot Amp Button Pressed     
                    csTarget = CS.kTargetAmp;
                    setState(State_CS.PREPARE_TO_SHOOT);
                    break;
                }

                                
                if (OperatorHID.getButton(ButtonBOX.SHOOT_MAX) || OperatorXbox.isYPressed()) {        // Shoot Amp Button Pressed     
                    csTarget = CS.kTargetMax;
                    setState(State_CS.PREPARE_TO_SHOOT);
                    break;
                }

                if (OperatorHID.getButton(ButtonBOX.INTAKE_EJECT) || OperatorXbox.isXPressed()) {        // Eject Button Pressed     
                    setCollectorSpeed(CS.kEjectFast);
                    setState(State_CS.EJECT_NOTE);
                    break;
                }

                break;

            case State_CS.PREPARE_TO_SHOOT:
                // DO NOT shoot until the wheels are at speed
                // spin the wheels for the shooter and get them up to speed

                // when the motors are finally up to speed, it is time to shoot
                // when true - transition to a new state
                if (csTarget == CS.kTargetSpeaker) {
                    setShooterSpeed(.8);
                } else if (csTarget == CS.kTargetMax) {
                    setShooterSpeed(1);
                } else {
                    setShooterSpeed(.195);
                    // setShooterVelocity(780);
                }

                // wait a period of time for the robot to eject the note
                if (countLoop > 50) { // OLD COUNT WAS 35
                // after the wait - transition back to the stop state
                    setState(State_CS.SHOOT);
                }
                // if(encoderTop.getVelocity() >= 3500) {
                //     setState(State_CS.SHOOT);
                // }

                break;

            case State_CS.SHOOT:

                // start the collection motor, to move the note forward
                setCollectorSpeed(.8);

                // wait a period of time for the robot to shoot the note
                if (countLoop > 70) {
                // after the wait - transition back to the stop state
                    setState(State_CS.OFF);
                }
                                
                break;

            case State_CS.EJECT_NOTE:

                // code to run the collector motors in reverse
                //setCollectorSpeed(-.5);
                // wait a period of time for the robot to eject the note
                if (countLoop > 100) {
                // after the wait - transition back to the stop state
                    setState(State_CS.OFF);
                }

                break;
        }

        //SmartDashboard.putNumber("CS State", stateCurrent);
        return stateCurrent; 
    }

}
