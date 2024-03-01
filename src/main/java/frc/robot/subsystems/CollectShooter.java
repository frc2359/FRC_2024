package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

//import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.CollectShooterConstants.State_CS;
import frc.robot.IO;
import frc.robot.IO.OI;
import frc.robot.IO.Modifiers;
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

    /** Initialize systems to set constants and defaults */
    public void init() {
        stateCurrent = State_CS.OFF;

        collector.restoreFactoryDefaults();   

        shootTop.restoreFactoryDefaults();
        shootBottom.restoreFactoryDefaults();
        collector.clearFaults();
        shootTop.clearFaults();
        shootBottom.clearFaults();

        //shootBottom.setInverted(true);
        
        shootTopPID = shootTop.getPIDController();     
        shootBottomPID = shootBottom.getPIDController();   

        // set PID coefficients
        shootTopPID.setSmartMotionMaxVelocity(6000, 0);
        shootTopPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        shootTopPID.setP(PIDConstants.kP);
        shootTopPID.setI(PIDConstants.kI);
        shootTopPID.setD(PIDConstants.kD);
        shootTopPID.setIZone(PIDConstants.kIz);
        shootTopPID.setFF(PIDConstants.kFF);
        shootTopPID.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

        shootBottomPID.setSmartMotionMaxVelocity(6000, 0);
        shootBottomPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
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

    public void setShooterVelocity(double velocity) {
        shootTopPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);
        shootBottomPID.setReference(velocity, CANSparkBase.ControlType.kVelocity);

    }

    /** Set percent power for the shooter motors */
    public void setShooterPctPower(double percent) {
        shootTop.set(Modifiers.withDeadband(OI.OperatorXbox.getRightX(), 0.1));        
        shootBottom.set(-Modifiers.withDeadband(OI.OperatorXbox.getRightX(), 0.1));
  }

    /** Basic run function for the shooter */
    public void runShooter() {
        // collectorSpark.set(Modifiers.withDeadband(OperatorXbox.getLeftX(), 0.1));
        collector.set(Modifiers.withDeadband(OI.OperatorXbox.getLeftX(), 0.1));
        shootTop.set(Modifiers.withDeadband(OI.OperatorXbox.getRightX(), 0.1));        
        shootBottom.set(-Modifiers.withDeadband(OI.OperatorXbox.getRightX(), 0.1));

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
    public int stateMachine() {

        SmartDashboard.putNumber("Current State", stateCurrent);

        joyCollector = -1 * Modifiers.withDeadband(OI.OperatorXbox.getLeftX(), 0.1);
        joyShooter = Modifiers.withDeadband(OI.OperatorXbox.getRightX(), 0.1);   
        if (joyCollector != 0 || joyShooter != 0) {
            setState(State_CS.MANUAL);
        }

        countLoop ++;
        if (countLoop > 1000) {countLoop = 0;}
        SmartDashboard.putNumber("cnt Loop", countLoop);

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

                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_COLLECTOR)) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.COLLECTOR_INTAKE);
                    break;
                }
                
                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_SHOOTER)) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.SHOOTER_INTAKE);
                    break;
                }
                
                if(IO.Sensor.isNoteDetected()) {
                    setState(State_CS.ALIGN_NOTE);
                }

                break;

            case State_CS.COLLECTOR_INTAKE:
 
                // code to run the intake motors until the note is detected goes here
                setCollectorSpeed(.8);
                //setCollectorSpeed(.5);   // temp slow down - while testing sensors
                
                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                if (IO.Sensor.isNoteDetected()) {
                    setState(State_CS.ALIGN_NOTE);
                    break;
                }

                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_OFF)) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.OFF);
                    break;
                }
                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_EJECT)) {        // Eject Button Pressed     
                    setState(State_CS.EJECT_NOTE);
                    break;
                }
                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_EJECT_FAST)) {        // Eject Button Pressed     
                    setState(State_CS.EJECT_NOTE);
                    break;
                }

                break;

            case State_CS.SHOOTER_INTAKE:

                // collect a note from the source
                // code to run the shooter motors in reverse (slowly maybe)
                setShooterSpeed(-.25);

                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                if (IO.Sensor.isNoteDetected()) {
                    setState(State_CS.ALIGN_NOTE);
                }

                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_OFF)) {        // Off Button Pressed
                    setState(State_CS.OFF);
                    break;
                }

                break;

            case State_CS.ALIGN_NOTE:
                // if the note is no longer detected move to OFF state
                if (!IO.Sensor.isNoteDetected()) {
                    setState(State_CS.OFF);
                    break;
                }

                // if the note is in the correct position in the chute, it is ready to be processed
                if (IO.Sensor.isNoteDetected() && !IO.Sensor.getNoteSensor(1) && !IO.Sensor.getNoteSensor(5)) {
                    setState(State_CS.NOTE_READY);
                    break;
                }

                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_EJECT)) {        // Eject Button Pressed     
                    setCollectorSpeed(CS.kEjectSlow);
                    setState(State_CS.EJECT_NOTE);
                    break;
                }
 
                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_EJECT_FAST)) {        // Eject Button Pressed     
                    setCollectorSpeed(CS.kEjectFast);
                    setState(State_CS.EJECT_NOTE);
                    break;
                }

                // if the note is not all the way in, then move it up slowly
                 if (IO.Sensor.getNoteSensor(1) && !IO.Sensor.getNoteSensor(5)) {
                    setCollectorSpeed(.25);;
                 }   

                // if the note is too far in, then move it down slowly
                 if (!IO.Sensor.getNoteSensor(1) && IO.Sensor.getNoteSensor(5)) {
                    setCollectorSpeed(-.25);;
                 }  

                break;

            case State_CS.NOTE_READY:

                // stop the collector motor (in case you collected from the ground)
                setCollectorSpeed(0);
                
                // stop the shooter motor (in case you were collecting from the source)
                setShooterSpeed(0);

                // if the note is no longer detected move to OFF state
                if (!IO.Sensor.isNoteDetected() || IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_OFF)) {
                    setState(State_CS.OFF);
                    break;
                }

                if (IO.OI.OperatorHID.getButton(ButtonBOX.SHOOT_SPEAKER)) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.PREPARE_TO_SHOOT);
                    break;
                }
                
                if (IO.OI.OperatorHID.getButton(ButtonBOX.SHOOT_AMP)) {        // Shoot Amp Button Pressed     
                    csTarget = CS.kTargetAmp;
                    setState(State_CS.PREPARE_TO_SHOOT);
                    break;
                }
                
                /* TODO
                Do we want two different buttons for two different speeds, SPEAKER and AMP
                or will one button do both?

                Decided: 2 buttons for 2 speeds, likely passing the speed parameter to shoot
                This will optimize the states, we won't need different states for SPEAKER vs AMP
                */

                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_EJECT)) {        // Eject Button Pressed     
                    setCollectorSpeed(CS.kEjectSlow);
                    setState(State_CS.EJECT_NOTE);
                    break;
                }
 
                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_EJECT_FAST)) {        // Eject Button Pressed     
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
                } else {
                    setShooterSpeed(.18);
                }

                // wait a period of time for the robot to eject the note
                if (countLoop > 35) {
                // after the wait - transition back to the stop state
                    setState(State_CS.SHOOT);
                }

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

        SmartDashboard.putNumber("Current State", stateCurrent);
        return stateCurrent; 
    }

}
