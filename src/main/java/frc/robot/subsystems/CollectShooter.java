package frc.robot.subsystems;

import frc.robot.IO;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.Modifiers;
import frc.robot.IO.OI.OperatorXbox;
import frc.robot.RobotMap.State_CS;
import frc.robot.RobotMap.ButtonBOX;
import frc.robot.RobotMap.CS;
import frc.robot.RobotMap.CollectShooterConstants;

public class CollectShooter extends SubsystemBase{

    /** Class Variables */
    private int stateCurrent = State_CS.UNKNOWN;    // Collector/Shooter Current State
    private int stateNext = State_CS.UNKNOWN;       // Next state
    private double speedCollector = 0;
    private double speedShooter = 0;
    private int csTarget = CS.kTargetNone;
    private double joyCollector = 0;
    private double joyShooter = 0;
    private int countLoop = 0; 

    
    
    /** CAN IDs -- MOVE TO ROBOTMAP */
    private final int kCANCollector = 10;
    private final int kCANTop = 2;    
    private final int kCANBot = 3;


    /** Collector CAN Spark Flex Motor */
    private CANSparkMax collector = new CANSparkMax(kCANCollector, MotorType.kBrushless);
    
    /** Upper shooter CAN Spark Flex Motor */
    private CANSparkFlex shootTop = new CANSparkFlex(kCANTop, MotorType.kBrushless);
    
    /** Lower shooter CAN Spark Flex Motor */
    private CANSparkFlex shootBottom = new CANSparkFlex(kCANBot, MotorType.kBrushless);

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

        shootBottom.setInverted(true);
        
        shootTopPID = shootTop.getPIDController();     
        shootBottomPID = shootBottom.getPIDController();   

        // set PID coefficients
        shootTopPID.setSmartMotionMaxVelocity(6000, 0);
        shootTopPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        shootTopPID.setP(CollectShooterConstants.kP);
        shootTopPID.setI(CollectShooterConstants.kI);
        shootTopPID.setD(CollectShooterConstants.kD);
        shootTopPID.setIZone(CollectShooterConstants.kIz);
        shootTopPID.setFF(CollectShooterConstants.kFF);
        shootTopPID.setOutputRange(CollectShooterConstants.kMinOutput, CollectShooterConstants.kMaxOutput);

        shootBottomPID.setSmartMotionMaxVelocity(6000, 0);
        shootBottomPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        shootBottomPID.setP(CollectShooterConstants.kP);
        shootBottomPID.setI(CollectShooterConstants.kI);
        shootBottomPID.setD(CollectShooterConstants.kD);
        shootBottomPID.setIZone(CollectShooterConstants.kIz);
        shootBottomPID.setFF(CollectShooterConstants.kFF);
        shootBottomPID.setOutputRange(CollectShooterConstants.kMinOutput, CollectShooterConstants.kMaxOutput);

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

        joyCollector = -1 * Modifiers.withDeadband(OperatorXbox.getLeftX(), 0.1);
        joyShooter =Modifiers.withDeadband(OperatorXbox.getRightX(), 0.1);   
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
                    setState(State_CS.NOTE_DETECTED);
                }

                break;

            case State_CS.COLLECTOR_INTAKE:
 
                // code to run the intake motors until the note is detected goes here
                setCollectorSpeed(.8);
                
                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                if (IO.Sensor.isNoteDetected()) {
                    setState(State_CS.NOTE_DETECTED);
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

                break;

            case State_CS.SHOOTER_INTAKE:

                // collect a note from the source
                // code to run the shooter motors in reverse (slowly maybe)
                setShooterSpeed(-.3);

                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                if (IO.Sensor.isNoteDetected()) {
                    setState(State_CS.NOTE_DETECTED);
                }

                if (IO.OI.OperatorHID.getButton(ButtonBOX.INTAKE_OFF)) {        // Shoot Speaker Button Pressed
                    csTarget = CS.kTargetSpeaker;
                    setState(State_CS.OFF);
                    break;
                }

                break;

            case State_CS.NOTE_DETECTED:

                // if the note is in the correct position in the chute, it is ready to be processed
                if (IO.Sensor.getNoteSensor(0)) {
                    setState(State_CS.NOTE_READY);
                }

                break;

            case State_CS.NOTE_READY:

                // stop the collector motor (in case you collected from the ground)
                setCollectorSpeed(0);
                
                // stop the shooter motor (in case you were collecting from the source)
                setShooterSpeed(0);

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
                    setShooterSpeed(1);
                } else {
                    setShooterSpeed(.8);
                }

                // wait a period of time for the robot to eject the note
                if (countLoop > 50) {
                // after the wait - transition back to the stop state
                    setState(State_CS.SHOOT);
                }

                break;

            case State_CS.SHOOT:

                // start the collection motor, to move the note forward
                setCollectorSpeed(.5);

                // wait a period of time for the robot to shoot the note
                if (countLoop > 100) {
                // after the wait - transition back to the stop state
                    setState(State_CS.OFF);
                }
                // after the wait - transition back to the stop state
                setState(State_CS.OFF);

                break;

            case State_CS.EJECT_NOTE:

                // code to run the collector motors in reverse
                setCollectorSpeed(-.5);
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
