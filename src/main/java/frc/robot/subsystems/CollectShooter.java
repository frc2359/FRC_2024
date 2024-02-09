package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.OI.OperatorXbox;
import frc.robot.RobotMap.State_CS;

public class CollectShooter extends SubsystemBase{

    /** CAN IDs -- MOVE TO ROBOTMAP */
    private final int kCANCollector = 10;
    private final int kCANTop = 2;    
    private final int kCANBot = 3;

    /** Collector Motor - CAN Spark Flex - Brushless*/
    private CANSparkFlex collector = new CANSparkFlex(kCANCollector, MotorType.kBrushless);
    
    /** Upper shooter CAN Spark Flex Motor */
    private CANSparkFlex shootTop = new CANSparkFlex(kCANTop, MotorType.kBrushless);
    
    /** Lower shooter CAN Spark Flex Motor */
    private CANSparkFlex shootBottom = new CANSparkFlex(kCANBot, MotorType.kBrushless);

    /** Initialize systems to set constants and defaults */
    public void init() {
        collector.restoreFactoryDefaults();        
        shootTop.restoreFactoryDefaults();
        shootBottom.restoreFactoryDefaults();
        collector.clearFaults();
        shootTop.clearFaults();
        shootBottom.clearFaults();
    }

    /** Basic run function for the shooter wheels -- needs to be moved to a command     */
    public void runShooter() {
        double val1 = (Math.abs(OperatorXbox.getLeftX()) < 0.1 ? 0 : OperatorXbox.getLeftX());
        double val2 = (Math.abs(OperatorXbox.getRightX()) < 0.1 ? 0 : OperatorXbox.getRightX());
        collector.set(val1);
        shootTop.set(val2);        
        shootBottom.set(-val2);

    }

    // Process the current state of the shooter
    // Execute code relevant to the state, and transition to a new state (if needed)
    // Return the value of the state machine after all code has executed
    public int collecterShooterStateMachine(int currentState) {

        SmartDashboard.putNumber("Current State", currentState);

        switch (currentState) {
            case State_CS.UNKNOWN:

                // init the robot

                // transition to a new state
                currentState = State_CS.OFF;
                break;

            case State_CS.OFF:

                // collector motor to zero power
                // shooter motor to zero power

                // when the INTAKE button is pressed - go to COLLECTOR_INTAKE
                // if statement to check for the INTAKE button goes here
                // if true - transition to a new state
                currentState = State_CS.COLLECTOR_INTAKE;

                // when the COLLECT FROM SHOOTER button is pressed - go to EJECT_NOTE
                // else statement to check for the button goes here
                // when true - transition to a new state
                currentState = State_CS.SHOOTER_INTAKE;

                // if the note is detected after the robot is turned on
                // (when pre-loading the note for autonomous)
                // transition to a new state
                currentState = State_CS.NOTE_DETECTED;

                break;

            case State_CS.COLLECTOR_INTAKE:
 
                // code to run the intake motors until the note is detected goes here

                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                currentState = State_CS.NOTE_READY;
                
                break;

            case State_CS.SHOOTER_INTAKE:

                // collect a note from the source
                // code to run the shooter motors in reverse (slowly maybe)

                // if statement for sensors or limit switches goes here
                // if true - transition to a new state
                currentState = State_CS.NOTE_DETECTED;


                break;

            case State_CS.NOTE_DETECTED:

                // if the note is in the correct position in the chute, it is ready to be processed
                currentState = State_CS.NOTE_READY;

                break;

            case State_CS.NOTE_READY:

                // stop the collector motor (in case you collected from the ground)
                // stop the shooter motor (in case you were collecting from the source)

                // when the SHOOT button is pressed - go to PREPARE_TO_SHOOT
                // DO NOT shoot until the wheels are at speed
                // if statement to check for the SHOOT button goes here
                // if true - transition to a new state
                currentState = State_CS.PREPARE_TO_SHOOT;

                /* TODO
                Do we want two different buttons for two different speeds, SPEAKER and AMP
                or will one button do both?

                Decided: 2 buttons for 2 speeds, likely passing the speed parameter to shoot
                This will optimize the states, we won't need different states for SPEAKER vs AMP
                */

                // when the EJECT button is pressed - go to EJECT_NOTE
                // else statement to check for the EJECT button goes here
                // when true - transition to a new state
                currentState = State_CS.EJECT_NOTE;

                break;

            case State_CS.PREPARE_TO_SHOOT:

                // spin the wheels for the shooter and get them up to speed

                // when the motors are finally up to speed, it is time to shoot
                // when true - transition to a new state
                currentState = State_CS.SHOOT;
                break;

            case State_CS.SHOOT:

                // start the collection motor, to move the note forward

                // wait a period of time for the robot to shoot the note

                // after the wait - transition back to the stop state
                currentState = State_CS.OFF;

                break;

            case State_CS.EJECT_NOTE:

                // code to run the collector motors in reverse

                // wait a period of time for the robot to eject the note

                // after the wait - transition back to the stop state
                currentState = State_CS.OFF;

                break;
        }

        SmartDashboard.putNumber("Current State", currentState);
        return currentState; 
    }

}
