package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.LifterConstants.PIDConstants;
import frc.robot.RobotMap.LifterConstants.CANID;
import frc.robot.RobotMap.LifterConstants.LifterStates;


public class LifterSubsystem extends SubsystemBase {
    private double speedLifterLeft = 0;
    private double speedLifterRight = 0;
    
    private int state = 0;

    private CANSparkMax left = new CANSparkMax(CANID.kLeft, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = left.getEncoder();;
    private CANSparkMax right = new CANSparkMax(CANID.kRight, MotorType.kBrushless);
    private RelativeEncoder rightEncoder = right.getEncoder();

    
    //private SparkPIDController leftPID;
    //private SparkPIDController rightPID;


    /** Initialize systems to set constants and defaults */
    public void init() {
        left.restoreFactoryDefaults();
        left.clearFaults();
        left.setIdleMode(IdleMode.kBrake);    

        right.restoreFactoryDefaults();
        right.clearFaults();
        right.setIdleMode(IdleMode.kBrake);
    }

    public Double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public Double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public void setLifterLeftSpeed(double spdNew) {
        speedLifterLeft = spdNew;
        left.set(speedLifterLeft);
    }

    public void setLifterRightSpeed(double spdNew) {
        speedLifterRight = spdNew;
        right.set(speedLifterRight);
    }

    public int stateMachine( int state ) {
        switch(state) {
            case LifterStates.UNKNOWN:
                break;

            case LifterStates.MANUAL:
                // need to determine optimal controls here
                break;
            
            case LifterStates.OFF:
                setLifterLeftSpeed(0);
                setLifterRightSpeed(0);
                break;
            
            case LifterStates.LIFT_LINEAR:
                // need to know what fully up case is
                // way to determine dist to fully up so that we can
                // have pid-like velocity control?

                /*
                 * if(!fullyup) {
                 * setLifterLeftSpeed( some fn of dist );
                 * setLifterRightSpeed( some fn of dist );
                 * }
                 */
        }
        return state;
    }
}
