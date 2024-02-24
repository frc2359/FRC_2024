package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.LifterConstants.PIDConstants;
import frc.robot.RobotMap.LifterConstants.CANID;
import frc.robot.RobotMap.LifterConstants.LifterStates;


public class LifterSybsystem extends SubsystemBase {
    private double speedLifterLeft = 0;
    private double speedLifterRight = 0;
    
    private int state = 0;

    private CANSparkMax left = new CANSparkMax(CANID.kLeft, MotorType.kBrushless);
    private CANSparkMax right = new CANSparkMax(CANID.kRight, MotorType.kBrushless);
    
    private SparkPIDController leftPID;
    private SparkPIDController rightPID;


    /** Initialize systems to set constants and defaults */
    public void init() {
        left.restoreFactoryDefaults();   
        right.restoreFactoryDefaults();

        left.clearFaults();
        right.clearFaults();
        

        //shootBottom.setInverted(true);
        
        rightPID = right.getPIDController();
        leftPID = left.getPIDController();   

        // set PID coefficients
        leftPID.setSmartMotionMaxVelocity(6000, 0);
        leftPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        leftPID.setP(PIDConstants.kP);
        leftPID.setI(PIDConstants.kI);
        leftPID.setD(PIDConstants.kD);
        leftPID.setIZone(PIDConstants.kIz);
        leftPID.setFF(PIDConstants.kFF);
        leftPID.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

        rightPID.setSmartMotionMaxVelocity(6000, 0);
        rightPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        rightPID.setP(PIDConstants.kP);
        rightPID.setI(PIDConstants.kI);
        rightPID.setD(PIDConstants.kD);
        rightPID.setIZone(PIDConstants.kIz);
        rightPID.setFF(PIDConstants.kFF);
        rightPID.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);

    }

    private void setLifterLeftSpeed(double spdNew) {
        speedLifterLeft = spdNew;
        left.set(-speedLifterLeft);
    }


    private void setLifterRightSpeed(double spdNew) {
        speedLifterRight = spdNew;
        left.set(-speedLifterRight);
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
