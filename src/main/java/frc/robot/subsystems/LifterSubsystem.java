package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.LifterConstants.PIDConstants;
import frc.robot.IO2;
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
        //leftEncoder.;

        right.restoreFactoryDefaults();
        right.clearFaults();
        right.setIdleMode(IdleMode.kBrake);
        //rightEncoder.reset();
    }

    public void homeLeftLifter() {
        leftEncoder.setPosition(0);
    }

    public Double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public void homeRightLifter() {
        rightEncoder.setPosition(0);
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
        right.set(-speedLifterRight);
    }

    public int stateMachine( int state ) {
        double spdL = 0;
        double spdR = 0;
        double posL = getLeftPosition();
        double posR = getRightPosition();
        SmartDashboard.putNumber("LiftL", posL);
        SmartDashboard.getNumber("LiftR", posR);
        int pov = IO2.OI.OperatorHID.getJoystickPos();
        
        if (!IO2.OI.OperatorHID.getButton(11)) {
            //homeLeftLifter();
            //homeRightLifter();
            state = LifterStates.OFF;
        }
        
        if (pov != 0) {
            state = LifterStates.MANUAL;
        }
        switch(state) {
            case LifterStates.UNKNOWN:
                break;

            case LifterStates.MANUAL:
                // need to determine optimal controls here
                switch (pov) {
                    case 0:
                        spdL = 0;
                        spdR = 0;
                        break;
                    case 1:
                        spdL = 1;
                        spdR = 1;
                        break;
                    case 2:
                        spdL = 0;
                        spdR = 1;
                        break;
                    case 3:
                        spdL = 0;
                        spdR = 0;
                        break;
                    case 4:
                        spdL = 0;
                        spdR = -1;
                        break;
                    case 5:
                        spdL = -1;
                        spdR = -1;
                        break;
                    case 6:
                        spdL = -1;
                        spdR = 0;
                        break;
                    case 7:
                        spdL = 0;
                        spdR = 0;
                        break;
                    case 8:
                        spdL = 1;
                        spdR = 0;
                        break;
                }
                break;
            
            case LifterStates.OFF:
                spdL = 0;
                spdR = 0;
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
                break;
        }
        setLifterLeftSpeed(spdL * .5);
        setLifterRightSpeed(spdR * .5);
        return state;
    }
}
