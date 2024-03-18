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
import frc.robot.RobotMap.ButtonBOX;
import frc.robot.RobotMap.CollectShooterConstants.State_CS;
import frc.robot.RobotMap.LifterConstants.CANID;
import frc.robot.RobotMap.LifterConstants.LifterStates;


public class LifterSubsystem extends SubsystemBase {
    private double speedLifterLeft = 0;
    private double speedLifterRight = 0;
    
    private int stateLifter = 0;

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
        //left.setInverted(false);
        left.setIdleMode(IdleMode.kBrake);
        //leftEncoder.setInverted(false);
        leftEncoder.setPositionConversionFactor(1);
        leftEncoder.setPosition(0);
        //SmartDashboard.putNumber("LL-EncCpR", leftEncoder.getCountsPerRevolution());
        //SmartDashboard.putNumber("LL-EncPCF",leftEncoder.getPositionConversionFactor());

        right.restoreFactoryDefaults();
        right.clearFaults();
        //right.setInverted(true);
        right.setIdleMode(IdleMode.kBrake);
        //rightEncoder.setInverted(true);
        rightEncoder.setPosition(0);
    }

    public void homeLeftLifter() {
        leftEncoder.setPosition(0);
    }

    public Double getLeftPosition() {
        double encPos  = leftEncoder.getPosition() / 24 * 10;    // 277 is 11.5 inches, so about 24 per inch
        encPos = Math.round(encPos);
        return encPos / 10;      
    }

    public void homeRightLifter() {
        rightEncoder.setPosition(0);
    }

    public Double getRightPosition() {
        double encPos  = rightEncoder.getPosition() / 42 * 10;    // 485 is 11.5 inches, so about 42 per inch
        encPos = Math.round(encPos);
        return -encPos / 10;      // invert direction
    }

    public double getRightRawPos() {
        return rightEncoder.getPosition();
    }

    public void setLifterLeftSpeed(double spdNew) {
        speedLifterLeft = spdNew;
        left.set(speedLifterLeft);
    }

    public void setLifterRightSpeed(double spdNew) {
        speedLifterRight = spdNew;
        right.set(-speedLifterRight);           // invert direction
    }

    public int stateMachine( int state ) {
        stateLifter = state;
        executePeriodic();
        return stateLifter;
    }

    public int executePeriodic() {
        double spdL = 0;
        double spdR = 0;
        double posL = getLeftPosition();
        double posR = getRightPosition();
        SmartDashboard.putNumber("LiftL-Pos", posL);
        //Math.round((inNum * mult )) / mult;
        SmartDashboard.putNumber("LiftR-Pos", posR);
        int pos = IO2.OI.OperatorHID.getJoystickPos();
        
        SmartDashboard.putBoolean("Btn9", IO2.OI.OperatorHID.getButton(9));
        if (!IO2.OI.OperatorHID.getButton(9)) {
            //homeLeftLifter();
            //homeRightLifter();
            stateLifter = LifterStates.OFF;
        }
        // Lifter joystick: Pos = 0 is center; Pos 1 to 8 starts at top and goes clockwise around joystick
        if (pos != 0) {
            stateLifter = LifterStates.MANUAL;
        }
        switch(stateLifter) {
            case LifterStates.UNKNOWN:
                break;

            case LifterStates.MANUAL:
                // need to determine optimal controls here
                switch (pos) {
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
                if (pos == 0) {
                    stateLifter = LifterStates.OFF;
                }
                break;
            
            case LifterStates.OFF:
                spdL = 0;
                spdR = 0;
                if (IO2.OI.OperatorHID.getButton(ButtonBOX.LIFTER_UP)) {
                    stateLifter = LifterStates.LIFTER_UP;
                    break;
                }
                if (IO2.OI.OperatorHID.getButton(ButtonBOX.LIFTER_DOWN)) {
                    stateLifter = LifterStates.LIFTER_DOWN;
                    break;
                }
                break;

            case LifterStates.LIFTER_UP:
                if (posL < 10) {
                    if (posL > 8) {
                        spdL = .5;
                    } else {
                        spdL = 1;
                    }               
                } else {
                    spdL = 0;
                }
                if (posR < 10) {
                    if (posR > 8) {
                        spdR = .5;
                    } else {
                        spdR = 1;
                    }               
                } else {
                    spdR = 0;
                }
                if (posL > 10 && posR > 10) {
                    stateLifter = LifterStates.OFF;
                }
                break;
 
            case LifterStates.LIFTER_DOWN:
                if (posL > 1) {
                    if (posL < 3) {
                        spdL = -.5;
                    } else {
                        spdL = -1;
                    }               
                } else {
                    spdL = 0;
                }
                if (posR > 1) {
                    if (posR < 3) {
                        spdR = -.5;
                    } else {
                        spdR = -1;
                    }               
                } else {
                    spdR = 0;
                }
                if (posL <= 0 && posR <= 0) {
                    stateLifter = LifterStates.OFF;
                }
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
        setLifterRightSpeed(spdR * .85);
        SmartDashboard.putNumber("lifterState", stateLifter);
        return stateLifter;
    }
}
