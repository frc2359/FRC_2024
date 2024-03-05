package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO2;
import frc.robot.RobotMap.CollectShooterConstants.State_CS;
import frc.robot.RobotMap.LEDConstants;

import static frc.robot.RobotMap.LEDConstants.*;

import java.util.OptionalInt;

// Code from Mr. R to control LEDs attached to the Robot. Can be used for system checks, etc

public class LEDs extends SubsystemBase {

    private int stateLEDs = STATE_LEDS_OFF;

    private int colR = 0;
    private int colG = 0;
    private int colB = 0;
    private boolean colBlink = false;
    private int colDelay = 500;
    
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;

    private boolean flagGRB = true;     // Set to True if Leds are GRB otherwise false if RGB
   
    private long startTime = 0;
    private long elapsedTime = 0;
    private double timeLeft = 120;

    private boolean flagBlink = false;
    private int countBlink = 0;

    private int csState = 0;
    private int moveCount = 0;
    private int movePos = 0;

     private void waitMSecs(long delay) {
        startTime = System.currentTimeMillis();
        elapsedTime = 0;
        while (elapsedTime < delay) {
            elapsedTime = System.currentTimeMillis() - startTime;
        }
    }

    public void init() {
        leds = new AddressableLED(PWM_LEDS);
        ledBuffer = new AddressableLEDBuffer(144);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    public void setCol(int cR, int cG, int cB, boolean blink) {
        colR = cR;
        colG = cG;
        colB = cB;
        colBlink = blink;
        stateLEDs = STATE_LEDS_COLOR;
    }

    /** */
    private void setColor(int iStart, int iEnd, int rCol, int gCol, int bCol) {
        for (int i = iStart -1; i < iEnd; i++) {
            setRGB(i, gCol, rCol, bCol);
        }
        //leds.setData(ledBuffer);
    }

    public void initLEDs() {
        setState(STATE_LEDS_INIT);
        runLEDs();
    }

    public void endGame(double tL) {
        timeLeft = tL;
        stateLEDs = STATE_LEDS_COUNTDOWN;
    }

    public void setState (int st) {
        stateLEDs = st;
    }

    public void setState (int st, int cs) {
        stateLEDs = st;
        csState = cs;
    }

    public void setPair(int iP, int cR, int cG, int cB) {
        if (iP>=1 && iP<=31) {
            setColor(iP, iP, cR, cG, cB);
            setColor(62-iP, 62-iP, cR, cG, cB);
        }
    }

    private void setRGB(int iP, int cR, int cG, int cB) {
            ledBuffer.setRGB(iP, cG, cR, cB);
            /** if (flagGRB) {
                ledBuffer.setRGB(iP, cG, cR, cB);
            } else {
                ledBuffer.setRGB(iP, cR, cG, cB);
            }
            */
    }

    public void runLEDs() {
        switch (stateLEDs) {
            case STATE_LEDS_OFF:
                setColor(1, 144, 0, 0, 0);
                break;

            case STATE_LEDS_COLOR:
                if (colBlink) {
                    countBlink++;
                    if (countBlink > 50) {
                        flagBlink = !flagBlink;
                        countBlink = 0;
                    }
                } else {
                    flagBlink = false;
                    countBlink = 0;
                }
                if(!flagBlink) {
                    for (int i = 0; i < 144; i++) {
                        setRGB(i, colR, colG, colB);
                    }
                } else {
                    for (int i=0; i < 144; i++) {
                        setRGB(i, 0, 0, 0);
                    }
                }

                break;

            case STATE_LEDS_INIT:
                setColor(1, 144, 0, 0, 0);
                for (int i = 1; i<=144 ; i++) {
                    setColor(1, i, 255,255,255);
                    waitMSecs(10);
                    //setColor(0, i, i, 0,0,0);
                }
                stateLEDs = STATE_LEDS_OFF;
                break;

            case STATE_LEDS_ALIGN:
                setColor(1, 144, 0, 0, 0);
                setColor(36, 37, 255, 255, 255);
                setColor(56,57,255,255,255);
                setColor(102,103,255,255,255);
                break;
                
            case STATE_LEDS_COUNTDOWN:
                setPair(31, 255, 255, 255);
                for (int i=30; i>0; i--) {
                    if (i>timeLeft) {
                        setPair(31-i, 0, 0, 0);
                    } else {
                        if (i>20) {
                            setPair(31-i, 0, 255, 0);
                        } else if(i>10) {
                            setPair(31-i, 128, 128, 0);
                        } else {
                            setPair(31-i, 255, 0, 0);
                        }
                    }
                }
                break;    

            case STATE_LEDS_STATUS:

                int rdyLvl = 0;

                for (int i=0; i<144; i++) {
                    setRGB(i, 0, 0, 0);
                }
                double batVal = IO2.Status.getBattVoltage();
                 
                if (batVal > 12.5) {
                    setPair(4, 0, 255, 0);
                    setPair(5, 0, 0, 255);
                    rdyLvl = Math.max(rdyLvl, 1);
                }
                if (batVal >= 12.0 && batVal <12.5 ) {
                    setPair(4, 0, 255, 0);
                    setPair(5, 0, 255, 0);
                    rdyLvl = Math.max(rdyLvl, 1);
                }
                if (batVal >= 11.5 && batVal <12.0 ) {
                    setPair(4, 0, 255, 0);
                    setPair(5, 255, 255, 0);
                    rdyLvl = Math.max(rdyLvl, 2);
                }
                if (batVal >= 11.0 && batVal <11.5 ) {
                    setPair(4, 255, 255, 0);
                    setPair(5, 255, 255, 0);
                    rdyLvl = Math.max(rdyLvl, 2);
                }
                if (batVal <11.0 ) {
                    setPair(4, 255, 0, 0);
                    setPair(5, 255, 0, 0);
                    rdyLvl = Math.max(rdyLvl, 3);
                }

                // Status Info

                if (DriverStation.isEnabled()) {
                    setPair(7, 0,255, 0);
                    if (DriverStation.isAutonomous()) {
                        setPair(8, 255, 255, 0);
                    }
                    if (DriverStation.isTeleop()) {
                        setPair(8, 0, 255, 0);
                    }
                    if (DriverStation.isTest()) {
                        setPair(8, 255, 255, 255);
                    }
                    setPair(9, 0,255, 0); 
                }
                if (DriverStation.isDisabled()) {
                    setPair(7, 128,128, 0);
                    setPair(8, 128,128, 0);
                    setPair(9, 128,128, 0); 
                }
                if (DriverStation.isEStopped()) {
                    setPair(7, 255,0, 0);
                    setPair(8, 255,0, 0);
                    setPair(9, 255,0, 0); 
                    rdyLvl = Math.max(rdyLvl, 3);
                }

                // Alliance Info
                int stn = DriverStation.getLocation().getAsInt();
                for (int i = 0; i<stn; i++) {
                    if(IO2.Status.isTeamRed()) {
                        setPair(11+i,255,0,0);
                    }
                    else {
                        setPair(11+i,0,0,255);
                    }
                }
                for (int i=stn; i<3; i++) {
                    setPair(11+i, 128, 128, 128);
                }
                
                if (DriverStation.isFMSAttached()) {
                    setPair(15, 0, 255, 0);
                } else {
                    setPair(15, 255, 0, 0);
                }

                if (DriverStation.isDSAttached()) {
                    setPair(17, 0, 255, 0);
                    rdyLvl = Math.max(rdyLvl, 1);
                } else {
                    setPair(17, 255, 0, 0);
                    rdyLvl = Math.max(rdyLvl, 2);
                }

                if (IO2.Gyro.isNavXAvail()) {
                    setPair(19, 0, 255, 0);
                    rdyLvl = Math.max(rdyLvl, 1);
                } else {
                    setPair(19, 255, 0, 0);
                    rdyLvl = Math.max(rdyLvl, 3);
                }
                // Show Readiness Level
                switch (rdyLvl) {
                    case 0:
                        setColor(29, 33, 128, 128, 128);
                        break;
                    case 1:
                        setColor(29, 33, 0, 255, 0);
                        break;
                    case 2:
                        setColor(29, 33, 128, 128, 0);
                        break;
                    case 3:
                        setColor(29, 33, 255, 0, 0);
                        break;
                }

                break;

            case STATE_LEDS_COLLECT_SHOOT:
                for (int i=0; i<144; i++) {
                    setRGB(i, 0, 0, 0);
                }
                switch(csState) {
                    case State_CS.OFF:
                        setColor(15, 90, 255, 255, 255);
                        break;
                    
                    case State_CS.COLLECTOR_INTAKE:
                        setColor(130, 144, 255, 255, 0);
                        moveCount++;
                        if (moveCount > 3) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 25) {
                                movePos = 0;
                            }
                        }
                        setColor(110 - movePos, 129 - movePos, 255, 95, 0);
                        break;
 
                    case State_CS.EJECT_NOTE:
                        //setColor(57, 61, 255, 95, 0);
                        moveCount++;
                        if (moveCount > 5) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 15) {
                                movePos = 0;
                            }
                        }
                        setColor(41 + movePos, 45 + movePos, 255, 95, 0);
                        break;

                    case State_CS.SHOOTER_INTAKE:
                        setColor(36, 75, 255, 95, 0);
                        moveCount++;
                        if (moveCount > 3) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 15) {
                                movePos = 0;
                            }
                        }
                        setColor(77 +  movePos, 96 + movePos, 255, 95, 0);
                        break;

                    case State_CS.ALIGN_NOTE:
                        //setColor(41, 55, 255, 95, 0);
                        setColor(35, 105, 255, 95, 0);
                        break;

                    case State_CS.NOTE_READY:
                        setColor(10, 130, 0, 255, 0);
                        //setColor(15,25,0,255,0);
                        break;

                    case State_CS.PREPARE_TO_SHOOT:
                    case State_CS.SHOOT:
                        setColor(130, 144, 0, 255, 0);
                        moveCount++;
                        if (moveCount > 1) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 20) {
                                movePos = 0;
                            }
                        }
                        setColor(130 - movePos, 110 - movePos, 255, 95, 0);
                        break;

                }

                break;    

            case STATE_LEDS_AUTO:
                break;
            }
       leds.setData(ledBuffer);
    }
}