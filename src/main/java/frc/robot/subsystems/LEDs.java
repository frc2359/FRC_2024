package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.RobotMap.LEDConstants.*;

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
   
    private long startTime = 0;
    private long elapsedTime = 0;
    private double timeLeft = 120;

    private boolean flagBlink = false;
    private int countBlink = 0;

     private void waitMSecs(long delay) {
        startTime = System.currentTimeMillis();
        elapsedTime = 0;
        while (elapsedTime < delay) {
            elapsedTime = System.currentTimeMillis() - startTime;
        }
    }

    public void init() {
        leds = new AddressableLED(PWM_LEDS);
        ledBuffer = new AddressableLEDBuffer(61);
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
            ledBuffer.setRGB(i, rCol, gCol, bCol);
        }
        //leds.setData(ledBuffer);
    }

    public void initLEDs() {
        /*
        setColor(1, 61, 0, 0, 0);
        for (int i = 1; i<=61 ; i++) {
            setColor(1, i, 255,255,255);
            waitMSecs(10);
            //setColor(0, i, i, 0,0,0);
        }
        */
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

    public void setPair(int iP, int cR, int cG, int cB) {
        if (iP>=1 && iP<=31) {
            setColor(iP, iP, cR, cG, cB);
            setColor(62-iP, 62-iP, cR, cG, cB);
            //ledBuffer.setRGB(iP-1, cR, cG, cB);
            //ledBuffer.setRGB(61-iP, cR, cG, cB);
        }
    }

    public void runLEDs() {
        switch (stateLEDs) {
            case STATE_LEDS_OFF:
                setColor(1, 61, 0, 0, 0);
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
                    for (int i = 0; i < 61; i++) {
                        ledBuffer.setRGB(i, colR, colG, colB);
                    }
                } else {
                    for (int i=0; i < 61; i++) {
                        ledBuffer.setRGB(i, 0, 0, 0);
                    }
                }

                break;

            case STATE_LEDS_INIT:
                setColor(1, 61, 0, 0, 0);
                for (int i = 1; i<=61 ; i++) {
                    setColor(1, i, 255,255,255);
                    waitMSecs(10);
                    //setColor(0, i, i, 0,0,0);
                }
                stateLEDs = STATE_LEDS_OFF;
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
        }
       leds.setData(ledBuffer);
    }
}