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

    private int numLEDs = 144;
    private boolean flagGRB = false;     // Set to True if Leds are GRB otherwise false if RGB
   
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

    public class LEDColorRGB {
        private int cR = 0;
        private int cG = 0;
        private int cB = 0;
        //constructor
        public void LEDColorRGB(int r, int g, int b) {
            cR = r;
            cG = g;
            cB = b;
        }
        public void LEDColorRGB() {
            cR = 0;
            cG = 0;
            cB = 0;
        }

        public void set(int r, int g, int b) {
            this.cR = r;
            this.cG = g;
            this.cB = b;
        }
        public int getRed() {
            return this.cR;
        }
        public int getGreen() {
            return this.cG;
        }
        public int getBlue() {
            return this.cB;
        }
    }
    public LEDColorRGB kColorBlack = new LEDColorRGB();
    public LEDColorRGB kColorWhite = new LEDColorRGB();
    public LEDColorRGB kColorPurple = new LEDColorRGB();
    public LEDColorRGB kColorPink = new LEDColorRGB();
    public LEDColorRGB kColorOrange = new LEDColorRGB();
    public LEDColorRGB kColorYellow= new LEDColorRGB();
    public LEDColorRGB kColorGreen = new LEDColorRGB();
    public LEDColorRGB kColorBlue = new LEDColorRGB();
    public LEDColorRGB kColorRed = new LEDColorRGB();

    public void init() {
        kColorBlack.set(0,0,0);
        kColorWhite.set(245,95,65);
        kColorPurple.set(156,0,205);
        kColorPink.set(255,0,46);
        kColorOrange.set(255,15,0);
        kColorYellow.set(255,146,0);
        kColorGreen.set(0,255,20);
        kColorRed.set(255,0,0);
        kColorBlue.set(0,0,255);

        leds = new AddressableLED(PWM_LEDS);
        ledBuffer = new AddressableLEDBuffer(144);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    public void initLEDs() {
        setState(STATE_LEDS_INIT);
        executePeriodic();
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

    private void setColor(int rCol, int gCol, int bCol) {
        for (int i = 0; i < numLEDs; i++) {
            setRGB(i, gCol, rCol, bCol);
        }
        //leds.setData(ledBuffer);
    }

    private void setColor(int iStart, int iEnd, LEDColorRGB col) {
        setColor(iStart,iEnd,col.getRed(),col.getGreen(),col.getBlue());
    }

    private void setColor(LEDColorRGB col) {
        setColor (1, numLEDs, col);
    }

    public void testLEDs() {
        //setColor(1,19,245,75,55);
        setColor(1,14,kColorWhite);
        //setColor(20,40,156,0,205);
        setColor(15,29,kColorPurple);
        //setColor(41,59,255,0,46);
        setColor(30,44,kColorPink);        
        //setColor(60,80,255,15,0);
        setColor(45,59,kColorOrange);
        //setColor(81,99,255,146,0);
        setColor(60,74,kColorYellow);
       // setColor(100,117,0,255,20);
        setColor(75,89,kColorGreen);
        setColor(90,104,kColorRed);
        setColor(105,119,kColorBlue);
        leds.setData(ledBuffer);
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
        if (iP > 0 && iP <= numLEDs /2) {
            setColor(iP, iP, cR, cG, cB);
            setColor(numLEDs+1-iP, numLEDs+1-iP, cR, cG, cB);
        }
    }

    public void setPair(int iP, LEDColorRGB col) {
        setPair(iP, col.getRed(), col.getGreen(), col.getBlue());
    }

    private void setRGB(int iP, int cR, int cG, int cB) {
            if (iP > 0 && iP <= numLEDs) {
                if (flagGRB) {
                    ledBuffer.setRGB(iP, cG, cR, cB);
                } else {
                    ledBuffer.setRGB(iP, cR, cG, cB);
                }
            }          
    }

    public void executePeriodic() {
        switch (stateLEDs) {
            case STATE_LEDS_OFF:
                setColor(kColorBlack);
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
                    setColor(colR, colG, colB);
                } else {
                    setColor(kColorBlack);
                }
                break;

            case STATE_LEDS_INIT:
                setColor(1, 144, 0, 0, 0);
                for (int i = 1; i<=144 ; i++) {
                    setColor(1, i, 255,255,255);
                    waitMSecs(5);
                }
                waitMSecs(20);
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

                setColor(kColorBlack);
                double batVal = IO2.Status.getBattVoltage();
                 
                // Battery Level
                if (batVal > 12.5) {
                    setPair(4, kColorGreen);
                    setPair(5, kColorBlue);
                    rdyLvl = Math.max(rdyLvl, 1);
                }
                if (batVal >= 12.0 && batVal <12.5 ) {
                    setPair(4, kColorGreen);
                    setPair(5, kColorGreen);
                    rdyLvl = Math.max(rdyLvl, 1);
                }
                if (batVal >= 11.5 && batVal <12.0 ) {
                    setPair(4, kColorGreen);
                    setPair(5, kColorYellow);
                    rdyLvl = Math.max(rdyLvl, 2);
                }
                if (batVal >= 11.0 && batVal <11.5 ) {
                    setPair(4, kColorYellow);
                    setPair(5, kColorYellow);
                    rdyLvl = Math.max(rdyLvl, 2);
                }
                if (batVal <11.0 ) {
                    setPair(4, kColorRed);
                    setPair(5, kColorRed);
                    rdyLvl = Math.max(rdyLvl, 3);
                }

                // Status Info
                if (DriverStation.isEnabled()) {
                    setPair(7, kColorGreen);
                    if (DriverStation.isAutonomous()) {
                        setPair(8, kColorYellow);
                    }
                    if (DriverStation.isTeleop()) {
                        setPair(8, kColorGreen);
                    }
                    if (DriverStation.isTest()) {
                        setPair(8, kColorWhite);
                    }
                    setPair(9, kColorGreen); 
                }
                if (DriverStation.isDisabled()) {
                    setPair(7, kColorYellow);
                    setPair(8, kColorYellow);
                    setPair(9, kColorYellow); 
                }
                if (DriverStation.isEStopped()) {
                    setPair(7, kColorRed);
                    setPair(8, kColorRed);
                    setPair(9, kColorRed); 
                    rdyLvl = Math.max(rdyLvl, 3);
                }

                // Alliance Info
                int stn = DriverStation.getLocation().getAsInt();
                for (int i = 0; i<stn; i++) {
                    if(IO2.Status.isTeamRed()) {
                        setPair(11+i,kColorRed);
                    }
                    else {
                        setPair(11+i,kColorBlue);
                    }
                }
                for (int i=stn; i<3; i++) {
                    setPair(11+i, kColorPurple);
                }
                
                if (DriverStation.isFMSAttached()) {
                    setPair(15, kColorGreen);
                } else {
                    setPair(15, kColorRed);
                }

                if (DriverStation.isDSAttached()) {
                    setPair(17, kColorGreen);
                    rdyLvl = Math.max(rdyLvl, 1);
                } else {
                    setPair(17, kColorRed);
                    rdyLvl = Math.max(rdyLvl, 2);
                }

                if (IO2.Gyro.isNavXAvail()) {
                    setPair(19, kColorGreen);
                    rdyLvl = Math.max(rdyLvl, 1);
                } else {
                    setPair(19,kColorRed);
                    rdyLvl = Math.max(rdyLvl, 3);
                }
                // Show Readiness Level
                switch (rdyLvl) {
                    case 0:
                        setColor(29, 33, kColorPurple);
                        break;
                    case 1:
                        setColor(29, 33, kColorGreen);
                        break;
                    case 2:
                        setColor(29, 33, kColorYellow);
                        break;
                    case 3:
                        setColor(29, 33, kColorRed);
                        break;
                }

                break;

            case STATE_LEDS_COLLECT_SHOOT:
                setColor(kColorBlack);
                switch(csState) {
                    case State_CS.OFF:
                        setColor(15, 90, kColorWhite);
                        break;
                    
                    case State_CS.COLLECTOR_INTAKE:
                        setColor(130, 144, kColorYellow);
                        setColor(1,9,kColorYellow);
                        moveCount++;
                        if (moveCount > 3) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 25) {
                                movePos = 0;
                            }
                        }
                        setColor(110 - movePos, 129 - movePos, kColorYellow);
                        setColor(10, 10+movePos, kColorYellow);
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
                        setColor(41 + movePos, 45 + movePos, kColorYellow);
                        break;

                    case State_CS.SHOOTER_INTAKE:
                        setColor(36, 75, kColorYellow);
                        moveCount++;
                        if (moveCount > 3) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 15) {
                                movePos = 0;
                            }
                        }
                        setColor(77 +  movePos, 96 + movePos, kColorYellow);
                        break;

                    case State_CS.ALIGN_NOTE:
                        //setColor(41, 55, 255, 95, 0);
                        setColor(1, 105, kColorOrange);
                        break;

                    case State_CS.NOTE_READY:
                        setColor(1, 144, kColorGreen);
                        break;

                    case State_CS.PREPARE_TO_SHOOT:
                    case State_CS.SHOOT:
                        setColor(130, 144, kColorGreen);
                        moveCount++;
                        if (moveCount > 1) {
                            moveCount = 0;
                            movePos++;
                            if (movePos > 20) {
                                movePos = 0;
                            }
                        }
                        setColor(130 - movePos, 110 - movePos, kColorGreen);
                        break;
                }

                break;    

            case STATE_LEDS_AUTO:
                break;
            }
       leds.setData(ledBuffer);
    }
}