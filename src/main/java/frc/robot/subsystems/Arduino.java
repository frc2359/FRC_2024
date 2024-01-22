package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;




public class Arduino {
    private SerialPort arduinoUSB;

    private String cmd;
    private boolean arduinoFound;

    //private final I2C.Port i2cPort = I2C.Port.kOnboard;

    //private final I2C arduinoI2C = new I2C(Port.kMXP,4);


    /** Arduino Init */
    public Arduino() {

        // m_colorMatcher.addColorMatch(kBlueTarget);
        // m_colorMatcher.addColorMatch(kRedTarget);

        
        // Establish Connection to Arduino
        try {
            arduinoUSB = new SerialPort(9600, SerialPort.Port.kUSB);
            SmartDashboard.putNumber("ard", 0);
            System.out.println("Connected on kUSB!");
            arduinoFound = true;
        } catch (Exception e) {
            System.out.println("Failed to connect on kUSB, trying kUSB1");
            try {
                arduinoUSB = new SerialPort(9600, SerialPort.Port.kUSB1);
                SmartDashboard.putNumber("ard", 1);
                System.out.println("Connected on kUSB1!");
                arduinoFound = true;
            } catch (Exception e1) {
                System.out.println("Failed to connect on kUSB1, trying kUSB2");
                try {
                    arduinoUSB = new SerialPort(9600, SerialPort.Port.kUSB2);
                    System.out.println("Connected on kUSB2!");
                    SmartDashboard.putNumber("ard", 2);
                    arduinoFound = true;
                } catch (Exception e2) {
                    SmartDashboard.putNumber("ard", -1);

                    System.out.println("Failed to connect on kUSB2, all attempts failed.");
                    arduinoFound = false;
                }
            }
         }
        
        SmartDashboard.putBoolean("Arduino", arduinoFound);
        System.out.print("Init nessage - ");
        // System.out.println(writeArduino(testString));
    }  
    
    public boolean writeArduino (String toSend) {
        //boolean success = arduinoI2C.transaction(toSend.getBytes(), toSend.getBytes().length, new byte[0], 0);
        return true; //success;
    }

    public String readArd() {
        if(arduinoFound) {
            return arduinoUSB.readString();
        } else {
            return "Arduino not found. No buffer to read.";
        }
    }

    public boolean isArduinoFound() {
        return arduinoFound;
    }

    public void write(String s) {
        if(arduinoFound) {
            arduinoUSB.writeString(s);
            arduinoUSB.write(new byte[]{0x0A}, 1);
        }
    }

    /**  LED CONTROL */
    public void defineLEDString (int port, int numLEDs) {
       // send to Arduino the port and numLEDs
        cmd = "D"+Integer.toString(port)+","+Integer.toString(numLEDs);
        //System.out.println(cmd);
        if (arduinoFound) {
            arduinoUSB.writeString(cmd);
            arduinoUSB.write(new byte[]{0x0A}, 1);
        }
     }
    
    public void setLEDColor (int port, int colorCode) {
        // send port and colorCode to Arduino
        cmd = "S"+Integer.toString(port)+","+Integer.toString(colorCode);
        //System.out.println(cmd);
        if (arduinoFound) {
            arduinoUSB.writeString(cmd);
            arduinoUSB.write(new byte[]{0x0A}, 1);
        }
    }
}
