package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class Leds extends SubsystemBase{
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private Timer ledTimer= new Timer();
    public Leds() {
        leds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledsPerStrip);

        leds.setLength(buffer.getLength());
        
        leds.setData(buffer);
        leds.start();
    }
    public void setHSVFunction(int H, int S, int V) {
        for (int i = 0; i < buffer.getLength(); i += 1) {
            buffer.setHSV(i, H, S, V);
        }
        leds.setData(buffer);
    }
    public void setRGB(int R, int G, int B) {
        for (int i = 0; i < buffer.getLength(); i+=1) {
            buffer.setHSV(i,R,G,B);
        }
        leds.setData(buffer);
    }

    public void lowHighHSV() {
        // reset timer
        // 117 
        // 87
        double ledNumber = ledTimer.get() * 30;
        if (ledTimer.get() < 1 ) {
            setHSVFunction(117 - (int)ledNumber, 255, 255);
        } else if ((ledTimer.get() >= 1) && (ledTimer.get() < 2) ) {
            setHSVFunction(87 + ((int)ledNumber - 30), 255, 255);
        } else {
            ledTimer.reset();
        }
    }


    
}
