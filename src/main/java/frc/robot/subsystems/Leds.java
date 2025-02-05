package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class Leds extends SubsystemBase {
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;
    XboxController controller;
    LEDPattern green;
    LEDPattern blue;
    LEDPattern orange;
    LEDPattern red;
    LEDPattern white;
    LEDPattern black;
    LEDPattern base;


    public Leds() {

        green = LEDPattern.solid(Color.kGreen);
        blue = LEDPattern.solid(Color.kBlue);
        orange = LEDPattern.solid(Color.kYellow);
        red = LEDPattern.solid(Color.kRed);
        white = LEDPattern.solid(Color.kWhite);
        black = LEDPattern.solid(Color.kBlack);

        leds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledsPerStrip);

        leds.setLength(buffer.getLength());

        base = black; // led Off
        base.applyTo(buffer);
        
        leds.setData(buffer);
        leds.start();

   



        controller = new XboxController(0);
    }

    //mode 

    

    public void Auto() { // blue
        base = blue;
    }
    public void Tele() {  // black
        base = black;
    }

    //scoirng
    
    public void Right() { // red
        base = red;
    }

    public void Left() { // blue
       base = blue;
    }

    public void IntakeRunningScoreComplete() { // Blink White
        // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
        base = white.blink(Seconds.of(1.5));
    }

    public void CoralControlled() { // white 
        base = white;
    }

    // Arm

    public void ArmDown() { // orange
        base = orange;
    }
    
  
    public void ClimbAligned() { // green
        base = green;
    }

    public void Climbing() { // orange blink
        base = orange.blink(Seconds.of(.6));

    }


    public void Error() { // red blink fast
        base = red.blink(Seconds.of(0.125));
    }


    @Override
    public void periodic() {
        base.applyTo(buffer);
        leds.setData(buffer);

        if (controller.getAButtonPressed()) { // blink white
            IntakeRunningScoreComplete();
        }
        else if (controller.getBButtonPressed()) { // white
            CoralControlled();
        }
        else if (controller.getXButtonPressed()) { // orange
            ArmDown();
        }
        else if (controller.getYButtonPressed()) { // green
            ClimbAligned();
        }
        else if (controller.getLeftBumperButtonPressed()) { // blink orange
            Climbing();
        }
        else if (controller.getRightBumperButtonPressed()) { // red
            Error();
        }
    }
}
