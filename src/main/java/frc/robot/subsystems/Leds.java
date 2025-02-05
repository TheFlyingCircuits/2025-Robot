package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

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
    }
    
    //TODO: FOR ALL, make them into commands

    //mode 

    public Command defaultCommand() {
        return DriverStation.isAutonomous() ? this.runOnce(() -> {base = blue;}).until(null) : this.runOnce(() -> {base = black;}).until(null); // auto: blue, tele: off
        
    }



    //scoirng
    
    public void right() { // red
        base = red;
    }

    public void left() { // blue
       base = blue;
    }

    //TODO: change this to just sides of the hexagon

    public void intakeRunningScoreComplete() { // Blink White
        // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
        base = white.blink(Seconds.of(1.5));
    }

    public void coralControlled() { // white 
        base = white;
    }

    // Arm

    public void armDown() { // orange
        base = orange;
    }
    
  
    public void climbAligned() { // green
        base = green;
    }

    public void climbing() { // orange blink
        base = orange.blink(Seconds.of(.6));

    }

    public void error() { // red blink fast
        base = red.blink(Seconds.of(0.125));
    }


    @Override
    public void periodic() {
        base.applyTo(buffer);
        leds.setData(buffer);

        if (controller.getAButtonPressed()) { // blink white
            intakeRunningScoreComplete();
        }
        else if (controller.getBButtonPressed()) { // white
            coralControlled();
        }
        else if (controller.getXButtonPressed()) { // orange
            armDown();
        }
        else if (controller.getYButtonPressed()) { // green
            climbAligned();
        }
        else if (controller.getLeftBumperButtonPressed()) { // blink orange
            climbing();
        }
        else if (controller.getRightBumperButtonPressed()) { // red
            error();
        }
    }
}
