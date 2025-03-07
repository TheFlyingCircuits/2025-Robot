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
import frc.robot.Constants.LEDConstants;

public class Leds extends SubsystemBase {
    private AddressableLED leftLeds;
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

        leftLeds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledsPerStrip);

        leftLeds.setLength(buffer.getLength());
        
        base = black; // led Off
        base.applyTo(buffer);
        
        leftLeds.setData(buffer);
        leftLeds.start();
    }

    //mode 
    public Command defaultCommand() {
        return DriverStation.isAutonomous() ? this.runOnce(() -> {base = blue;}) : this.runOnce(() -> {base = black;}); // auto: blue, tele: off
        
    }



    //scoirng

    public void orange() {
        base = orange;
    }

    public void green() {
        base = green;
    }
    
    public void red() { // red
       base = red;
    }

    public void blue() { // blue
        base = blue;
    }
    
    public Command scoreCompleteCommand() { // Blink Yellow
        LEDPattern yellow = LEDPattern.solid(Color.kYellow);
        return this.run(() -> {base = yellow.blink(Seconds.of(0.125));}).withTimeout(1);        
    }

    public Command coralControlledCommand() { //Blink White 
        white = LEDPattern.solid(Color.kWhite);
        return this.run(() -> {base = white.blink(Seconds.of(0.125));}).withTimeout(1);
    }

    // Arm  
    public Command climbAlignedCommand() { // green
        return this.runOnce(() -> {base = green;});
    }

    public Command climbingCommand() { // orange blink
        return this.runOnce(() -> {base = orange.blink(Seconds.of(.6));});
    }

    public Command errorCommand() { // red blink fast
        return this.runOnce(() -> {base = red.blink(Seconds.of(0.125));});
    }

    public void progressBar(double percent) {
        LEDPattern bar = LEDPattern.progressMaskLayer(() -> percent);
        base = bar;
    }

    @Override
    public void periodic() {
        base.applyTo(buffer);
        leftLeds.setData(buffer);
    }
}
