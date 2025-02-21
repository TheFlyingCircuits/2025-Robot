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
    
    public Command rightCommand() { // red
        return this.runOnce(() -> {base = red;});
    }

    public Command leftCommand() { // blue
        return this.runOnce(() -> {base = blue;});
    }

    //TODO: change this to just sides of the hexagon

    public Command scoreCompleteCommand() { // Blink White
        return this.runOnce(() -> {base = white.blink(Seconds.of(0.125));}).withTimeout(1);        
    }

    public Command coralControlledCommand() { // white 
        return this.runOnce(() -> {base = white;});
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


    @Override
    public void periodic() {
        base.applyTo(buffer);
        leds.setData(buffer);
    }
}
