package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    
    private Timer heartbeatTimer = new Timer();

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
        
        black.applyTo(buffer);
        
        leftLeds.setData(buffer);
        leftLeds.start();

        
        heartbeatTimer.restart();
    }

    private void solidColor(Color color) {
        LEDPattern.solid(color).applyTo(buffer);
    }

    public int getAllianceHue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent() || !DriverStation.isDSAttached()) {
            return LEDConstants.Hues.betweenBlueAndRed;
        }
        else if (alliance.get() == Alliance.Blue) {
            return LEDConstants.Hues.blueBumpers;
        }
        else if (alliance.get() == Alliance.Red) {
            return LEDConstants.Hues.redBumpers;
        }
        return LEDConstants.Hues.betweenBlueAndRed;
    }


    public Command heartbeatCommand(double timeBetweenDoubleTaps) {
        return this.runOnce(() -> {heartbeatTimer.restart();}).andThen(
                this.run(() -> {
            //double timeBetweenDoubleTaps = 1.5; // time for one full heatbeat cycle
            double timeBetweenSingleTaps = 0.25; // time between each individual pump within a single heartbeat cycle

            double currentTime = heartbeatTimer.get();
            double k1 = 8; // arbitrary decay time constants that were chosen
            double k2 = 2; // based on what was aesthetically pleasing.
            k2 = 2 * (1.5 / timeBetweenDoubleTaps);

            double normalizedBrightness = 0;
            if (currentTime < timeBetweenSingleTaps) {
                normalizedBrightness = Math.exp(-k1*currentTime);
            }
            else {
                normalizedBrightness = Math.exp(-k2*(currentTime - timeBetweenSingleTaps));
            }

            int hue = this.getAllianceHue();
            int saturation = 255;
            int value = (int)(255 * normalizedBrightness);
            Color color = Color.fromHSV(hue, saturation, value);
            this.solidColor(color);
        }).until(() -> {return heartbeatTimer.get() >= timeBetweenDoubleTaps;})).repeatedly();
    }




    //scoirng

    public void orange() {
        
        orange.applyTo(buffer);
    }

    public void green() {
        green.applyTo(buffer);
    }
    
    public void red() { // red
       red.applyTo(buffer);
    }

    public void blue() { // blue
        blue.applyTo(buffer);
    }
    
    public Command scoreCompleteCommand() { // Blink Yellow
        LEDPattern yellow = LEDPattern.solid(Color.kYellow);
        return this.run(() -> {yellow.blink(Seconds.of(0.125)).applyTo(buffer);}).withTimeout(1);        
    }

    public Command coralControlledCommand() { //Blink White 
        white = LEDPattern.solid(Color.kWhite);
        return this.run(() -> {white.blink(Seconds.of(0.125)).applyTo(buffer);}).withTimeout(1);
    }


    public void progressBar(double percent) {
        LEDPattern bar = LEDPattern.progressMaskLayer(() -> percent);
        bar.applyTo(buffer);
    }

    @Override
    public void periodic() {
        leftLeds.setData(buffer);
    }
}
