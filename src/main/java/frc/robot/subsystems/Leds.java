package frc.robot.subsystems;

<<<<<<< Updated upstream
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
=======
import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    LEDPattern white;
    LEDPattern black;
    LEDPattern base;
    Timer ledTimer = new Timer();
    double starLEDTimes[];
<<<<<<< Updated upstream

    public Leds() {



        green = LEDPattern.solid(Color.kGreen);
        blue = LEDPattern.solid(Color.kBlue);
        orange = LEDPattern.solid(Color.kYellow);

        public class Leds extends SubsystemBase {
        
        leds.setData(buffer);
        leds.start();

        Random random = new Random();
        
        double starLEDTimes[] = new double[buffer.getLength()];
        for (int i = 0; i < starLEDTimes.length; i++) {
            starLEDTimes[i] = random.nextDouble(2);
        }
        this.starLEDTimes=starLEDTimes;
        ledTimer.start();
    }

    public void setHSVFunction(int h, int s, int v) {
        for (int i=0; i<buffer.getLength(); i++) {
            buffer.setHSV(i, h, s, v);
        }
        leds.setData(buffer);
    }

    public void starLEDs(double timeInbetween, int colorHSVNum) {
        double maxBrightnessIncrimentals = 255 / (timeInbetween/2);
        for (int i=0; i<buffer.getLength(); i++) {
            if (ledTimer.get() >= starLEDTimes[i]) {
                double timeDiff = ledTimer.get() - starLEDTimes[i];
                if(timeDiff < (timeInbetween/2)) {
                    timeDiff = timeDiff * maxBrightnessIncrimentals;
                    buffer.setHSV(i, colorHSVNum, (int)timeDiff, (int)timeDiff);
                } else if (timeDiff > timeInbetween) {
                    starLEDTimes[i] = ledTimer.get() + timeInbetween;
                    buffer.setHSV(i, 0, 255, 255);
                } else {
                    double brightnesSubraction = (timeInbetween - timeDiff) * maxBrightnessIncrimentals;
                    buffer.setHSV(i, colorHSVNum, (int)brightnesSubraction, (int)brightnesSubraction);
                }
            } else {
                buffer.setHSV(i, 100, 255, 255);
            }
        }
        leds.setData(buffer);
=======
    Random random;

    public Leds() {
        leds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledsPerStrip);

        leds.setLength(buffer.getLength());
        
        leds.setData(buffer);
        leds.start();
        
        Random random = new Random();

        double starLEDTimes[] = new double[buffer.getLength()];
        this.starLEDTimes=starLEDTimes;
        for (int i = 0; i < starLEDTimes.length; i++) {
            starLEDTimes[i] = random.nextDouble(20);
        }
        ledTimer.start();
>>>>>>> Stashed changes
    }

    //mode 
    public Command defaultCommand() {
<<<<<<< Updated upstream
        return DriverStation.isAutonomous() ? this.runOnce(() -> {base = blue;}) : this.runOnce(() -> {base = black;}); // auto: blue, tele: off
        
        // return DriverStation.isAutonomous() ? this.runOnce(() -> {base = blue;}) : this.runOnce(() -> {base = black;}); // auto: blue, tele: off
        return this.run(() -> {starLEDs(2,25);});
=======
        // return DriverStation.isAutonomous() ? this.runOnce(() -> {base = blue;}) : this.runOnce(() -> {base = black;}); // auto: blue, tele: off
        return this.run(() -> {starLEDs(3,2,120);});
>>>>>>> Stashed changes
    }



<<<<<<< Updated upstream

@@ -86,14 +130,18 @@ public class Leds extends SubsystemBase {
        return this.runOnce(() -> {base = red.blink(Seconds.of(0.125));});
    }

    public Command starLEDCommand() {
        return this.run(() -> {starLEDs(2,25);});
    }

    public void progressBar(double percent) {
        LEDPattern bar = LEDPattern.progressMaskLayer(() -> percent);
        base = bar;
    }

    @Override
    public void periodic() {
        base.applyTo(buffer);
        leds.setData(buffer);
    }
    // @Override
    // public void periodic() {
    //     base.applyTo(buffer);
    //     leds.setData(buffer);
    // }
}
=======
    //scoirng

    public void testing() {
        // for (int i = 0; i<buffer.getLength(); i++){
        //     System.out.println("star time " + i + " " + starLEDTimes[i]);
        // }
        // System.out.println(ledTimer.get());
    }

    public void starLEDs(double timeInbetween, double timeLength, int colorHSVNum) {
        double maxBrightnessIncrimentals = 85 / (timeLength/2);
        for (int i=0; i<buffer.getLength(); i++) {
            if (ledTimer.get() >= starLEDTimes[i]) {
                double timeDiff = ledTimer.get() - starLEDTimes[i];
                if(timeDiff < (timeLength/2)) {
                    // double brightnesSubraction = (timeLength - timeDiff) * maxBrightnessIncrimentals;
                    buffer.setHSV(i, colorHSVNum, (int)(255 - (timeDiff) * maxBrightnessIncrimentals), 255);
                } else if (timeDiff > timeLength) {
                    while (starLEDTimes[i] < ledTimer.get()) {
                        starLEDTimes[i] = starLEDTimes[i] + timeInbetween;
                    }
                    buffer.setHSV(i, 0, 0, 0);
                } else {
                    timeDiff = timeDiff * maxBrightnessIncrimentals;
                    buffer.setHSV(i, colorHSVNum, ((int)(170 + (timeDiff/2)*maxBrightnessIncrimentals)), 255);
                }
            } else {
                buffer.setHSV(i, 0, 0, 0);
            }
        }
        leds.setData(buffer);
    }
    
}
>>>>>>> Stashed changes
