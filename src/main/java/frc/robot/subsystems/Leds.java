package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.LEDConstants;

public class Leds extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private static boolean putSwitcher = true;

    private Timer heartbeatTimer = new Timer();

    /**
     * TODO: document whole class
     */
    public Leds() {
        leds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledsPerStrip);

        leds.setLength(buffer.getLength());
        
        leds.setData(buffer);
        leds.start();

        heartbeatTimer.restart();
    }

    private void solidColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i += 1) {
            buffer.setLED(i, color);
        }
        leds.setData(buffer);
    }

    private void turnOff() {
        this.solidColor(Color.kBlack);
    }

    private void wipeToNewColor(double progress, Color colorThatEnters) {
        double maxPositionOfOldColor = LEDConstants.stripLengthMeters + (LEDConstants.metersPerLed/2) - (LEDConstants.stripLengthMeters/2)*progress;
        double minPositionOfOldColor = 0 - (LEDConstants.metersPerLed/2) + (LEDConstants.stripLengthMeters/2)*progress;

        for (int i = 0; i < buffer.getLength(); i += 1) {
            double position = (i + 1) * LEDConstants.metersPerLed;

            if (position > maxPositionOfOldColor || position < minPositionOfOldColor) {
                buffer.setLED(i, colorThatEnters);
            }
        }
        leds.setData(buffer);
    }

    /**
     * Draws a color gradient (red -> yellow -> green) on a segment of the LED strip.
     * Intended to be used as a progress-bar-esque indicator for things like
     * flywheel spinup and pivot/drivetrain alignment to a goal.
     * 
     * @param startPositionMeters The position on the LED strip (measured in meters) where the pattern will start (this will be red)
     *                            A position of 0 is the part of the strip that's closest to the strip's power cable.
     *                            A position of stripLengthMeters will be the other end of the strip.
     * 
     * @param endPositionMeters The posiion on the LED strip (measured in meters) where the pattern will end (this will be green)
     *                          A position of 0 is the part of the strip that's closest to the strip's power cable.
     *                          A position of stripLengthMeters will be the other end of the strip.
     * 
     * @param progress A number in the range [0, 1] that indicates how much of the pattern should be drawn.
     *                 0 will have no lights on, 0.5 will have some red and yellow, 1.0 will have the full red -> yellow -> green pattern
     *                 (it will look like a traffic light with all of the lights on.)
     */
    private void showProgressAsTrafficLight(double startPositionMeters, double endPositionMeters, double progress) {
        double startHue = LEDConstants.Hues.redTrafficLight;
        double endHue = LEDConstants.Hues.greenTrafficLight;

        // iterate through all LEDs on the strip
        for (int i = 0; i < buffer.getLength(); i += 1) {

            // Find how far we are along the strip
            double positionAlongStrip = (i + 1) * LEDConstants.metersPerLed;

            // See where the current LED lies within the progress bar segment.
            // 0 = beginning of segment, 1 = end of segment
            double positionWithinSegment = (positionAlongStrip - startPositionMeters) / (endPositionMeters - startPositionMeters);

            // leave the current LED alone if it's not in the segment.
            if (positionWithinSegment < 0 || positionWithinSegment > 1) {
                continue;
            }

            // turn off the LED if it's in the segment,
            // but the progress bar hasn't reached it yet.
            if (positionWithinSegment > progress) {
                buffer.setLED(i, Color.kBlack);
                continue;
            }

            // Color the LED based on it's position within the segment, as well as the given progress.
            // Squaring positionWithinSegent was experimentally found to give more aesthetically
            // pleasing results than simply lerping from startHue to endHue.
            double hue = (endHue - startHue) * Math.pow(positionWithinSegment, 2) + startHue;
            buffer.setHSV(i, (int)hue, 255, 255);
        }
        leds.setData(buffer);
    }

    private void loadingPattern(int numBlobs, Color colorOfEvenBlobs, Color colorOfOddBlobs) {
        // use negative speed to move from the top of the strip to the bottom of the strip.
        double patternSpeedMetersPerSecond = -1.0;
        double blobLength = (LEDConstants.stripLengthMeters/12);
        double blobRadius = blobLength / 2;

        // Start with a blank strip
        for (int i = 0; i < buffer.getLength(); i += 1) {
            buffer.setLED(i, Color.kBlack);
        }

        // draw each blob, starting with the first.
        // blob position is updated at the end of the outer for loop.
        double blobCenter = this.wrapPosition(patternSpeedMetersPerSecond * Timer.getFPGATimestamp());
        for (int blobIndex = 0; blobIndex < numBlobs; blobIndex += 1) {

            Color colorOfBlob = colorOfEvenBlobs;
            if (blobIndex % 2 == 1) {
                colorOfBlob = colorOfOddBlobs;
            }

            double upperBound = this.wrapPosition(blobCenter + blobRadius);
            double lowerBound = this.wrapPosition(blobCenter - blobRadius);

            // find which LEDs are within the current blob
            for (int ledIndex = 0; ledIndex < buffer.getLength(); ledIndex += 1) {
                double position = (ledIndex + 1) * LEDConstants.metersPerLed;

                boolean partOfBlob = (lowerBound <= upperBound) && (position >= lowerBound) && (position <= upperBound);
                
                // wrap around from the end of the strip to the beginning of the strip
                partOfBlob |= (upperBound < lowerBound) && ( (position >= lowerBound) || (position <= upperBound) );

                if (partOfBlob) {
                    buffer.setLED(ledIndex, colorOfBlob);
                }
            }

            // Move to the next blob
            blobCenter = this.wrapPosition(blobCenter + (LEDConstants.stripLengthMeters / numBlobs));
        }

        leds.setData(buffer);
    }

    private double wrapPosition(double positionToWrap) {
        // https://stackoverflow.com/questions/5385024/mod-in-java-produces-negative-numbers
        double wrappedPosition = positionToWrap % LEDConstants.stripLengthMeters;
        if (wrappedPosition < 0) {
            wrappedPosition += LEDConstants.stripLengthMeters;
        }
        return wrappedPosition;
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
        return super.runOnce(() -> {heartbeatTimer.restart();}).andThen(
               super.run(() -> {
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
        }).until(() -> {return heartbeatTimer.get() >= timeBetweenDoubleTaps;}));
    }


    public Command heartbeatCommand() {
        return this.heartbeatCommand(1.5);
    }

    public Command fastHeartbeatCommand() {
        return this.heartbeatCommand(0.53);
    }

    public Command fasterHeartbeatSequence() {
        SequentialCommandGroup output = new SequentialCommandGroup();
        double timeBetweenDoubleTaps = 1.5;
        int n = 1;
        while (timeBetweenDoubleTaps > 0.6) {
            if (timeBetweenDoubleTaps <= 0.8) {
                n += 6;
            }
            for (int i = 0; i < n; i += 1) {
                output.addCommands(this.heartbeatCommand(timeBetweenDoubleTaps));
            }
            timeBetweenDoubleTaps -= 0.2;
        }
        return output;
    }


    public Command playIntakeAnimationCommand(Supplier<Boolean> intakeCamSeesNote) {
        Color orange = Color.fromHSV(LEDConstants.Hues.orangeSignalLight, 255, 255);
        Color green = new Color(0, 255, 0);
        return super.run(() -> {
            if (intakeCamSeesNote.get().booleanValue()) {
                this.loadingPattern(6, orange, green);
            }
            else {
                this.loadingPattern(3, orange, orange);
            }
        });
    }


    public Command solidColorCommand(Color color) {
        return super.run(() -> {this.solidColor(color);});
    }

    public Command solidOrangeCommand() {
        Color orange = Color.fromHSV(LEDConstants.Hues.orangeSignalLight, 255, 255);
        return this.solidColorCommand(orange).withName("solid orange command");
    }

    public Command turnOffCommand() {
        return super.run(this::turnOff);
    }

    public Command strobeCommand(Color color, int numFlashes, double totalRuntimeSeconds) {
        double onTime = (totalRuntimeSeconds / numFlashes) / 2.0;
        double offTime = onTime;

        SequentialCommandGroup output = new SequentialCommandGroup();
        for (int i = 0; i < numFlashes; i += 1) {
            output.addCommands(this.solidColorCommand(color).withTimeout(onTime),
                               this.turnOffCommand().withTimeout(offTime));
        }

        return output.withName("strobe");
    }

    // IDK how I feel about this structure, but at least it works for now.

    public Command playWipeToColorCommand(double animationLengthSeconds, Supplier<Color> color) {
        Timer timer = new Timer();
        return this.runOnce(timer::restart).andThen(this.run(() -> {
            double progress = timer.get() / animationLengthSeconds;
            this.wipeToNewColor(progress, color.get());
        })).until(() -> {return timer.get() > animationLengthSeconds;});
    }

    public Command playWipeToColorCommand(double animationLengthSeconds, Color color) {
        return this.playWipeToColorCommand(animationLengthSeconds, () -> {return color;});
    }


    public Command playWipeToAllianceColorCommand(double animationLengthSeconds) {
        return this.playWipeToColorCommand(animationLengthSeconds, () -> {
            return Color.fromHSV(this.getAllianceHue(), 255, 255);
        });
    }

    public Command playFireNoteAnimationCommand() {
        double timeForEachSegment = 1./5.;
        Color orange = Color.fromHSV(LEDConstants.Hues.orangeSignalLight, 255, 255);
        Color green = new Color(0, 255, 0);

        return this.solidColorCommand(green).withTimeout(0.05)
               .andThen(this.playWipeToColorCommand(timeForEachSegment, orange))
               .andThen(this.playWipeToAllianceColorCommand(timeForEachSegment));
    }

    public Command temporarilySwitchPattern(Supplier<Command> switchSupplier) {
        return new InstantCommand(() -> {
            // System.out.println("starting instant command");
            // Find whatever pattern the LEDs are currently displaying,
            // so that we can return to that pattern after we're done with the new patter.
            Command interruptedPattern = this.getCurrentCommand();
            Command patternToSwitchTo = switchSupplier.get();
            // System.out.println("interrupted name: " + interruptedPattern.getName());
            if (interruptedPattern.equals(patternToSwitchTo)) {
                // don't let a pattern interrupt itself.
                // TODO: use a more robust equality check than .equals(),
                //       becuase it just checks for reference equality by default?
                // System.out.println("rejected");
                return;
            }

            // Schedule the whole sequence.
            // System.out.println("switchToMe: " + patternToSwitchTo.runsWhenDisabled());
            // System.out.println("interruptMe: " + interruptedPattern.runsWhenDisabled());
            Command fullCommand = patternToSwitchTo.andThen(new ScheduleCommand(interruptedPattern));
            fullCommand.setName("switcher");
            fullCommand.addRequirements(this);
            fullCommand.schedule();

            // System.out.println("fullCommand: " + fullCommand.runsWhenDisabled());
            // fullCommand.schedule();
            // System.out.println("done with instant command");
        });
    }
}
