package frc.robot.subsystems.placerGrabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PlacerGrabberSim implements PlacerGrabberIO {

    private Timer timeCoralHeld = new Timer();
    
    @Override
    public void updateInputs(PlacerGrabberIOInputs inputs) {
        boolean leftSees = SmartDashboard.getBoolean("placerGrabberSimLeftSensor", false);
        boolean rightSees = SmartDashboard.getBoolean("placerGrabberSimRightSensor", false);
        SmartDashboard.putBoolean("placerGrabberSimLeftSensor", leftSees);
        SmartDashboard.putBoolean("placerGrabberSimRightSensor", rightSees);
        inputs.leftSensorSeesCoral = leftSees;
        inputs.rightSensorSeesCoral = rightSees;

        if ((leftSees || rightSees) && !timeCoralHeld.isRunning()) {
            timeCoralHeld.restart();
        }

        if (!(leftSees || rightSees)) {
            timeCoralHeld.stop();
            timeCoralHeld.reset();
        }
    }

    @Override
    public void setSideNeoVolts(double volts) {
        if (volts < 0) {
            SmartDashboard.putBoolean("placerGrabberSimLeftSensor", false);
            SmartDashboard.putBoolean("placerGrabberSimRightSensor", false);
        }
    }

    @Override
    public void setFrontNeoVolts(double volts) {
        // only reason to apply volts after we've had coral for a while
        // is to eject it!
        if ( (Math.abs(volts) > 0) && (timeCoralHeld.get() >= 1.0) ) {
            SmartDashboard.putBoolean("placerGrabberSimLeftSensor", false);
            SmartDashboard.putBoolean("placerGrabberSimRightSensor", false);
        }
    }
}
