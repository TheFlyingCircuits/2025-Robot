package frc.robot.subsystems.placerGrabber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PlacerGrabberSim implements PlacerGrabberIO {
    public void updateInputs(PlacerGrabberIOInputs inputs) {
        boolean leftSees = SmartDashboard.getBoolean("placerGrabberLeftSim", false);
        boolean rightSees = SmartDashboard.getBoolean("placerGrabberRightSim", false);
        SmartDashboard.putBoolean("placerGrabberLeftSim", leftSees);
        SmartDashboard.putBoolean("placerGrabberRightSim", rightSees);
        inputs.leftSensorSeesCoral = leftSees;
        inputs.rightSensorSeesCoral = rightSees;
    };
}
