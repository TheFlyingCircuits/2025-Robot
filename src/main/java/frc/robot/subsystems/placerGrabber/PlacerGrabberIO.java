package frc.robot.subsystems.placerGrabber;

import org.littletonrobotics.junction.AutoLog;

public interface PlacerGrabberIO {
    @AutoLog
    public class PlacerGrabberIOInputs {

        public double frontRollerRPM = 0.0;
        public double sideRollerRPM = 0.0;

        public boolean leftSensorSeesCoral = false;
        public boolean rightSensorSeesCoral = false;
    }

    public default void updateInputs(PlacerGrabberIOInputs inputs) {};

    public default void setFrontNeoVolts(double volts) {};

    public default void setSideNeoVolts(double volts) {};

}
