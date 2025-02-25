package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
     public class WristIOInputs {

        public double wristAngleDegrees = 0.0;
        
        public double wristDegreesPerSecond = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {};

    public default void setWristNeoVolts(double volts) {};
    
}
