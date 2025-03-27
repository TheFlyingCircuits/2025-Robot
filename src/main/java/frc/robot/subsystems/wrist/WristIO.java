package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
     public class WristIOInputs {

        public double wristAngleDegrees = 0.0;
        
        public double wristDegreesPerSecond = 0.0;
        
        public double motorOutputVoltage = 0.0;
        
        public double motorOutputCurrentAmps = 0.0;

        public double analogInputVolts = 0.0;

        public double analogInputVoltsPerSecond = 0.0;

        public double absoluteAngleDegrees = 0.0;
        
        public double absoluteDegreesPerSecond = 0.0;
    }

    public default void updateInputs(WristIOInputs inputs) {};

    public default void setWristNeoVolts(double volts) {};

    public default void setWristPosition(double degrees) {};

    public default void toggleIdleMode() {};
    
}
