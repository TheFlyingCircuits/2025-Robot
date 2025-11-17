package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public double armAngleDeg = -90;
        public double armVelDegPerSec = 0;
        public double armAppliedVolts = 0;
        public double armAmps = 0;
        public double armDesiredAngleDegrees = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {};

    public default void setArmVolts(double volts) {};
    
}
