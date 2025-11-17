package frc.robot.subsystems.arm;


public class ArmIOSim implements ArmIO{

    private double armVelocityDegreesPerSec = 0.0;
    
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armVelDegPerSec = armVelocityDegreesPerSec;
        inputs.armAngleDeg = armVelocityDegreesPerSec * 0.02; 
        // 0.02 for 20ms cycle
    }

    @Override
    public void setArmVolts(double volts) {
        armVelocityDegreesPerSec = volts * 30.0;
    }
}
