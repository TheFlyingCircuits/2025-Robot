package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {

    
    private DCMotor shoulderMotor = DCMotor.getKrakenX60(2).withReduction(ArmConstants.shoulderGearReduction);
    private DCMotor extensionMotor = DCMotor.getKrakenX60(1).withReduction(1);

    public ArmIOSim() {
        
    };

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO Auto-generated method stub
        ArmIO.super.updateInputs(inputs);
    }


    @Override
    public void setArmAccelerationDegreesPerSecondSquared(double degreesPerSecondSquared) {
        // TODO Auto-generated method stub
        ArmIO.super.setArmAccelerationDegreesPerSecondSquared(degreesPerSecondSquared);
    }

    @Override
    public void setArmMotorVolts(double volts) {
        // TODO Auto-generated method stub
        ArmIO.super.setArmMotorVolts(volts);
    }

}
