package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {


    // dummy values for the simulator
    double momentOfInertia = 0.00001;
    private FlywheelSim simulatedNeo = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), momentOfInertia, 
    1./WristConstants.gearReduction),  DCMotor.getNEO(1) , 0.004);

    private double angleDegrees = 0;
    private double degreesPerSecond = 0;


    public WristIOSim() {

    }


    public void setWristPosition(double degrees) {
        this.angleDegrees = degrees;
    }

    public void setWristNeoVolts(double volts) {
        simulatedNeo.setInputVoltage(volts);
        // double voltsToSpeed = SmartDashboard.getNumber("wristVoltsToSpeed", 1);
        // SmartDashboard.putNumber("wristVoltsToSpeed", voltsToSpeed);
        double voltsToSpeed = 32; // random tuning, not physical at all!
        this.degreesPerSecond = voltsToSpeed * volts;
    }

    public void updateInputs(WristIOInputs inputs) {
        double deltaT = 0.02;
        simulatedNeo.update(deltaT);

        inputs.motorOutputVoltage = simulatedNeo.getInputVoltage();

        // double degreesPerSecond = Units.radiansToDegrees(simulatedNeo.getAngularVelocityRadPerSec());
        this.angleDegrees += this.degreesPerSecond * deltaT;

        inputs.absoluteAngleDegrees = this.angleDegrees;
        inputs.wristAngleDegrees = this.angleDegrees;
        inputs.absoluteDegreesPerSecond = degreesPerSecond;
        inputs.wristDegreesPerSecond = degreesPerSecond;
    }
}
