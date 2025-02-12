package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{
    
    private WristIO io;
    private WristIOInputsAutoLogged inputs;

    private ArmFeedforward  wristNeoFeedForward;

    private PIDController wristNeoPID;
    // might need arm feedforward because of gravity???
    // private SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(WristConstants.kSArmVolts,WristConstants.kVArmVoltsSecondsPerRadian,WristConstants.kSArmVolts);

    private double desiredWristPositionRadians;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        wristNeoFeedForward = new ArmFeedforward(0, 0,0,0);

        wristNeoPID = new PIDController(1,0,0); // radians
        wristNeoPID.setTolerance(Units.degreesToRadians(1)); // radians

    }

    public void setWristNeoVolts(double volts) {

        io.setWristNeoVolts(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("WristInputs", inputs);

        moveToTargetPosition(desiredWristPositionRadians);
    }

    public void setTargetPositionRadians(double targetAngleRadians) {
        
        desiredWristPositionRadians = targetAngleRadians;
    }

    private void moveToTargetPosition(double targetRadians) {
        
        double outputVolts = (wristNeoPID.calculate(inputs.wristAngleRadians, desiredWristPositionRadians));
        //+ wristNeoFeedForward.calculate(desiredWristPositionRadians)

        // if (wristNeoPID.atSetpoint()) {
        //     return;
        // }
        
        io.setWristNeoVolts(outputVolts);

    }

    public Command setTargetPositionCommand (double targetRad) {
        return this.run(() -> {this.setTargetPositionRadians(targetRad);});
    }
}
