package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    
    private WristIO io;
    private WristIOInputsAutoLogged inputs;

    private PIDController wristNeoPID;

    private double desiredWristPositionDegrees;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        wristNeoPID = new PIDController(0.3,0,0); // kp has units of volts per degree
        wristNeoPID.setTolerance(1); // degrees

    }

    public void setWristNeoVolts(double volts) {

        io.setWristNeoVolts(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("WristInputs", inputs);

        moveToTargetPosition(desiredWristPositionDegrees);
    }

    public void setTargetPositionDegrees(double targetAngleDegrees) {
        
        desiredWristPositionDegrees = targetAngleDegrees;
    }

    private void moveToTargetPosition(double targetDegrees) {
        
        double outputVolts = (wristNeoPID.calculate(inputs.wristAngleDegrees, desiredWristPositionDegrees));

        // if (wristNeoPID.atSetpoint()) {
        //     return;
        // }
        
        io.setWristNeoVolts(outputVolts);

    }

    public Command setTargetPositionCommand (double targetDegrees) {
        return this.run(() -> {this.setTargetPositionDegrees(targetDegrees);});
    }
}
