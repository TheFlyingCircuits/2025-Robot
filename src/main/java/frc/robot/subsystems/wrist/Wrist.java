package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

        wristNeoPID = new PIDController(0.40,0,0); // kp has units of volts per degree
        wristNeoPID.setTolerance(1); // degrees

    }

    public void setWristNeoVolts(double volts) {

        io.setWristNeoVolts(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("wristInputs", inputs);

        moveToTargetPosition(desiredWristPositionDegrees);
    }


    public double getWristAngleDegrees() {
        return inputs.wristAngleDegrees;
    }

    public double getTargetWristDegrees() {
        return desiredWristPositionDegrees;
    }

    private void moveToTargetPosition(double targetDegrees) {
        
        double outputVolts = (wristNeoPID.calculate(inputs.wristAngleDegrees, desiredWristPositionDegrees));

        outputVolts = MathUtil.clamp(outputVolts, -6, 6);

        // if (wristNeoPID.atSetpoint()) {
        //     return;
        // }
        
        io.setWristNeoVolts(outputVolts);

    }

    public void setTargetPositionDegrees(double targetAngleDegrees) {
        
        desiredWristPositionDegrees = targetAngleDegrees;
    }

    public Command setTargetPositionCommand (double targetDegrees) {
        return this.run(() -> {this.setTargetPositionDegrees(targetDegrees);});
    }
}
