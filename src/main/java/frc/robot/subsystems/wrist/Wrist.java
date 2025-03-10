package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    
    private WristIO io;
    private WristIOInputsAutoLogged inputs;

    private PIDController wristNeoPID;

    private double desiredWristPositionDegrees;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        wristNeoPID = new PIDController(0.25,0,0); // kp has units of volts per degree
        wristNeoPID.setTolerance(1); // degrees

    }

    public void setWristNeoVolts(double volts) {

        io.setWristNeoVolts(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("wristInputs", inputs);

        moveToTargetPosition();

        Logger.recordOutput("wrist/desiredWristPositionDegrees", desiredWristPositionDegrees);
    }


    public double getWristAngleDegrees() {
        return inputs.absoluteAngleDegrees;
    }

    public double getTargetWristDegrees() {
        return desiredWristPositionDegrees;
    }

    private void moveToTargetPosition() {
        
        double outputVolts = (wristNeoPID.calculate(inputs.absoluteAngleDegrees, desiredWristPositionDegrees));
        // if (wristNeoPID.atSetpoint()) {
        //     outputVolts = 0;
        // }
        // wrist is feedback only, so we need some output to fight gravity.

        double maxVolts = 11;
        outputVolts = MathUtil.clamp(outputVolts, -maxVolts, maxVolts);

        Logger.recordOutput("wrist/desiredVolts", outputVolts);

        io.setWristNeoVolts(outputVolts);
    }


    public void setTargetPositionDegrees(double targetAngleDegrees) {
        desiredWristPositionDegrees = targetAngleDegrees;
    }

    public Command setTargetPositionCommand(double targetDegrees) {
        return this.run(() -> {
            this.setTargetPositionDegrees(targetDegrees);
        });
    }
}
