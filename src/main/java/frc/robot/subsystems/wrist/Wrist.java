package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    
    private WristIO io;
    private WristIOInputsAutoLogged inputs;

    private PIDController wristNeoPID;

    private double desiredWristPositionDegrees;

    private double maxVolts;

    private Timer homingTimer;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        wristNeoPID = new PIDController(0.25,0,0); // kp has units of volts per degree
        wristNeoPID.setTolerance(1); // degrees

        homingTimer = new Timer();
        homingTimer.restart();

    }

    public void toggleIdleMode() {
        io.toggleIdleMode();
    }

    public void setWristNeoVolts(double volts) {

        io.setWristNeoVolts(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("wristInputs", inputs);

        if (homingTimer.advanceIfElapsed(1)
                && Math.abs(inputs.wristDegreesPerSecond) < 1
                    && inputs.wristAngleDegrees > 135
                        && inputs.absoluteAngleDegrees > 135) {
            io.setWristPosition(inputs.absoluteAngleDegrees);
            homingTimer.restart();
        }

        moveToTargetPosition();

        Logger.recordOutput("wrist/desiredWristPositionDegrees", desiredWristPositionDegrees);
    }


    public double getWristAngleDegrees() {
        return inputs.wristAngleDegrees;
    }

    public double getTargetWristDegrees() {
        return desiredWristPositionDegrees;
    }

    private void moveToTargetPosition() {
        
        double outputVolts = (wristNeoPID.calculate(inputs.wristAngleDegrees, desiredWristPositionDegrees));
        // if (wristNeoPID.atSetpoint()) {
        //     outputVolts = 0;
        // }
        // wrist is feedback only, so we need some output to fight gravity.

        outputVolts = MathUtil.clamp(outputVolts, -maxVolts, maxVolts);

        Logger.recordOutput("wrist/desiredVolts", outputVolts);

        io.setWristNeoVolts(outputVolts);
    }

    public void setTargetPositionDegrees(double targetAngleDegrees) {
        setTargetPositionDegrees(targetAngleDegrees, 12);
    }

    public void setTargetPositionDegrees(double targetAngleDegrees, double maxVolts) {

        this.maxVolts = maxVolts;

        if (targetAngleDegrees > WristConstants.maxAngleDegrees || targetAngleDegrees < WristConstants.minAngleDegrees) {
            System.out.println("Invalid wrist angle requested!");
            this.maxVolts = 0;
            return;
        }

        
        desiredWristPositionDegrees = targetAngleDegrees;
    }

    public Command setTargetPositionCommand(double targetDegrees) {
        return setTargetPositionCommand(targetDegrees, 12);
    }
    
    public Command setTargetPositionCommand(double targetDegrees, double maxVolts) {
        return this.run(() -> {
            this.setTargetPositionDegrees(targetDegrees, maxVolts);
        });
    }
}
