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

    public void setWristNeoVolts(double volts) {

        io.setWristNeoVolts(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("wristInputs", inputs);

        if (homingTimer.advanceIfElapsed(5) && inputs.wristDegreesPerSecond == 0 && Math.abs(inputs.motorOutputVoltage) < 0.1) {
            io.setWristPosition(inputs.absoluteAngleDegrees);
            homingTimer.restart();
        }

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

        outputVolts = MathUtil.clamp(outputVolts, -maxVolts, maxVolts);

        Logger.recordOutput("wrist/desiredVolts", outputVolts);

        io.setWristNeoVolts(outputVolts);
    }

    public void setTargetPositionDegrees(double targetAngleDegrees) {
        setTargetPositionDegrees(targetAngleDegrees, 12);
    }

    public void setTargetPositionDegrees(double targetAngleDegrees, double maxVolts) {

        if (targetAngleDegrees > WristConstants.maxAngleDegrees || targetAngleDegrees < WristConstants.minAngleDegrees) {
            System.out.println("Invalid wrist angle requested!");
            this.maxVolts = 0;
        }

        this.maxVolts = maxVolts;
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
