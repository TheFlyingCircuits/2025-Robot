package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase{
    
    private WristIO io;
    private WristIOInputsAutoLogged inputs;

    private SimpleMotorFeedforward wristNeoFeedForward;

    private PIDController wristNeoPID;
    // might need arm feedforward because of gravity???
    private SimpleMotorFeedforward wristFeedForward = new SimpleMotorFeedforward(WristConstants.kSArmVolts,WristConstants.kVArmVoltsSecondsPerRadian,WristConstants.kSArmVolts);

    private double desiredWristPositionRadians;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        wristNeoFeedForward = new SimpleMotorFeedforward(0, 0);

        wristNeoPID = new PIDController(1,0,0); // radians
        wristNeoPID.setTolerance(Units.degreesToRadians(1)); // radians

    }

    public void setWristNeoRPM(double rpm) {

        double feedforwardVolts = wristNeoFeedForward.calculate(rpm);
        double pidVolts = wristNeoPID.calculate(inputs.wristRpm, rpm);

        io.setWristNeoVolts(feedforwardVolts + pidVolts);
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

        double distance = (desiredWristPositionRadians - inputs.wristAngleRadians);
        
        double outputVolts = (wristNeoPID.calculate(distance, 0) + wristFeedForward.calculate(Math.toRadians(desiredWristPositionRadians)));

        if (wristNeoPID.atSetpoint()) {
            return;
        }
        
        io.setWristNeoVolts(outputVolts);

    }

    public Command setTargetPositionCommand (double targetDeg) {
        return this.run(() -> {this.setTargetPositionRadians(targetDeg);});
    }
}
