package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    private double desiredWristPositionDegrees;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        wristNeoFeedForward = new SimpleMotorFeedforward(0, 0);

        wristNeoPID = new PIDController(1,0,0);
        wristNeoPID.setTolerance(1); // should be degrees because of encoder output I think

    }

    public void setFrontRollerRPM(double rpm) {

        double feedforwardVolts = wristNeoFeedForward.calculate(rpm);
        double pidVolts = wristNeoPID.calculate(inputs.wristRpm, rpm);

        io.setWristNeoVolts(feedforwardVolts + pidVolts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("WristInputs", inputs);

        Logger.recordOutput("WristRPM", inputs.wristRpm);
    }

    public void setTargetPositionDegrees(double targetAngleDegrees) {
        
        desiredWristPositionDegrees = targetAngleDegrees;
    }

    public void moveToTargetPosition(double targetDeg) {

        setTargetPositionDegrees(targetDeg);

        double distance = (desiredWristPositionDegrees - inputs.wristAngleDegrees);
        
        double outputVolts = (wristNeoPID.calculate(distance, 0) + wristFeedForward.calculate(Math.toRadians(desiredWristPositionDegrees)));

        if (wristNeoPID.atSetpoint()) {
            return;
        }
        
        io.setWristNeoVolts(outputVolts);

    }

    public Command moveToTargetPositionCommand (double targetDeg) {
        return this.run(() -> {this.moveToTargetPositionCommand(targetDeg);});
    }
}
