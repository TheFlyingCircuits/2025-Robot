package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private boolean manualVoltsControl = false;

    public Wrist(WristIO io) {
        this.io = io;
        inputs = new WristIOInputsAutoLogged();

        //kp 0.25 on belted wrist
        wristNeoPID = new PIDController(0.16,0,0); // kp has units of volts per degree
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

        if (!this.manualVoltsControl) {
            moveToTargetPosition();
        }

        Logger.recordOutput("wrist/desiredWristPositionDegrees", desiredWristPositionDegrees);
    }

    private void reHomeWrist() {
        if (homingTimer.advanceIfElapsed(1)
                && Math.abs(inputs.wristDegreesPerSecond) < 1
                    && inputs.wristAngleDegrees > 135
                        && inputs.absoluteAngleDegrees > 135) {
            io.setWristPosition(inputs.absoluteAngleDegrees);
            homingTimer.restart();
        }
    }


    public double getWristAngleDegrees() {
        return inputs.absoluteAngleDegrees;
    }

    public double getTargetWristDegrees() {
        return desiredWristPositionDegrees;
    }

    private void moveToTargetPosition() {
        // wristNeoPID.setP(SmartDashboard.getNumber("wrist/voltsPerDegreeOfError", wristNeoPID.getP()));
        // SmartDashboard.putNumber("wrist/voltsPerDegreeOfError", wristNeoPID.getP());
        
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

    public Command setVoltsCommand() { return this.run(() -> {
        this.manualVoltsControl = true;
        double volts = SmartDashboard.getNumber("wrist/manualVolts", 0);
        SmartDashboard.putNumber("wrist/manualVolts", volts);

        if ((this.getWristAngleDegrees() < 10) && (volts < 0)) {
            volts = 0;
        }
        if ((this.getWristAngleDegrees() > 100) && (volts > 0)) {
            volts = 0;
        }
        io.setWristNeoVolts(volts);


    }).finallyDo(() -> {io.setWristNeoVolts(0); this.manualVoltsControl = false;});}

    public Command setDutyCycleCommand() { return this.run(() -> {
        this.manualVoltsControl = true;
        double volts = SmartDashboard.getNumber("wrist/manualVolts", 0);
        SmartDashboard.putNumber("wrist/manualVolts", volts);

        if ((this.getWristAngleDegrees() < 10) && (volts < 0)) {
            volts = 0;
        }
        if ((this.getWristAngleDegrees() > 100) && (volts > 0)) {
            volts = 0;
        }
        io.setDutyCycle(volts);


    }).finallyDo(() -> {io.setDutyCycle(0); this.manualVoltsControl = false;});}
}
