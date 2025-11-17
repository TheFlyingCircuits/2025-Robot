package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs;

    private ArmFeedforward armFeedForward;
    private TrapezoidProfile armProfile;
    private PIDController armPID;

    private double desiredArmPosDeg;

    Timer timer;

    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIOInputsAutoLogged();
        timer = new Timer();

        armFeedForward = new ArmFeedforward(ArmConstants.kSArmVolts, ArmConstants.kGArmVolts,ArmConstants.kVArmVoltsSecondsPerRadian, ArmConstants.kAArmVoltsSecondsSquaredPerRadian);
        armProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmConstants.maxArmVelocityRadianPerSecond, ArmConstants.maxArmAccelerationRadianPerSecondSquared));
        
        armPID = new PIDController(ArmConstants.kPArmVoltsPerRadian, 0, ArmConstants.kDArmVoltsPerRadianPerSecond);
        armPID.setTolerance(0.015);
        
        timer.start();
    }

    @Override
    public void periodic() {
        inputs.armDesiredAngleDegrees = desiredArmPosDeg;
        io.updateInputs(inputs);

        goToDesiredPivotAngle();
    }

    private void setArmVolts(double volts) {
        io.setArmVolts(volts);
    }

    private void setArmTargetAngleDeg(double targetDeg) {
        if((targetDeg > ArmConstants.maxAngleDegrees) || (targetDeg < ArmConstants.minAngleDegrees)) {
            System.out.println("desired arm position out of range!!!");
            return;
        }
        desiredArmPosDeg = targetDeg;
    }

    private void goToDesiredPivotAngle() {
        TrapezoidProfile.State profileSetpointVelRadPerSec = armProfile.calculate(0.02, new TrapezoidProfile.State(Units.degreesToRadians(inputs.armAngleDeg), Units.degreesToRadians(inputs.armVelDegPerSec)),
             new TrapezoidProfile.State(Units.degreesToRadians(inputs.armDesiredAngleDegrees), 0));

        double pIDOutputVolts = armPID.calculate(Units.degreesToRadians(inputs.armAngleDeg), Units.degreesToRadians(inputs.armDesiredAngleDegrees));
        double setpointVel = profileSetpointVelRadPerSec.velocity;
        if(Math.abs(inputs.armDesiredAngleDegrees - inputs.armAngleDeg) < 2) {
            setpointVel = 0;
        } 
        // get acceleration by delta velcocity from thisloop-last loop divided by delta time from thislooptime-lastlooptime
        double feedForwardVolts = armFeedForward.calculate(Units.degreesToRadians(inputs.armAngleDeg), setpointVel);

        Logger.recordOutput("goToDesiredPos/ pidOutputVolts", pIDOutputVolts);
        Logger.recordOutput("goToDesiredPos/ feedForwardVolts", feedForwardVolts);
        Logger.recordOutput("goToDesiredPos/ setpointVelocity", Units.radiansToDegrees(profileSetpointVelRadPerSec.velocity));

        double outputVoltsCombined = pIDOutputVolts + feedForwardVolts;

        setArmVolts(outputVoltsCombined);
    }

    private void defaultFunction() {
        desiredArmPosDeg = 90.0;
    }

    public Command setArmTargetAngleDegCommand(double angleDeg) {
        return this.run(() -> setArmTargetAngleDeg(angleDeg));
    }

    public Command armDefaultCommand(double angleDeg) {
        return this.run(() -> defaultFunction());
    } 
    
}
