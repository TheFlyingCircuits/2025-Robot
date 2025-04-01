// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;

public class Arm {

    ArmIOInputsAutoLogged inputs;
    ArmIO io;

    /** Shoulder subsystem object, separated out in order for commands to function independently of extension. */
    public Shoulder shoulder;
    /** Extension subsystem object, separated out in order for commands to function independently of shoulder. */
    public Extension extension;

    // LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 3); //all units for mech2d are meters
    // LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("pivot", 1, 1);
    // LoggedMechanismLigament2d shoulderBracket;
    // LoggedMechanismLigament2d telescoper;



    public Arm(ArmIO io) {

        this.io = io;
        inputs = new ArmIOInputsAutoLogged();

        shoulder = new Shoulder();
        extension = new Extension();

        // shoulderBracket = mech2dRoot.append(
        //     new LoggedMechanismLigament2d("pivotBracket", ArmConstants.shoulderBracketLengthMeters, 90, 30, new Color8Bit(Color.kDarkOrange)));
        // telescoper = shoulderBracket.append(
        //     new LoggedMechanismLigament2d("telescoper", ArmConstants.minExtensionMeters, -90, 10, new Color8Bit(Color.kSilver)));

    }

    public void toggleIdleMode() {
        this.io.toggleIdleMode();
    }

    
    public class Shoulder extends SubsystemBase {
        
        public Command setTargetAngleCommand(double degrees) {
            return this.run(() -> {
                io.setShoulderTargetAngle(degrees);
            });
        }

        public Command waitForRetraction() { return new WaitUntilCommand(() -> {
            Logger.recordOutput("arm/waitingForRetraction", true);
            return extension.isSafelyRetracted();
        }).finallyDo(() -> Logger.recordOutput("arm/waitingForRetraction", false));}

        /** Waits until extension is retracted before moving the arm to the desired angle. */
        public Command safeSetTargetAngleCommand(double degrees) {
            return waitForRetraction().andThen(this.setTargetAngleCommand(degrees));
        }

        @Override
        //used to call the periodic for the arm as a whole
        public void periodic() {
            armPeriodic();
        }
    }

    public class Extension extends SubsystemBase {

        
        public Command setTargetLengthCommand(double meters) {
            return retractThenHome().onlyIf(() -> {return meters == ArmConstants.minExtensionMeters;})
            .andThen(this.run(() -> {
                io.setExtensionTargetLength(meters);
            }));
        }

        private Command retractThenHome() {
            return this.run(() -> {
                io.setExtensionTargetLength(ArmConstants.minExtensionMeters);
            }).until(() -> {
                return inputs.extensionLengthMeters < (ArmConstants.minExtensionMeters + Units.inchesToMeters(1));
            })
            .andThen(homeExtensionCommand());
        }

        public Command homeExtensionCommand() {
            return this.run(() -> {
                Logger.recordOutput("arm/homingExtension", true);
                io.setExtensionMotorVolts(-1);
            }).until(() -> {
                boolean highCurrent = Math.abs(inputs.extensionStatorCurrent) > 15;
                boolean standingStill = Math.abs(inputs.extensionLengthMetersPerSecond) < 0.005;
                return highCurrent && standingStill;
            }).andThen(new InstantCommand(() -> {
                io.setExtensionEncoderPositionToMin();
            })).finallyDo(() -> {
                Logger.recordOutput("arm/homingExtension", false);
            });
        }

        public boolean isSafelyRetracted() {
            boolean retracted = getExtensionMeters() <= (ArmConstants.minExtensionMeters+0.1);
            boolean steady = getExtensionMetersPerSecond() < 0.1;
            return retracted && steady;
        }

    }
    
    public void setShoulderVoltage(double volts) {
        io.setShoulderMotorVolts(volts);
    }

    public double getShoulderAngleDegrees() {
        return inputs.shoulderAngleDegrees;
    }

    public double getTargetExtensionMeters() {
         return inputs.targetExtensionLengthMeters;
    }

    public double getTargetShoulderAngleDegrees() {
        return inputs.targetShoulderAngleDegrees;
    }

    public double getExtensionMeters() {
        return inputs.extensionLengthMeters;
    }

    public double getExtensionMetersPerSecond() {
        return inputs.extensionLengthMetersPerSecond;
    }

    /**
     * Sets the target extension and angle of the arm to be equal to the one in the armPosition object.
     */
    public void setArmPosition(ArmPosition armPosition) {
        io.setShoulderTargetAngle(armPosition.shoulderAngleDegrees);
        io.setExtensionTargetLength(armPosition.extensionMeters);
    }

    public void setShoulderTargetAngle(double degrees) {
        io.setShoulderTargetAngle(degrees);

        // shoulderBracket.setAngle(90+degrees);
    }

    public void setExtensionTargetLength(double meters) {
        io.setExtensionTargetLength(meters);

        // telescoper.setLength(meters);
    }


    public void armPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("armInputs", this.inputs);

        // Logger.recordOutput("arm/mech2d", this.mech2d);

        // commenting these out because i want mech2d to display desired angle first
        // shoulderBracket.setAngle(90+inputs.shoulderAngleDegrees);
        // telescoper.setLength(inputs.extensionLengthMeters);
    }
}
