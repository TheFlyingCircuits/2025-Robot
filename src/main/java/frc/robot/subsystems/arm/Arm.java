// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Arm extends SubsystemBase {

    ArmIOInputsAutoLogged inputs;
    ArmIO io;

    LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 3); //all units for mech2d are meters
    LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("pivot", 1, 1);
    LoggedMechanismLigament2d shoulderBracket;
    LoggedMechanismLigament2d telescoper;



    public Arm(ArmIO io) {

        this.io = io;
        inputs = new ArmIOInputsAutoLogged();

        shoulderBracket = mech2dRoot.append(
            new LoggedMechanismLigament2d("pivotBracket", ArmConstants.shoulderBracketLengthMeters, 90, 30, new Color8Bit(Color.kDarkOrange)));
        telescoper = shoulderBracket.append(
            new LoggedMechanismLigament2d("telescoper", ArmConstants.minExtensionMeters, -90, 10, new Color8Bit(Color.kSilver)));

    }

    /**
     * Sets the taret extension and angle of the arm to be equal to the one in the armPosition object.
     */
    public void setArmPosition(ArmPosition armPosition) {
        io.setShoulderTargetAngle(armPosition.shoulderAngleDegrees);
        io.setExtensionTargetLength(armPosition.extensionMeters);
    }

    public void setShoulderTargetAngle(double degrees) {
        io.setShoulderTargetAngle(degrees);
    }

    public void setExtensionTargetLength(double meters) {
        io.setExtensionTargetLength(meters);
    }

    public Command setShoulderTargetAngleCommand(double degrees) {
        return new InstantCommand(() -> {
            System.out.println("echo X");
            io.setShoulderTargetAngle(degrees);
        });
    }

    public Command setExtensionTargetLengthCommand(double meters) {
        return new InstantCommand(() -> {
            System.out.println("echo A");
            io.setExtensionTargetLength(meters);
        });
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("armInputs", this.inputs);

        //TODO: for some reason the @autologoutput annotation doesn't work
        Logger.recordOutput("arm/mech2d", this.mech2d);

        shoulderBracket.setAngle(90+inputs.shoulderAngleDegrees);
        telescoper.setLength(inputs.extensionLengthMeters);
    }
}
