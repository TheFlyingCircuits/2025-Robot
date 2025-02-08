// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ArmIOKraken implements ArmIO{

    TalonFX shoulderOne = new TalonFX(99);//TODO add device IDs and such
    TalonFX shoulderTwo = new TalonFX(99);

    TalonFX extensionMotor = new TalonFX(99);

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.extensionLengthMeters = extensionMotor.getPosition().getValueAsDouble();
        
    }

    @Override
    public void setShoulderTargetAngle(double degrees) {

    }

    @Override
    public void setExtensionTargetLength(double meters) {
        // TODO Auto-generated method stub
        ArmIO.super.setExtensionTargetLength(meters);
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        // TODO Auto-generated method stub
        ArmIO.super.setShoulderMotorVolts(volts);
    }

    @Override
    public void setExtensionMotorVolts(double volts) {
        // TODO Auto-generated method stub
        ArmIO.super.setExtensionMotorVolts(volts);
    }
}
