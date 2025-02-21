// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class ArmIOKraken implements ArmIO{

    TalonFX leftShoulder = new TalonFX(99);//TODO add device IDs and such
    TalonFX rightShoulder = new TalonFX(99);

    TalonFX frontExtensionMotor = new TalonFX(99);
    TalonFX backExtensionMotor = new TalonFX(99);

    CANcoder leftPivotEncoder = new CANcoder(0);
    CANcoder rightPivotEncoder = new CANcoder(0);
    
    ArmFeedforward shoulderFeedforward = new ArmFeedforward(0, 0, 0);

    public ArmIOKraken() {

        configMotors();
    }

    private void configMotors() {

        CANcoderConfiguration leftPivotConfig = new CANcoderConfiguration();
        leftPivotConfig.MagnetSensor.MagnetOffset = 0; //TODO: get angle offset
        leftPivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        leftPivotEncoder.getConfigurator().apply(leftPivotConfig);

        CANcoderConfiguration rightPivotConfig = new CANcoderConfiguration();
        rightPivotConfig.MagnetSensor.MagnetOffset = 0; //TODO: get angle offset
        rightPivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        leftPivotEncoder.getConfigurator().apply(rightPivotConfig);


        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.CurrentLimits.SupplyCurrentLimit = 60; //TODO: find a good value
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.maxExtensionMeters / ArmConstants.extensionMetersPerMotorRotation;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        extensionConfig.Feedback.SensorToMechanismRatio = ArmConstants.extensionMetersPerMotorRotation;

        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        frontExtensionMotor.getConfigurator().apply(extensionConfig);

        extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        backExtensionMotor.getConfigurator().apply(extensionConfig);



        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.CurrentLimits.SupplyCurrentLimit = 60; //TODO: find a good value
        shoulderConfig.Feedback.SensorToMechanismRatio = ArmConstants.shoulderGearReduction;

        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 1; //rps of the motor
        shoulderConfig.MotionMagic.MotionMagicAcceleration = 1; //rotations per second squared
        shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        


        shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftShoulder.getConfigurator().apply(extensionConfig);

        shoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightShoulder.getConfigurator().apply(extensionConfig);

        leftShoulder.setPosition(leftPivotEncoder.getAbsolutePosition().getValueAsDouble());
        rightShoulder.setPosition(leftPivotEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.extensionLengthMeters = frontExtensionMotor.getPosition().getValueAsDouble();
        inputs.extensionLengthMetersPerSecond = frontExtensionMotor.getVelocity().getValueAsDouble();
        inputs.extensionAppliedVolts = frontExtensionMotor.getMotorVoltage().getValueAsDouble();

        if (leftPivotEncoder.getAbsolutePosition().getStatus() == StatusCode.OK) {
            inputs.shoulderAngleDegrees = leftPivotEncoder.getAbsolutePosition().getValueAsDouble();
            inputs.shoulderVelocityDegreesPerSecond = leftPivotEncoder.getVelocity().getValueAsDouble();
        }
        else if (rightPivotEncoder.getAbsolutePosition().getStatus() == StatusCode.OK) {
            System.out.println("Left pivot encoder not measuring! Using right instead");
            inputs.shoulderAngleDegrees = rightPivotEncoder.getAbsolutePosition().getValueAsDouble();
            inputs.shoulderVelocityDegreesPerSecond = leftPivotEncoder.getVelocity().getValueAsDouble();
        }
        else {
            System.out.println("Both pivot encoders not measuring. Defaulting to shoulder angle of 0");
            inputs.shoulderAngleDegrees = 0;
            inputs.shoulderVelocityDegreesPerSecond = 0;
        }


        
    }

    @Override
    public void setShoulderTargetAngle(double degrees) {
        leftShoulder.setControl(
            new MotionMagicTorqueCurrentFOC(degrees));
        rightShoulder.setControl(new Follower(99, true));
    }

    @Override
    public void setExtensionTargetLength(double meters) {
        frontExtensionMotor.setControl(
            new MotionMagicVoltage(meters));
        backExtensionMotor.setControl(new Follower(99, true));
    }


    //climbing only
    @Override
    public void setShoulderMotorVolts(double volts) {
        leftShoulder.setVoltage(volts);
    }

    //climbing only
    @Override
    public void setExtensionMotorVolts(double volts) {
        frontExtensionMotor.setVoltage(volts);
    }
}
