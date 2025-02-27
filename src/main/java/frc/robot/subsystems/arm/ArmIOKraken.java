// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import javax.net.ssl.ExtendedSSLSession;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UniversalConstants;

/** Add your docs here. */
public class ArmIOKraken implements ArmIO{

    TalonFX leftShoulder = new TalonFX(ArmConstants.leftShoulderMotorID, UniversalConstants.canivoreName);
    TalonFX rightShoulder = new TalonFX(ArmConstants.rightShoulderMotorID, UniversalConstants.canivoreName);

    TalonFX frontExtensionMotor = new TalonFX(ArmConstants.frontExtensionMotorID, UniversalConstants.canivoreName);
    TalonFX backExtensionMotor = new TalonFX(ArmConstants.backExtensionMotorID, UniversalConstants.canivoreName);

    TalonFXConfiguration extensionConfig;
    TalonFXConfiguration shoulderConfig;

    CANcoder leftPivotEncoder = new CANcoder(ArmConstants.leftPivotEncoderID, UniversalConstants.canivoreName);
    CANcoder rightPivotEncoder = new CANcoder(ArmConstants.rightPivotEncoderID, UniversalConstants.canivoreName);
    
    ArmFeedforward shoulderFeedforward = new ArmFeedforward(0, 0, 0);

    public ArmIOKraken() {

        SmartDashboard.putNumber("extensionkS", 0);
        SmartDashboard.putNumber("extensionkV", 0);
        SmartDashboard.putNumber("extensionkA", 0);
        SmartDashboard.putNumber("extensionkP", 0);

        configMotors();
    }

    private void configMotors() {

        /* CANCODER CONFIG */
        CANcoderConfiguration leftPivotConfig = new CANcoderConfiguration();
        leftPivotConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        leftPivotConfig.MagnetSensor.MagnetOffset = 0.2705078125;
        leftPivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        leftPivotEncoder.getConfigurator().apply(leftPivotConfig);

        CANcoderConfiguration rightPivotConfig = new CANcoderConfiguration();
        leftPivotConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        rightPivotConfig.MagnetSensor.MagnetOffset = 0.182861328125;
        rightPivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        leftPivotEncoder.getConfigurator().apply(rightPivotConfig);


        /* EXTENSION CONFIG */
        extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        extensionConfig.Feedback.SensorToMechanismRatio = 1./ArmConstants.extensionMetersPerMotorRotation;

        extensionConfig.CurrentLimits.StatorCurrentLimit = 50; //TODO: find a good value

        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.maxExtensionMeters;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.minExtensionMeters;

        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        extensionConfig.MotionMagic.MotionMagicAcceleration = 1;

        extensionConfig.Slot0.kS = 0.5;
        extensionConfig.Slot0.kV = 3.2;
        extensionConfig.Slot0.kA = 0;
        extensionConfig.Slot0.kP = 0;

        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        frontExtensionMotor.getConfigurator().apply(extensionConfig);

        extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        backExtensionMotor.getConfigurator().apply(extensionConfig);

        frontExtensionMotor.setPosition(ArmConstants.minExtensionMeters);
        backExtensionMotor.setPosition(ArmConstants.minExtensionMeters);


        /* SHOULDER CONFIG */
        shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.CurrentLimits.StatorCurrentLimit = 0.1; //TODO: find a good value

        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shoulderConfig.Feedback.SensorToMechanismRatio = ArmConstants.shoulderGearReduction;
        shoulderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        shoulderConfig.Feedback.FeedbackRemoteSensorID = 4;

        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 1; //rps of the motor
        shoulderConfig.MotionMagic.MotionMagicAcceleration = 1; //rotations per second squared

        shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.armMaxAngleDegrees * ArmConstants.shoulderGearReduction / 360;
        shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.armMinAngleDegrees * ArmConstants.shoulderGearReduction / 360;

        shoulderConfig.Slot0.kS = 0;
        shoulderConfig.Slot0.kV = 0;
        shoulderConfig.Slot0.kA = 0;
        shoulderConfig.Slot0.kG = 0;
        shoulderConfig.Slot0.kP = 0;
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
        rightShoulder.setControl(new Follower(ArmConstants.leftShoulderMotorID, true));
    }

    @Override
    public void setExtensionTargetLength(double meters) {
        frontExtensionMotor.setControl(
            new MotionMagicVoltage(meters));
        backExtensionMotor.setControl(new Follower(ArmConstants.frontExtensionMotorID, true));
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

    public void setIdleMode(NeutralModeValue mode) {
        frontExtensionMotor.setNeutralMode(mode);
        backExtensionMotor.setNeutralMode(mode);
        leftShoulder.setNeutralMode(mode);
        rightShoulder.setNeutralMode(mode);
    }
}
