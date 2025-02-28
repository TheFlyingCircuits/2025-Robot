// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import javax.net.ssl.ExtendedSSLSession;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

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

    double targetExtensionMeters = ArmConstants.minExtensionMeters;
    double extensionMeters = ArmConstants.minExtensionMeters;
    double targetShoulderAngleDegrees = 0;
    double shoulderAngleDegrees = 0;
    

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
        rightPivotEncoder.getConfigurator().apply(rightPivotConfig);


        /* EXTENSION CONFIG */
        extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        extensionConfig.Feedback.SensorToMechanismRatio = 1./ArmConstants.extensionMetersPerMotorRotation;

        extensionConfig.CurrentLimits.StatorCurrentLimit = 50; //TODO: find a good value

        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.maxExtensionMeters;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.minExtensionMeters;

        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 2; //mps of the extension
        extensionConfig.MotionMagic.MotionMagicAcceleration = 1; //m/s^2 of the extension

        extensionConfig.Slot0.kS = ArmConstants.kSExtensionVolts;
        extensionConfig.Slot0.kV = ArmConstants.kVExtensionVoltsSecondsPerRadian;
        extensionConfig.Slot0.kA = ArmConstants.kAExtensionVoltsSecondsSquaredPerRadian;
        extensionConfig.Slot0.kP = ArmConstants.kPExtensionVoltsPerMeter;

        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        frontExtensionMotor.getConfigurator().apply(extensionConfig);

        extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        backExtensionMotor.getConfigurator().apply(extensionConfig);

        frontExtensionMotor.setPosition(ArmConstants.minExtensionMeters);
        backExtensionMotor.setPosition(ArmConstants.minExtensionMeters);


        /* SHOULDER CONFIG */
        shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.CurrentLimits.StatorCurrentLimit = 2; //TODO: find a good value

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
        leftShoulder.getConfigurator().apply(shoulderConfig);

        shoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightShoulder.getConfigurator().apply(shoulderConfig);

        leftShoulder.setPosition(leftPivotEncoder.getAbsolutePosition().getValueAsDouble());
        rightShoulder.setPosition(leftPivotEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        extensionMeters = frontExtensionMotor.getPosition().getValueAsDouble();
        inputs.extensionLengthMeters = extensionMeters;
        inputs.extensionLengthMetersPerSecond = frontExtensionMotor.getVelocity().getValueAsDouble();
        inputs.extensionAppliedVolts = frontExtensionMotor.getMotorVoltage().getValueAsDouble();

        if (leftPivotEncoder.getAbsolutePosition().getStatus() == StatusCode.OK) {
            shoulderAngleDegrees = leftPivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
            inputs.shoulderAngleDegrees = shoulderAngleDegrees;
            inputs.shoulderVelocityDegreesPerSecond = leftPivotEncoder.getVelocity().getValueAsDouble() * 360;
        }
        else if (rightPivotEncoder.getAbsolutePosition().getStatus() == StatusCode.OK) {
            System.out.println("Left pivot encoder not measuring! Using right instead");
            shoulderAngleDegrees = rightPivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
            inputs.shoulderAngleDegrees = shoulderAngleDegrees;
            inputs.shoulderVelocityDegreesPerSecond = rightPivotEncoder.getVelocity().getValueAsDouble() * 360;
        }
        else {
            System.out.println("Both pivot encoders not measuring. Defaulting to shoulder angle of 0");
            shoulderAngleDegrees = 0;
            inputs.shoulderAngleDegrees = 0;
            inputs.shoulderVelocityDegreesPerSecond = 0;
        }


        
    }

    @Override
    public void setShoulderTargetAngle(double degrees) {
        this.targetShoulderAngleDegrees = degrees;
        leftShoulder.setControl(
            new MotionMagicTorqueCurrentFOC(degrees).withFeedForward(calculateShoulderFeedForward()));
        rightShoulder.setControl(new Follower(ArmConstants.leftShoulderMotorID, true));
    }

    @Override
    public void setExtensionTargetLength(double meters) {
        if (meters < ArmConstants.minExtensionMeters || meters > ArmConstants.maxExtensionMeters) {
            System.out.println("Invalid extension length requested!");
            return;
        }

        this.targetExtensionMeters = meters;
        Logger.recordOutput("arm/extensionFeedforward", calculateExtensionFeedForwardVolts());

        frontExtensionMotor.setControl(
            new MotionMagicVoltage(meters).withFeedForward(calculateExtensionFeedForwardVolts()).withEnableFOC(true));
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

    /** Calculates an additional feedforward output to add ON TOP of the feedforwad calculated by MotionMagic. 
     * In this case, it counteracts the force of gravity on the extension.
    */
    private double calculateExtensionFeedForwardVolts(){
        double feedforwardVolts = 0;
        feedforwardVolts += ArmConstants.kGExtensionVolts * Math.sin(Math.toRadians(shoulderAngleDegrees));

        //35.59 is the newtons of 8 pounds
        double constantForceSpringTorqueNm = 35.59*ArmConstants.extensionPulleyRadiusMeters/ArmConstants.extensionGearReduction/2; //force exerted by the constant force spring attached to the wire snake
        feedforwardVolts += constantForceSpringTorqueNm * Kraken.windingResistance / Kraken.torquePerAmp;
        return feedforwardVolts;
    }

    private double calculateShoulderFeedForward(){
        double feedforwardAmps = 0;
        // feedforwardAmps += ArmConstants.kSArmVolts * Math.signum(targetShoulderAngleDegrees - shoulderAngleDegrees);
        // feedforwardAmps += ArmConstants.kGArmVolts * Math.cos(Math.toRadians(shoulderAngleDegrees)) * extensionMeters; //TODO could be more precise, may be unecessary
        return feedforwardAmps;
    }
}
