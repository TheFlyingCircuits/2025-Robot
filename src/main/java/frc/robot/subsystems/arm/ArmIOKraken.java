// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

    Kraken leftShoulder = new Kraken("leftShoulder", ArmConstants.leftShoulderMotorID, UniversalConstants.canivoreName);
    Kraken rightShoulder = new Kraken("rightShoulder", ArmConstants.rightShoulderMotorID, UniversalConstants.canivoreName);

    Kraken frontExtensionMotor = new Kraken("frontExtension", ArmConstants.frontExtensionMotorID, UniversalConstants.canivoreName);
    Kraken backExtensionMotor = new Kraken("backExtension", ArmConstants.backExtensionMotorID, UniversalConstants.canivoreName);

    TalonFXConfiguration extensionConfig;
    TalonFXConfiguration shoulderConfig;

    CANcoder leftPivotEncoder = new CANcoder(ArmConstants.leftPivotEncoderID, UniversalConstants.canivoreName);
    CANcoder rightPivotEncoder = new CANcoder(ArmConstants.rightPivotEncoderID, UniversalConstants.canivoreName);
    
    ArmFeedforward shoulderFeedforward = new ArmFeedforward(0, 0, 0);

    double targetExtensionMeters = ArmConstants.minExtensionMeters;
    double extensionMeters = ArmConstants.minExtensionMeters;
    double targetShoulderAngleDegrees = 0;
    double shoulderAngleDegrees = 0;

    NeutralModeValue neutralMode = NeutralModeValue.Brake;
    

    public ArmIOKraken() {
        configMotors();
    }

    private void configMotors() {

        /* CANCODER CONFIG */
        CANcoderConfiguration leftPivotConfig = new CANcoderConfiguration();
        leftPivotConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        leftPivotConfig.MagnetSensor.MagnetOffset = 0.26806640625;
        leftPivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        leftPivotEncoder.getConfigurator().apply(leftPivotConfig);

        CANcoderConfiguration rightPivotConfig = new CANcoderConfiguration();
        leftPivotConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        rightPivotConfig.MagnetSensor.MagnetOffset = 0.17919921875;
        rightPivotConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        rightPivotEncoder.getConfigurator().apply(rightPivotConfig);


        /* EXTENSION CONFIG */
        extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        extensionConfig.Feedback.SensorToMechanismRatio = 1./ArmConstants.extensionMetersPerMotorRotation;

        extensionConfig.CurrentLimits.StatorCurrentLimit = 100;

        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.maxExtensionMeters;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.minExtensionMeters-0.05;

        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 4; //mps of the extension
        extensionConfig.MotionMagic.MotionMagicAcceleration = 6; //m/s^2 of the extension

        extensionConfig.Slot0.kS = ArmConstants.kSExtensionVolts;
        extensionConfig.Slot0.kV = ArmConstants.kVExtensionVoltsSecondsPerRadian;
        extensionConfig.Slot0.kA = ArmConstants.kAExtensionVoltsSecondsSquaredPerRadian;
        extensionConfig.Slot0.kP = ArmConstants.kPExtensionVoltsPerMeter;
        extensionConfig.Slot0.kD = ArmConstants.kDExtensionVoltsPerMeterPerSecond;

        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        frontExtensionMotor.applyConfig(extensionConfig);

        extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        backExtensionMotor.applyConfig(extensionConfig);

        frontExtensionMotor.setPosition(ArmConstants.minExtensionMeters);
        backExtensionMotor.setPosition(ArmConstants.minExtensionMeters);


        /* SHOULDER CONFIG */
        shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.CurrentLimits.StatorCurrentLimit = 20;

        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shoulderConfig.Feedback.RotorToSensorRatio = ArmConstants.shoulderGearReduction;
        shoulderConfig.Feedback.SensorToMechanismRatio = 1;
        shoulderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        shoulderConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.leftPivotEncoderID;

        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 0.387; //rps of the arm
        shoulderConfig.MotionMagic.MotionMagicAcceleration = 1.4; //rotations per second squared of the arm
        shoulderConfig.MotionMagic.MotionMagicExpo_kV = 20; //unused
        shoulderConfig.MotionMagic.MotionMagicExpo_kA = 6; //unused

        shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.armMaxAngleDegrees / 360;
        shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (ArmConstants.armMinAngleDegrees - 0.5) / 360;

        //torquecurrentfoc constants
        // shoulderConfig.Slot0.kS = 1.5;
        // shoulderConfig.Slot0.kV = 0;
        // shoulderConfig.Slot0.kA = 20;
        // shoulderConfig.Slot0.kG = 3.75;
        // shoulderConfig.Slot0.kP = 0;
        
        shoulderConfig.Slot0.kS = 0.117;
        shoulderConfig.Slot0.kV = 31;
        shoulderConfig.Slot0.kA = 0;
        shoulderConfig.Slot0.kG = 0.22;
        shoulderConfig.Slot0.kP = 90;

        shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        shoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftShoulder.applyConfig(shoulderConfig);

        shoulderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightShoulder.applyConfig(shoulderConfig);

        leftShoulder.setPosition(leftPivotEncoder.getAbsolutePosition().getValueAsDouble());
        rightShoulder.setPosition(leftPivotEncoder.getAbsolutePosition().getValueAsDouble());
    }


    public void setExtensionEncoderPositionToMin() {
        frontExtensionMotor.setPosition(ArmConstants.minExtensionMeters);
        backExtensionMotor.setPosition(ArmConstants.minExtensionMeters);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        // homeExtensionIfRetracted();

        extensionMeters = frontExtensionMotor.getPosition().getValueAsDouble();
        inputs.extensionLengthMeters = extensionMeters;
        inputs.extensionLengthMetersPerSecond = frontExtensionMotor.getVelocity().getValueAsDouble();
        inputs.extensionMetersPerSecondSquared = frontExtensionMotor.getAcceleration().getValueAsDouble();

        inputs.targetExtensionLengthMeters = targetExtensionMeters;

        inputs.extensionAppliedVolts = frontExtensionMotor.getMotorVoltage().getValueAsDouble();
        inputs.extensionStatorCurrent = (frontExtensionMotor.getStatorCurrent().getValueAsDouble()
                                    + backExtensionMotor.getStatorCurrent().getValueAsDouble())/2;


        shoulderAngleDegrees = leftShoulder.getPosition().getValueAsDouble() * 360;
        inputs.shoulderAngleDegrees = shoulderAngleDegrees;
        inputs.shoulderVelocityDegreesPerSecond = leftShoulder.getVelocity().getValueAsDouble() * 360;

        inputs.targetShoulderAngleDegrees = targetShoulderAngleDegrees;
        
        // if (leftPivotEncoder.getAbsolutePosition().getStatus() == StatusCode.OK) {
        //     shoulderAngleDegrees = leftPivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        //     inputs.shoulderAngleDegrees = shoulderAngleDegrees;
        //     inputs.shoulderVelocityDegreesPerSecond = leftPivotEncoder.getVelocity().getValueAsDouble() * 360;
        // }
        // else if (rightPivotEncoder.getAbsolutePosition().getStatus() == StatusCode.OK) {
        //     System.out.println("Left pivot encoder not measuring! Using right instead");
        //     shoulderAngleDegrees = rightPivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        //     inputs.shoulderAngleDegrees = shoulderAngleDegrees;
        //     inputs.shoulderVelocityDegreesPerSecond = rightPivotEncoder.getVelocity().getValueAsDouble() * 360;
        // }
        // else {
        //     System.out.println("Both pivot encoders not measuring. Defaulting to shoulder angle of 0");
        //     shoulderAngleDegrees = 0;
        //     inputs.shoulderAngleDegrees = 0;
        //     inputs.shoulderVelocityDegreesPerSecond = 0;
        // }

        inputs.shoulderAppliedVolts = (leftShoulder.getMotorVoltage().getValueAsDouble()
                                        + rightShoulder.getMotorVoltage().getValueAsDouble())/2.;
        inputs.shoulderTorqueCurrent = (leftShoulder.getTorqueCurrent().getValueAsDouble()
                                         + rightShoulder.getTorqueCurrent().getValueAsDouble())/2.;


        Logger.recordOutput("arm/pivotReferencePosition", Units.rotationsToDegrees(leftShoulder.getClosedLoopReference().getValueAsDouble()));
        Logger.recordOutput("arm/pivotReferenceSlope", Units.rotationsToDegrees(leftShoulder.getClosedLoopReferenceSlope().getValueAsDouble()));
        Logger.recordOutput("arm/extensionReferencePosition", backExtensionMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("arm/extensionReferenceSlope", backExtensionMotor.getClosedLoopReferenceSlope().getValueAsDouble());
        Logger.recordOutput("arm/targetShoulderAngleDegrees", targetShoulderAngleDegrees);
        Logger.recordOutput("arm/targetExtensionLengthMeters", targetExtensionMeters);
        Logger.recordOutput("arm/statorCurrentLimitReached", leftShoulder.getFault_StatorCurrLimit().getValue());
    }


    @Override
    public void setShoulderTargetAngle(double degrees) {

        if (degrees < ArmConstants.armMinAngleDegrees || degrees > ArmConstants.armMaxAngleDegrees) {
            System.out.println("Invalid shoulder angle requested!");
            return;
        }

        this.targetShoulderAngleDegrees = degrees;

        // leftShoulder.setControl(new MotionMagicExpoVoltage(degrees/360.));
        leftShoulder.setControl(new MotionMagicVoltage(degrees/360.));
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

        boolean targetCloseToMin = Math.abs(meters - ArmConstants.minExtensionMeters) < 0.10;
        backExtensionMotor.setControl(
            new DynamicMotionMagicVoltage(
                meters,
                ArmConstants.extensionMaxMetersPerSecond,
                targetCloseToMin ? 6 : ArmConstants.extensionMaxMetersPerSecondSquared, //if the target might hit the hard stop, make it go slightly slower
                0
            ).withFeedForward(calculateExtensionFeedForwardVolts()).withEnableFOC(true));
        frontExtensionMotor.setControl(new Follower(ArmConstants.backExtensionMotorID, true));
    }


    @Override
    public void setShoulderMotorVolts(double volts) {
        leftShoulder.setVoltage(volts);
        rightShoulder.setVoltage(volts);
    }


    @Override
    public void setExtensionMotorVolts(double volts) {
        frontExtensionMotor.setVoltage(volts);
        backExtensionMotor.setVoltage(volts);
    }

    public void toggleIdleMode() {

        if (neutralMode == NeutralModeValue.Brake) {
            neutralMode = NeutralModeValue.Coast;
        }
        else if (neutralMode == NeutralModeValue.Coast) {
            neutralMode = NeutralModeValue.Brake;
        }

        frontExtensionMotor.setNeutralMode(neutralMode);
        backExtensionMotor.setNeutralMode(neutralMode);
        leftShoulder.setNeutralMode(neutralMode);
        rightShoulder.setNeutralMode(neutralMode);
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
