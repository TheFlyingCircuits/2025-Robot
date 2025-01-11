package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.VendorWrappers.Neo;

public class SwerveModuleIONeo implements SwerveModuleIO{

    /**
     * This angle offset is added directly to the raw angle read by the absolute encoder.
     */
    private double angleOffset;
    private CANcoder absoluteEncoder;
    private Neo angleMotor;
    private Neo driveMotor;


    /**
     * Constructs the hardware implementation for each swerve module
     * @param driveMotorID - ID of the motor controller to the drive motor 
     * @param angleMotorID - ID of the motor controller to the angle motor
     * @param angleOffset - Offset for the individual CANcoders on each swerve module, in +-1
     * @param cancoderID - ID of the CANcoders mounted on each swerve module
     */
    public SwerveModuleIONeo(int driveMotorID, int angleMotorID, double angleOffset, int cancoderID){
        this.angleOffset = angleOffset;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID);
        configCANCoder();

        /** Angle motor config */
        angleMotor = new Neo(angleMotorID);
        configAngleMotor();

        /** Drive motor config */
        driveMotor = new Neo(driveMotorID);
        configDriveMotor();

    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getPosition();
        inputs.driveVelocityMetersPerSecond = driveMotor.getVelocity();
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }

    private void configCANCoder() {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    private void configAngleMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
        config.inverted(MotorConstants.angleInvert);
        config.absoluteEncoder.positionConversionFactor(SwerveModuleConstants.steerGearReduction*360.0);
        config.absoluteEncoder.velocityConversionFactor(SwerveModuleConstants.steerGearReduction*360.0/60.0);
        //converts rpm of motor into deg/s of wheel
        angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configDriveMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(MotorConstants.driveContinuousCurrentLimit);
        config.absoluteEncoder.positionConversionFactor(SwerveModuleConstants.driveGearReduction 
        * SwerveModuleConstants.wheelCircumferenceMeters);
        config.absoluteEncoder.velocityConversionFactor(1./60. * SwerveModuleConstants.driveGearReduction
        * SwerveModuleConstants.wheelCircumferenceMeters);
        config.inverted(MotorConstants.driveInvert);
        driveMotor.setPosition(0.0);
        // Converts rpm of motor to m/s of wheel.
        driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
