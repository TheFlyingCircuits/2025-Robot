package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.VendorWrappers.Neo;

public class WristIONeo implements WristIO{
    
    public Neo wristNeo;

    public CANcoder wristAbsoluteEncoder;

    public WristIONeo() {

        wristAbsoluteEncoder = new CANcoder(WristConstants.wristCANcoderID, "CTRENetwork");

        // configures both Neo's
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        // config.smartCurrentLimit();
        // config.inverted();
        // config.absoluteEncoder.positionConversionFactor();
        // config.absoluteEncoder.velocityConversionFactor(); need to see if I need all of this

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CANcoderConfiguration wristCANCoderConfig = new CANcoderConfiguration();
        wristCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        wristCANCoderConfig.MagnetSensor.MagnetOffset = WristConstants.wristCANcoderOffset;
        wristCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        wristAbsoluteEncoder.getConfigurator().apply(wristCANCoderConfig);
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristRpm = wristNeo.getVelocity();

        inputs.wristAngleDegrees = wristAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    @Override
    public void setWristNeoVolts(double volts) {
        if (wristNeo.getMotorTemperature() > 70) {
            // need to change the max temp because I don't know what it should be
            //TODO: add error message for overheating
            //TODO: make motor temperature universal constraint
            volts = 0;
        }

        wristNeo.setVoltage(volts);
    }

    @Override
    public void setWristEncoderPosition(double degrees) {
        wristAbsoluteEncoder.setPosition(degrees/360.);
    }


}
