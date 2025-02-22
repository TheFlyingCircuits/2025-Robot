package frc.robot.subsystems.wrist;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristConstants;
import frc.robot.VendorWrappers.Neo;

public class WristIONeo implements WristIO{
    
    private Neo wristNeo;

    private SparkAbsoluteEncoder leftEncoder;
    private SparkAbsoluteEncoder rightEncoder;

    /**
     * Creates a WristIONeo object.
     * @param leftEncoder - The left side throughbore encoder object. This has to be obtained from the placerGrabber object.
     */
    public WristIONeo(SparkAbsoluteEncoder leftEncoder) {

        this.leftEncoder=leftEncoder;

        // configures both Neos
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(true);

        config.softLimit.forwardSoftLimitEnabled(true)
            .forwardSoftLimit(WristConstants.maxAngleRadians)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(WristConstants.minAngleDegrees);

        config.absoluteEncoder.positionConversionFactor(2 * Math.PI)
            .velocityConversionFactor(2 * Math.PI * 60)
            .zeroCentered(true)
            .inverted(false);

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightEncoder = wristNeo.getAbsoluteEncoder();
        
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        if (wristNeo.getLastError() == REVLibError.kOk) {
            inputs.wristRadiansPerSecond = leftEncoder.getVelocity();
            inputs.wristAngleRadians = leftEncoder.getPosition();
        } else {
            inputs.wristRadiansPerSecond = rightEncoder.getVelocity();
            inputs.wristAngleRadians = rightEncoder.getPosition();
        }

    }

    @Override
    public void setWristNeoVolts(double volts) {
        wristNeo.setVoltage(volts);
    }


}
