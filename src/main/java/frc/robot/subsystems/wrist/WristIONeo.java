package frc.robot.subsystems.wrist;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
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

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    /**
     * Creates a WristIONeo object.
     * @param leftEncoder - The left side throughbore encoder object. This has to be obtained from the placerGrabber object.
     */
    public WristIONeo(RelativeEncoder leftEncoder) {

        wristNeo = new Neo(1);

        this.leftEncoder=leftEncoder;

        configMotors();
    }

    private void configMotors() {
        // configures both Neos
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.alternateEncoder.positionConversionFactor(1) //rotations to degrees
            .velocityConversionFactor(360/60) //rpm to deg/s
            .inverted(true);

        config.idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60)
            .inverted(true);

        config.softLimit.forwardSoftLimitEnabled(true)
            .forwardSoftLimit(WristConstants.maxAngleDegrees)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(WristConstants.minAngleDegrees);

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightEncoder = wristNeo.getAlternateEncoder();

        rightEncoder.setPosition(WristConstants.maxAngleDegrees);
        leftEncoder.setPosition(WristConstants.maxAngleDegrees);
        
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristDegreesPerSecond = leftEncoder.getVelocity();
        inputs.wristAngleDegrees = leftEncoder.getPosition();
    }

    @Override
    public void setWristNeoVolts(double volts) {
        wristNeo.setVoltage(volts);
    }


}
