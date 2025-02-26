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
        
        config.alternateEncoder.positionConversionFactor(180) //rotations to degrees
            .velocityConversionFactor(360/60) //rpm to deg/s
            .inverted(true);

        config.encoder.positionConversionFactor(360/WristConstants.gearReduction)
            .velocityConversionFactor(360/60/WristConstants.gearReduction);

        config.idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)
            .inverted(true);

        config.softLimit.forwardSoftLimitEnabled(true)
            .forwardSoftLimit(WristConstants.maxAngleDegrees)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(WristConstants.minAngleDegrees);

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightEncoder = wristNeo.getAlternateEncoder();

        wristNeo.setPosition(WristConstants.minAngleDegrees);
        rightEncoder.setPosition(WristConstants.maxAngleDegrees);
        leftEncoder.setPosition(WristConstants.maxAngleDegrees);
        
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristDegreesPerSecond = wristNeo.getVelocity();
        inputs.wristAngleDegrees = wristNeo.getPosition();

        inputs.motorOutputCurrentAmps = wristNeo.getOutputCurrent();
    }

    @Override
    public void setWristNeoVolts(double volts) {
        wristNeo.setVoltage(volts);
    }


}
