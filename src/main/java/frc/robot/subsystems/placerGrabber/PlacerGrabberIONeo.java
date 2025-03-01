package frc.robot.subsystems.placerGrabber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.VendorWrappers.Neo;

public class PlacerGrabberIONeo implements PlacerGrabberIO {
    
    public Neo frontNeo;
    public Neo sideNeo;

    /**
     * Creates a PlacerGrabberIONeo object. <p>
     * PlacerGrabber wiring setup: <p>
     * wrist SparkMAX - right throughbore encoder <p>
     * front roller SparkMAX - left throughbore encoder <p>
     * side roller SparkMAX - coral sensors
     */
    public PlacerGrabberIONeo() {

        frontNeo = new Neo(2);
        sideNeo = new Neo(3);

        // configures both Neos
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(30);

        config.inverted(false);
        sideNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //config for the left alternate encoder, eventually used by the wristIO
        config.alternateEncoder.positionConversionFactor(360) //rotations to degrees
            .velocityConversionFactor(360/60) //rpm to deg/s
            .inverted(false);

        config.inverted(true);
        frontNeo.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PlacerGrabberIOInputs inputs) {
        inputs.frontRollerRPM = frontNeo.getVelocity();
        inputs.sideRollerRPM = sideNeo.getVelocity();

        inputs.frontMotorAppliedCurrent = frontNeo.getOutputCurrent();
        inputs.sideMotorAppliedCurrent = sideNeo.getOutputCurrent();
        
        inputs.leftSensorSeesCoral = sideNeo.getReverseLimitSwitch().isPressed();
        inputs.rightSensorSeesCoral = sideNeo.getForwardLimitSwitch().isPressed();

    }

    @Override
    public void setFrontNeoVolts(double volts) {
        frontNeo.setVoltage(volts);
    }

    @Override
    public void setSideNeoVolts(double volts) {
        sideNeo.setVoltage(volts);
    }

    /** Returns the left side throughbore encoder that is used by the wrist subsystem. */
    public RelativeEncoder getLeftThroughboreEncoder() {
        return frontNeo.getAlternateEncoder();
    }

}