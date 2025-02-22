package frc.robot.subsystems.placerGrabber;

import com.revrobotics.AbsoluteEncoder;
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
        // configures both Neo's
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(60);

        config.inverted(false);
        sideNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.absoluteEncoder.positionConversionFactor(2 * Math.PI)
            .zeroCentered(true) 
            .inverted(false);
            
        config.inverted(true);
        frontNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PlacerGrabberIOInputs inputs) {
        inputs.frontRollerRPM = frontNeo.getVelocity();
        inputs.sideRollerRPM = sideNeo.getVelocity();

        inputs.leftSensorSeesCoral = sideNeo.getForwardLimitSwitch().isPressed(); //TODO: find which forward and reverse is
        inputs.rightSensorSeesCoral = sideNeo.getReverseLimitSwitch().isPressed();

    }

    @Override
    public void setFrontNeoVolts(double volts) {

        frontNeo.setVoltage(volts);
    }

    @Override
    public void setSideNeoVolts(double volts) {
        if (sideNeo.getMotorTemperature() > 70) {
            volts = 0;
        }

        sideNeo.setVoltage(volts);
    }

    /** Returns the left side throughbore encoder that */
    public SparkAbsoluteEncoder getLeftThroughboreEncoder() {
        return frontNeo.getAbsoluteEncoder();
    }

}