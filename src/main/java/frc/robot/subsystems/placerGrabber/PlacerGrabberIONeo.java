package frc.robot.subsystems.placerGrabber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.VendorWrappers.Neo;

public class PlacerGrabberIONeo implements PlacerGrabberIO {
    
    public Neo frontNeo;
    public Neo sideNeo;

    public PlacerGrabberIONeo() {
        // configures both Neo's
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        // config.smartCurrentLimit();
        // config.inverted();
        // config.absoluteEncoder.positionConversionFactor();
        // config.absoluteEncoder.velocityConversionFactor(); need to see if I need all of this
        frontNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sideNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(PlacerGrabberIOInputs inputs) {
        inputs.frontRollerRPM = frontNeo.getVelocity();
        inputs.sideRollerRPM = sideNeo.getVelocity();
    }

    @Override
    public void setFrontNeoVolts(double volts) {
        if (frontNeo.getMotorTemperature() > 70) {
            // need to change the max temp because I don't know what it should be
            //TODO: add error message for overheating
            //TODO: make motor temperature universal constraint
            volts = 0;
        }

        frontNeo.setVoltage(volts);
    }

    @Override
    public void setSideNeoVolts(double volts) {
        if (sideNeo.getMotorTemperature() > 70) {
            volts = 0;
        }

        sideNeo.setVoltage(volts);
    }

}