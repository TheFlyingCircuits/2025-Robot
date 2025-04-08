package frc.robot.subsystems.placerGrabber;

import org.littletonrobotics.junction.Logger;

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
        SparkMaxConfig sideNeoConfig = new SparkMaxConfig();
        sideNeoConfig.idleMode(IdleMode.kBrake);
        sideNeoConfig.smartCurrentLimit(40);

        sideNeoConfig.inverted(false);
        sideNeoConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        sideNeoConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        sideNeo.configure(sideNeoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        SparkMaxConfig frontNeoConfig = new SparkMaxConfig();

        
        frontNeoConfig.idleMode(IdleMode.kBrake);
        
        frontNeoConfig.smartCurrentLimit(15);

        //config for the left alternate encoder, eventually used by the wristIO
        frontNeoConfig.alternateEncoder.positionConversionFactor(360) //rotations to degrees
            .velocityConversionFactor(360/60) //rpm to deg/s
            .inverted(false);


        frontNeoConfig.inverted(true);
        frontNeo.configure(frontNeoConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PlacerGrabberIOInputs inputs) {
        inputs.frontRollerRPM = frontNeo.getVelocity();
        inputs.sideRollerRPM = sideNeo.getVelocity();

        inputs.frontMotorAppliedCurrent = frontNeo.getOutputCurrent();
        inputs.sideMotorAppliedCurrent = sideNeo.getOutputCurrent();
        
        inputs.leftSensorSeesCoral = sideNeo.getReverseLimitSwitch().isPressed();
        inputs.rightSensorSeesCoral = sideNeo.getForwardLimitSwitch().isPressed();

        Logger.recordOutput("placerGrabber/orangeWheelsNeo/faultFlags", frontNeo.getFaults().rawBits);
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/hasActiveFault", frontNeo.hasActiveFault());
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/warningFlags", frontNeo.getWarnings().rawBits);
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/hasActiveWarning", frontNeo.hasActiveWarning());
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/overcurrent", frontNeo.getWarnings().overcurrent);
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/busVoltage", frontNeo.getBusVoltage());
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/dutyCycle", frontNeo.getAppliedOutput());
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/supposedAppliedVolts", frontNeo.getAppliedOutput() * frontNeo.getBusVoltage());

        Logger.recordOutput("placerGrabber/omniwheelsNeo/faultFlags", sideNeo.getFaults().rawBits);
        Logger.recordOutput("placerGrabber/omniwheelsNeo/hasActiveFault", sideNeo.hasActiveFault());
        Logger.recordOutput("placerGrabber/omniwheelsNeo/warningFlags", sideNeo.getWarnings().rawBits);
        Logger.recordOutput("placerGrabber/omniwheelsNeo/hasActiveWarning", sideNeo.hasActiveWarning());
        Logger.recordOutput("placerGrabber/omniwheelsNeo/overcurrent", sideNeo.getWarnings().overcurrent);
        Logger.recordOutput("placerGrabber/omniwheelsNeo/busVoltage", sideNeo.getBusVoltage());
        Logger.recordOutput("placerGrabber/omniwheelsNeo/dutyCycle", sideNeo.getAppliedOutput());
        Logger.recordOutput("placerGrabber/omniwheelsNeo/supposedAppliedVolts", sideNeo.getAppliedOutput() * sideNeo.getBusVoltage());

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