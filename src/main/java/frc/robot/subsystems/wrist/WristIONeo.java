package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.WristConstants;
import frc.robot.VendorWrappers.Neo;

public class WristIONeo implements WristIO{
    
    private Neo wristNeo;

    private SparkMaxConfig config;

    private IdleMode idleMode = IdleMode.kBrake;

    /**
     * Creates a WristIONeo object.
     */
    public WristIONeo() {

        wristNeo = new Neo(1);

        configMotors();
    }

    private void configMotors() {
        // configures both Neos
        config = new SparkMaxConfig();
        
        // config.alternateEncoder.positionConversionFactor(180) //rotations to degrees
        //     .velocityConversionFactor(360/60) //rpm to deg/s
        //     .inverted(true);

        config.encoder.positionConversionFactor(360/WristConstants.alternateChainGearReduction)
            .velocityConversionFactor(360/60/WristConstants.alternateChainGearReduction);

        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);


        config.softLimit.forwardSoftLimitEnabled(false);

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // wristNeo.setPosition(this.getAbsoluteAngleDegrees(wristNeo.getAnalogInputVolts()));
        wristNeo.setPosition(WristConstants.maxAngleDegrees);
    }

    private double getAbsoluteAngleDegrees(double analogInputVolts) {
        boolean usingWristWithOrange3dPrint = true; // CHANGE WHEN NEW INTAKE
        double output = 0;
        if (usingWristWithOrange3dPrint) {
            double magnetDegreesWhenWristAtZero = 78.446;
            output = analogInputVolts * this.getAnalogInputDegreesPerVolt() - magnetDegreesWhenWristAtZero;
        }
        else {
            double magnetDegreesWhenWristAtZero = 152.174;
            output = analogInputVolts * this.getAnalogInputDegreesPerVolt() - magnetDegreesWhenWristAtZero;
        }

        // account for sensor wrap around / disconitinuity
        while (output > 180) {
            output -= 360;
        }
        while (output < -180) {
            output += 360;
        }

        return output;
    }

    private double getAnalogInputDegreesPerVolt() {
        // 3.3 volt sensor over full range of 1 rotation,
        // negated to match convention for positive wrist angles.
        return -1 * (360.0 / 3.3);
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristDegreesPerSecond = wristNeo.getVelocity();
        inputs.wristAngleDegrees = wristNeo.getPosition();

        inputs.motorOutputVoltage = wristNeo.getAppliedOutput()*wristNeo.getBusVoltage();
        inputs.motorOutputCurrentAmps = wristNeo.getOutputCurrent();
        inputs.analogInputVolts = wristNeo.getAnalogInputVolts();
        inputs.analogInputVoltsPerSecond = wristNeo.getAnalogInputVoltsPerSecond();

        inputs.absoluteAngleDegrees = this.getAbsoluteAngleDegrees(inputs.analogInputVolts);
        inputs.absoluteDegreesPerSecond = this.getAnalogInputDegreesPerVolt() * inputs.analogInputVoltsPerSecond;
        wristNeo.getBusVoltage();

        Logger.recordOutput("wrist/faultFlags", wristNeo.getFaults().rawBits);
        Logger.recordOutput("wrist/hasActiveFault", wristNeo.hasActiveFault());
        Logger.recordOutput("wrist/warningFlags", wristNeo.getWarnings().rawBits);
        Logger.recordOutput("wrist/hasActiveWarning", wristNeo.hasActiveWarning());
        Logger.recordOutput("wrist/overcurrent", wristNeo.getWarnings().overcurrent);
        Logger.recordOutput("wrist/busVoltage", wristNeo.getBusVoltage());
        Logger.recordOutput("wrist/dutyCycle", wristNeo.getAppliedOutput());
        Logger.recordOutput("wrist/supposedAppliedVolts", wristNeo.getAppliedOutput() * wristNeo.getBusVoltage());
    }

    @Override
    public void setWristPosition(double degrees) {
        wristNeo.setPosition(degrees);
    } 

    @Override
    public void setWristNeoVolts(double volts) {
        wristNeo.setVoltage(volts);
    }

    public void setDutyCycle(double dutyCycle) {
        wristNeo.set(dutyCycle);
    }

    public void toggleIdleMode() {
        if (idleMode == IdleMode.kBrake) {
            idleMode = IdleMode.kCoast;
        }
        else if (idleMode == IdleMode.kCoast) {
            idleMode = IdleMode.kBrake;
        }

        config.idleMode(idleMode);

        wristNeo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }


}
