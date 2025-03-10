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

        config.encoder.positionConversionFactor(360/WristConstants.gearReduction)
            .velocityConversionFactor(360/60/WristConstants.gearReduction);

        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(true);

        config.softLimit.forwardSoftLimitEnabled(true)
            .forwardSoftLimit(WristConstants.maxAngleDegrees+2)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(WristConstants.minAngleDegrees-2);

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        wristNeo.setPosition(this.getAbsoluteAngleDegrees(wristNeo.getAnalogInputVolts()));
    }

    private double getAbsoluteAngleDegrees(double analogInputVolts) {
        double magnetDegreesWhenWristAtZero = -204.1;
        return analogInputVolts * this.getAnalogInputDegreesPerVolt() - magnetDegreesWhenWristAtZero;
    }

    private double getAnalogInputDegreesPerVolt() {
        // 5 volt sensor over full range of 1 rotation, negated to match convention for positive wrist angles.
        return -1 * (360.0 / 5.0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristDegreesPerSecond = wristNeo.getVelocity();
        inputs.wristAngleDegrees = wristNeo.getPosition();

        inputs.motorOutputVoltage = wristNeo.getAppliedOutput()*12;
        inputs.motorOutputCurrentAmps = wristNeo.getOutputCurrent();
        inputs.analogInputVolts = wristNeo.getAnalogInputVolts();
        inputs.analogInputVoltsPerSecond = wristNeo.getAnalogInputVoltsPerSecond();

        inputs.absoluteAngleDegrees = this.getAbsoluteAngleDegrees(inputs.analogInputVolts);
        inputs.absoluteDegreesPerSecond = this.getAnalogInputDegreesPerVolt() * inputs.analogInputVoltsPerSecond;
        wristNeo.getBusVoltage();


    }

    @Override
    public void setWristPosition(double degrees) {
        wristNeo.setPosition(degrees);
    } 

    @Override
    public void setWristNeoVolts(double volts) {
        wristNeo.setVoltage(volts);
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
