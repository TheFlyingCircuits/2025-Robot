package frc.robot.subsystems.wrist;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.VendorWrappers.Neo;

public class WristIONeo implements WristIO{
    
    private Neo wristNeo;

    private SparkAbsoluteEncoder leftEncoder;
    private SparkAbsoluteEncoder rightEncoder;

    /**
     * Creates a WristIONeo object.
     * @param rightEncoder - The right side throughbore encoder object. This has to be obtained from the placerGrabber object.
     */
    public WristIONeo(SparkAbsoluteEncoder rightEncoder) {

        this.rightEncoder=rightEncoder;

        // configures both Neo's
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .inverted(true);
        
        leftEncoder = wristNeo.getAbsoluteEncoder();
        config.absoluteEncoder.positionConversionFactor(2 * Math.PI)
            .zeroCentered(true)
            .inverted(false);

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        if (wristNeo.getLastError() == REVLibError.kOk) {
            inputs.wristRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity());
            inputs.wristAngleRadians = Units.rotationsToRadians(leftEncoder.getPosition());
        } else {
            inputs.wristRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity());
            inputs.wristAngleRadians = Units.rotationsToRadians(rightEncoder.getPosition());;
        }

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


}
