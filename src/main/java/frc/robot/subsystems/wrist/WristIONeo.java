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

    private SparkAbsoluteEncoder encoder ;

    private SparkAbsoluteEncoder encoderBeingUsed;

    private SparkAbsoluteEncoder encoder2;

    public WristIONeo(SparkAbsoluteEncoder encoder2) {

        this.encoder2=encoder2;
        // configures both Neo's
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        // config.smartCurrentLimit();
        // config.inverted();
        // config.absoluteEncoder.positionConversionFactor();
        // config.absoluteEncoder.velocityConversionFactor(); need to see if I need all of this

        wristNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = wristNeo.getAbsoluteEncoder();
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.wristRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(encoderBeingUsed.getVelocity());

        if (wristNeo.getLastError() == REVLibError.kOk) {
            encoderBeingUsed = encoder;
        } else {
            encoderBeingUsed = encoder2;
        }

        inputs.wristAngleRadians = Units.rotationsToRadians(encoderBeingUsed.getPosition());
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
