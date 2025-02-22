package frc.robot.VendorWrappers;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class Neo extends SparkMax {

    private String name;

    private RelativeEncoder encoder;
    private double mostRecentGoodPosition = 0;
    private double mostRecentGoodVelocity = 0;
    private double mostRecentGoodAppliedOutput = 0;
    private double mostRecentGoodBusVoltage = 0;
    private double mostRecentGoodOutputCurrent = 0;


    // physics constants
    /** How many volts were applied to the motor by the manufacturer when determining its electrical characteristics. */
    public static final double nominalVoltage = 12;

    /** How quickly the motor turns (radians per second) when 12 volts are applied and there is no load on the motor. */
    public static final double freeSpeed = Units.rotationsPerMinuteToRadiansPerSecond(5820);

    /** How much current (amps) flowed through the motor while spinning at the free speed */
    public static final double freeCurrent = 1.7;

    /** How much torque (newton-meters) the stator exerts on the rotor when the rotor is held still,
     *  and then 12 volts are applied to the motor. */
    public static final double stallTorque = 3.28;

    /** How much current (amps) flows through the motor coils when the rotor is held still, and then 12 volts are applied to the motor. */
    public static final double stallCurrent = 181;

    /** How much torque (newton-meters) the stator will exert on the rotor per amp of current that flows through the motor coils.  */
    public static final double torquePerAmp = stallTorque / stallCurrent;

    /** How much resistance (ohms) the motor windings have. */
    public static final double windingResistance = nominalVoltage / stallCurrent;

    /** How many volts are induced in the motor windings by the permanent magnets on the rotor
     *  for each radian-per-second that the rotor is spinning at. */
    public static final double kEMF = (nominalVoltage - (freeCurrent * windingResistance)) / freeSpeed;

    public Neo(int canID) {
        this("Neo #"+canID, canID);
    }

    public Neo(String name, int canID) {
        super(canID, MotorType.kBrushless);
        this.name = name;
        encoder = super.getEncoder();
    }

    @Override
    public REVLibError configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
        double secondsBetweenRetrys = 5;

        while (true) {
            REVLibError errorCode = super.configure(config, resetMode, persistMode);

            if (errorCode == REVLibError.kOk) {
                return errorCode;
            }

            System.out.println("Error enocuntered while configuring "+name);
            System.out.println("REVLibError: " + errorCode.toString());
            System.out.println("Retrying...");
            Timer.delay(secondsBetweenRetrys);
        }
    }

    public REVLibError waitForConfig(Supplier<REVLibError> configFunction, String errorMessage) {
        double waitTimeSeconds = 4.0/8.0;
        
        while (true) {
            REVLibError errorCode = configFunction.get();

            if (errorCode == REVLibError.kOk) {
                return errorCode;
            }

            String fullErrorMessage = errorMessage;
            fullErrorMessage += "\nNeo Error: " + errorCode;
            fullErrorMessage += "\nRetrying...";
            System.out.println(fullErrorMessage);
            Timer.delay(waitTimeSeconds);
        }
    }

    public double getPosition() {
        // this is how advantage kit odometry example does it.
        double newPosition = encoder.getPosition();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodPosition = newPosition;
            return newPosition;
        }
        else {
            System.out.println("Error getting the position of "+name+": "+errorCode);
            System.out.println("Returning most recent valid position");
            return mostRecentGoodPosition;
        }
    }

    public double getVelocity() {
        // this is how advantage kit odometry example does it
        double newVelocity = encoder.getVelocity();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodVelocity = newVelocity;
            return newVelocity;
        }
        else {
            System.out.println("Error getting the velocity of "+name+": "+errorCode);
            System.out.println("Returning most recent valid velocity");
            return mostRecentGoodVelocity;
        }
    }

    public double getAppliedOutput() {
        double newAppliedOutput = super.getAppliedOutput();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodAppliedOutput = newAppliedOutput;
            return newAppliedOutput;
        }
        else {
            System.out.println("Error getting the applied output of "+name+": "+errorCode);
            System.out.println("Returning most recent valid applied output");
            return mostRecentGoodAppliedOutput;
        }
    }

    public double getBusVoltage() {
        double newBusVoltage = super.getBusVoltage();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodBusVoltage = newBusVoltage;
            return newBusVoltage;
        }
        else {
            System.out.println("Error getting the bus voltage for "+name+": "+errorCode);
            System.out.println("Returning most recent valid bus voltage");
            return mostRecentGoodBusVoltage;
        }
    }

    public double getOutputCurrent() {
        double newOutputCurrent = super.getOutputCurrent();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodOutputCurrent = newOutputCurrent;
            return newOutputCurrent;
        }
        else {
            System.out.println("Error getting the output current of "+name+": "+errorCode);
            System.out.println("Returning most recent valid output current");
            return mostRecentGoodOutputCurrent;
        }
    }

    public REVLibError setPosition(double position) {
        String errorMessage = "Failed to set the position of "+name+" to "+position+"!";
        return this.waitForConfig(() -> {return encoder.setPosition(position);}, errorMessage);
    }

    /**
     * Finds the amount of volts needed to feed to the motor in order for the motor to exert a certain torque.
     */
    public double getVoltsForTorque(double newtonMeters) {
        /* The Neos have a Current (amps) control mode, but its only through
         * a PID interface which doesn't feel appropriate for compensating for backEMF.
         * I would need a feedforward that's a function of rotor velocity as well as amp setpoint,
         * but the controller only provides support for the latter.
         * Therefore, I just implement this in terms of voltage and compensate for the backEMF myself.
         * Todo: I should probably also compare the desired current to the measured current to see how well
         *       this stragety is working / maybe even add a small proportional controller.
         *
         * T = kT * I
         * T = kT * (V / R)
         * T = kT * ([vApplied + vInduced] / R)
         * T = kT * ([vApplied - radiansPerSecond * kEMF] / R)
         * vApplied = (T / kT) * R + (radiansPerSecond * kEMF)
         */
        double rpm = this.getVelocity();
        double radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rpm);

        return ((newtonMeters / torquePerAmp) * windingResistance) + (radiansPerSecond * kEMF);
        


        // double amps = newtonMeters / torquePerAmp;
        // super.getPIDController();
        // super.getOutputCurrent();
        // super.enableVoltageCompensation(nominalVoltage);
        // super.getPIDController().setReference(amps, ControlType.kCurrent);
    }

    public void exertTorque(double newtonMeters) {
        super.setVoltage(getVoltsForTorque(newtonMeters));
    }

    @Override
    public void setVoltage(double volts) {
        if (super.getMotorTemperature() > 70) {
            System.out.println("SPARK motor with ID " + super.getDeviceId() + " overheating! Temperature: " + super.getMotorTemperature());
            volts = 0;
        }
        super.setVoltage(volts);
    }

    
}
