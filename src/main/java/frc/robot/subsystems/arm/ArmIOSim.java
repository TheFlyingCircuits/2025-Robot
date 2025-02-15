package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UniversalConstants;
import edu.wpi.first.math.system.NumericalIntegration;

public class ArmIOSim implements ArmIO {

    private TalonFX shoulderTalon = new TalonFX(0, "*");
    private TalonFX extensionTalon = new TalonFX(1, "*");
    
    private DCMotor shoulderMotor = DCMotor.getKrakenX60(2).withReduction(ArmConstants.shoulderGearReduction);
    private DCMotor extensionMotor = DCMotor.getKrakenX60(1).withReduction(ArmConstants.extensionGearReduction);

    private MotionMagicTorqueCurrentFOC shoulderRequest;
    private MotionMagicVoltage extensionRequest;

    Matrix<N2, N1> systemInputs = VecBuilder.fill(0, 0);

    //configs
    //shoulder
    double shoulderkP = 2;
    double shoulderkV = 1;
    double shoulderkA = 1;

    //extenstion
    double extensionkP = 2;
    double extensionkV = 1;
    double extensionkA = 1;
     
    public ArmIOSim() {

        

        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

        shoulderConfig.MotionMagic.MotionMagicAcceleration = 10;
        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
        shoulderConfig.Slot0.kP = 2;
        shoulderConfig.Slot0.kV = 1;
        shoulderConfig.Slot0.kA = 1;

        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoulderConfig.CurrentLimits.StatorCurrentLimit = 45; // re-determined after firmware upgrade to prevent wheel slip. Feels pretty low though
        shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shoulderTalon.getConfigurator().apply(shoulderConfig);

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        extensionConfig.MotionMagic.MotionMagicAcceleration = 10;
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
        extensionConfig.Slot0.kP = 2;
        extensionConfig.Slot0.kV = 1;
        extensionConfig.Slot0.kA = 1;

        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extensionConfig.CurrentLimits.StatorCurrentLimit = 45; // re-determined after firmware upgrade to prevent wheel slip. Feels pretty low though
        extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        extensionTalon.getConfigurator().apply(extensionConfig);
    };

    /*
     *    STATE VECTOR
     * shoulder angle (deg))   <needs to become radians>
     * shoulder angvel (deg/s)
     * arm length (m)
     * arm extension vel. (m/s)
     * 
     * 
     *    INPUT VECTOR
     * shoulder motor CURRENT (amps)
     * extension motor voltage (volts)
     */

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        this.systemInputs.set(1, 0, extensionTalon.getMotorVoltage().getValueAsDouble());
        this.systemInputs.set(0, 0, shoulderTalon.getTorqueCurrent().getValueAsDouble());
        //this.systemInputs.set(1, 0,)


        Matrix<N4, N1> state = VecBuilder.fill(
            inputs.shoulderAngleDegrees,
            inputs.shoulderVelocityDegreesPerSecond,
            inputs.extensionLengthMeters,
            inputs.extensionLengthMetersPerSecond
        );
        
        Matrix<N4, N1> nextState = NumericalIntegration.rk4(
            this::calculateSystemDerivative,
            state,
            this.systemInputs, 
            UniversalConstants.defaultPeriodSeconds
        );

        inputs.shoulderAngleDegrees = nextState.get(0, 0);
        shoulderTalon.setPosition(inputs.shoulderAngleDegrees);
        inputs.shoulderVelocityDegreesPerSecond = nextState.get(1, 0);
        inputs.shoulderAppliedCurrent = this.systemInputs.get(0, 0);

        inputs.extensionLengthMeters = nextState.get(2, 0);
        extensionTalon.setPosition(inputs.extensionLengthMeters);

        inputs.extensionLengthMetersPerSecond = nextState.get(3, 0);
        inputs.extensionAppliedVolts = this.systemInputs.get(1, 0);
    }




    /**
     * Returns the first derivative of state matrix
     */
    private Matrix<N4, N1> calculateSystemDerivative(Matrix<N4, N1> state, Matrix<N2, N1> input) {
        double shoulderDegrees = state.get(0, 0);
        double shoulderDegreesPerSecond = state.get(1, 0);
        double extensionMeters = state.get(2, 0);

        double shoulderMotorTorqueNM = shoulderMotor.getTorque(input.get(0, 0));
        double shoulderMotorRadiansPerSecondSquared = shoulderMotorTorqueNM / calculateShoulderMomentOfInertia(extensionMeters);

        Translation2d centerOfMassMeters = calculateCenterOfMassMeters(shoulderDegrees, extensionMeters);
        double shoulderGravityRadiansPerSecondSquared = -UniversalConstants.gravityMetersPerSecondSquared
                                                        /centerOfMassMeters.getNorm()
                                                        *Math.cos(centerOfMassMeters.getAngle().getRadians());

        double shoulderRadiansPerSecondSquared = shoulderMotorRadiansPerSecondSquared + shoulderGravityRadiansPerSecondSquared;
        shoulderRadiansPerSecondSquared -= Math.signum(shoulderDegreesPerSecond)*5; //friction

        

        double extensionMetersPerSecond = state.get(3, 0);
        double extensionVoltage = input.get(1, 0);
        double extensionCurrentAmps = extensionMotor.getCurrent(extensionMetersPerSecond, extensionVoltage);
        double extensionTorqueNM = extensionMotor.KtNMPerAmp * extensionCurrentAmps;


        //double extensionMetersPerSecondSquared = extensionKv * extensionVoltage;
        double extensionMetersPerSecondSquared = extensionTorqueNM / ArmConstants.pulleyRadiusMeters;
        

        //return null; 
        return VecBuilder.fill(
            shoulderDegreesPerSecond,
            Math.toDegrees(shoulderRadiansPerSecondSquared),
            extensionMetersPerSecond,
            extensionMetersPerSecondSquared
        );
    }


    private double calculateShoulderMomentOfInertia(double armLengthMeters) {
        return ArmConstants.armMassKg / 3. * armLengthMeters * armLengthMeters; //somewhat temporary, assumes that arm is a constant density beam thing.
    }

    /**
     * Calculates the center of mass of the arm, in the pivot reference frame.
     */
    private Translation2d calculateCenterOfMassMeters(double shoulderAngleDegrees, double armLengthMeters) {
        Translation2d armFrameMeters = new Translation2d(armLengthMeters/2, -ArmConstants.shoulderBracketLengthMeters);
        return armFrameMeters.rotateBy(Rotation2d.fromDegrees(shoulderAngleDegrees));
    }

    @Override
    public void setShoulderTargetAngle(double degrees) {
        shoulderRequest = new MotionMagicTorqueCurrentFOC(degrees); //TODO units issue? param is rotation we give degrees
        shoulderTalon.setControl(shoulderRequest);
    }

    @Override
    public void setExtensionTargetLength(double meters) {
        extensionRequest = new MotionMagicVoltage(meters); //^Here too 
        extensionTalon.setControl(extensionRequest);
    }


    @Override
    public void setShoulderMotorAmps(double amps) {
        systemInputs.set(0, 0, amps);
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        systemInputs.set(1, 0, volts);
    }

}
