package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.system.NumericalIntegration;

public class ArmIOSim implements ArmIO {

    
    private DCMotor shoulderMotor = DCMotor.getKrakenX60(2).withReduction(ArmConstants.shoulderGearReduction);
    private DCMotor extensionMotor = DCMotor.getKrakenX60(1).withReduction(1);

    public ArmIOSim() {

    };

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO Auto-generated method stub
        //ArmIO.super.updateInputs(inputs);

        Matrix<N4, N1> state = VecBuilder.fill(
            inputs.shoulderAngleDegrees,
            inputs.shoulderVelocityDegreesPerSecond,
            inputs.extensionLengthMeters,
            inputs.extensionLengthMetersPerSecond
        );
        Matrix<N2, N1> systemInputs = VecBuilder.fill(
            
        );
        Matrix<N4, N1> nextState = NumericalIntegration.rk4(this::calculateSystemDerivative, state, systemInputs, 0.05); //period should be replaced with a constant maybe

        inputs.shoulderAngleDegrees = nextState.get(0, 0);
        inputs.shoulderVelocityDegreesPerSecond = nextState.get(1, 0);
        inputs.shoulder


        inputs.extensionLengthMeters = nextState.get(2, 0);
        inputs.extensionLengthMetersPerSecond = nextState.get(3, 0);
        inputs.extensionAppliedVolts = systemInputs.get(1, 0);
    }


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



    /**
     * Returns the first derivative of state matrix
     */
    private Matrix<N4, N1> calculateSystemDerivative(Matrix<N4, N1> state, Matrix<N2, N1> input) {


        double shoulderDegrees = state.get(0, 0);
        double shoulderDegreesPerSecond = state.get(1, 0);

        double extensionMeters = state.get(2, 0);

        //kt = torque/current
        double shoulderMotorTorqueNM = shoulderMotor.KtNMPerAmp * input.get(0, 0);
        double shoulderMotorRadiansPerSecondSquared = shoulderMotorTorqueNM / calculateShoulderMomentOfInertia(extensionMeters);
        double shoulderGravityRadiansPerSecondSquared = ArmConstants.gravityMetersPerSecondSquared
                                                        /calculateCenterOfMassMeters(extensionMeters)*Math.cos(Math.toRadians(shoulderDegrees));

        double shoulderRadiansPerSecondSquared = shoulderMotorRadiansPerSecondSquared + shoulderGravityRadiansPerSecondSquared;

        

        double extensionMetersPerSecond = state.get(3, 0);
        double extensionVoltage = input.get(1, 0);

        double extensionCurrentAmps = extensionMotor.getCurrent(extensionMetersPerSecond, extensionVoltage);
        double extensionTorqueNM = extensionMotor.KtNMPerAmp * extensionCurrentAmps;


        //double extensionMetersPerSecondSquared = extensionKv * extensionVoltage;
        double extensionMetersPerSecondSquared = extensionTorqueNM / ArmConstants.pulleyGearRadiusMeters;
        

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

    private double calculateCenterOfMassMeters(double armLengthMeters) {
        return armLengthMeters/2;
    }


    @Override
    public void setArmAccelerationDegreesPerSecondSquared(double degreesPerSecondSquared) {
        // TODO Auto-generated method stub
        ArmIO.super.setArmAccelerationDegreesPerSecondSquared(degreesPerSecondSquared);
    }

    @Override
    public void setArmMotorVolts(double volts) {
        // TODO Auto-generated method stub
        ArmIO.super.setArmMotorVolts(volts);
    }

}
