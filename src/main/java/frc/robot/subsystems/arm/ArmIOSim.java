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

public class ArmIOSim implements ArmIO {

    
    private DCMotor shoulderMotor = DCMotor.getKrakenX60(2).withReduction(ArmConstants.shoulderGearReduction);
    private DCMotor extensionMotor = DCMotor.getKrakenX60(1).withReduction(1);

    public ArmIOSim() {

    };

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO Auto-generated method stub
        ArmIO.super.updateInputs(inputs);
    }


    /*
     *    STATE VECTOR
     * shoulder angle (deg))
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

        double extensionCurrent = extensionMotor.getCurrent(extensionMetersPerSecond, extensionVoltage);



        

        return null;
        // return VecBuilder.fill(
        //     shoulderDegreesPerSecond,
        //     Math.toDegrees(shoulderRadiansPerSecondSquared),

        // )




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
