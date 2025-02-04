package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

    
    private DCMotor shoulderMotor = DCMotor.getKrakenX60(2).withReduction(ArmConstants.shoulderGearReduction);
    private DCMotor extensionMotor = DCMotor.getKrakenX60(1).withReduction(ArmConstants.extensionGearReduction);

    Matrix<N2, N1> systemInputs = VecBuilder.fill(0, 0);

    public ArmIOSim() {};

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
        inputs.shoulderVelocityDegreesPerSecond = nextState.get(1, 0);
        inputs.shoulderAppliedCurrent = this.systemInputs.get(0, 0);

        inputs.extensionLengthMeters = nextState.get(2, 0);
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


        Logger.recordOutput("arm/shoulderGravityRadiansPerSecondSquared", shoulderGravityRadiansPerSecondSquared);

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
    public void setShoulderMotorAmps(double amps) {
        systemInputs.set(0, 0, amps);
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        systemInputs.set(1, 0, volts);
    }

}
