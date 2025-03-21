package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UniversalConstants;
import edu.wpi.first.math.system.NumericalIntegration;

public class ArmIOSim implements ArmIO {

    private TalonFX shoulderTalon = new TalonFX(0, "*");
    private TalonFX extensionTalon = new TalonFX(1, "*");
    
    private DCMotor shoulderMotor = DCMotor.getKrakenX60(2).withReduction(ArmConstants.shoulderGearReduction);
    private DCMotor extensionMotor = DCMotor.getKrakenX60(1).withReduction(ArmConstants.extensionGearReduction);


    Matrix<N2, N1> systemInputs = VecBuilder.fill(0, 0);


    double targetExtensionMeters = ArmConstants.minExtensionMeters;
    double extensionMeters = ArmConstants.minExtensionMeters;
    double targetShoulderAngleDegrees = 0;
    double shoulderAngleDegrees = 0;

     
    public ArmIOSim() {

        // TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

        // shoulderConfig.MotionMagic.MotionMagicAcceleration = 1; //units of rotation per second squared
        // shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
        // shoulderConfig.Slot0.kP = 0; //units of volts per rps
        // shoulderConfig.Slot0.kV = 0.5; //units of volts per rps
        // shoulderConfig.Slot0.kA = 0;

        // shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // shoulderConfig.CurrentLimits.StatorCurrentLimit = 45;
        // shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.CurrentLimits.StatorCurrentLimit = 40; //TODO: find a good value

        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // shoulderConfig.Feedback.RotorToSensorRatio = ArmConstants.shoulderGearReduction;
        shoulderConfig.Feedback.SensorToMechanismRatio = ArmConstants.shoulderGearReduction;
        // shoulderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;//FeedbackSensorSourceValue.FusedCANcoder;
        // shoulderConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.leftPivotEncoderID;

        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 1; //rps of the arm
        shoulderConfig.MotionMagic.MotionMagicAcceleration = 0.8; //rotations per second squared of the arm

        shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        shoulderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.armMaxAngleDegrees / 360;
        shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        shoulderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (ArmConstants.armMinAngleDegrees - 0.5) / 360;

        //torquecurrentfoc constants
        // shoulderConfig.Slot0.kS = 1.5;
        // shoulderConfig.Slot0.kV = 0;
        // shoulderConfig.Slot0.kA = 20;
        // shoulderConfig.Slot0.kG = 3.75;
        // shoulderConfig.Slot0.kP = 0;
        
        shoulderConfig.Slot0.kS = 0.117;
        shoulderConfig.Slot0.kV = 31;
        shoulderConfig.Slot0.kA = 0;
        shoulderConfig.Slot0.kG = 0.22;
        shoulderConfig.Slot0.kP = 125;

        // shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        shoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shoulderTalon.getConfigurator().apply(shoulderConfig);

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

        // extensionConfig.MotionMagic.MotionMagicAcceleration = 5;
        // extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
        // extensionConfig.Slot0.kP = 0.5;
        // extensionConfig.Slot0.kV = 4;
        // extensionConfig.Slot0.kA = 0;

        // extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // extensionConfig.CurrentLimits.StatorCurrentLimit = 45;
        // extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        extensionConfig.Feedback.SensorToMechanismRatio = 1./ArmConstants.extensionMetersPerMotorRotation;

        extensionConfig.CurrentLimits.StatorCurrentLimit = 70;

        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.maxExtensionMeters;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.minExtensionMeters-0.05;

        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 4; //mps of the extension
        extensionConfig.MotionMagic.MotionMagicAcceleration = 6; //m/s^2 of the extension

        extensionConfig.Slot0.kS = ArmConstants.kSExtensionVolts;
        extensionConfig.Slot0.kV = ArmConstants.kVExtensionVoltsSecondsPerRadian;
        extensionConfig.Slot0.kA = ArmConstants.kAExtensionVoltsSecondsSquaredPerRadian;
        extensionConfig.Slot0.kP = ArmConstants.kPExtensionVoltsPerMeter;
        extensionConfig.Slot0.kD = ArmConstants.kDExtensionVoltsPerMeterPerSecond;

        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

        extensionTalon.getConfigurator().apply(extensionConfig);
    };

    public double getExtensionReference() {
        return extensionTalon.getClosedLoopReference().getValueAsDouble();
    }
    public double getShoulderReference() {
        return Units.rotationsToDegrees(shoulderTalon.getClosedLoopReference().getValueAsDouble());
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

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // inputs.shoulderAngleDegrees = SmartDashboard.getNumber("simArmAngleDegs", 0);
        // SmartDashboard.putNumber("simArmAngleDegs", inputs.shoulderAngleDegrees);
        inputs.targetExtensionLengthMeters = this.targetExtensionMeters;
        inputs.targetShoulderAngleDegrees = this.targetShoulderAngleDegrees;
        inputs.extensionLengthMeters = getExtensionReference();
        inputs.shoulderAngleDegrees = getShoulderReference();
        // StatusSignal<Voltage> extensionVoltage = extensionTalon.getMotorVoltage();
        // StatusSignal<Current> shoulderCurrent = shoulderTalon.getTorqueCurrent();

        // this.systemInputs.set(1, 0, extensionVoltage.getValueAsDouble());
        // this.systemInputs.set(0, 0, shoulderCurrent.getValueAsDouble());
    
        // Matrix<N4, N1> state = VecBuilder.fill(
        //     inputs.shoulderAngleDegrees,
        //     inputs.shoulderVelocityDegreesPerSecond,
        //     inputs.extensionLengthMeters,
        //     inputs.extensionLengthMetersPerSecond
        // );
        
        // Matrix<N4, N1> nextState = NumericalIntegration.rk4(
        //     this::calculateSystemDerivative,
        //     state,
        //     this.systemInputs, 
        //     UniversalConstants.defaultPeriodSeconds
        // );

        // inputs.shoulderAngleDegrees = nextState.get(0, 0);
        // // shoulderTalon.setPosition(inputs.shoulderAngleDegrees);
        // inputs.shoulderVelocityDegreesPerSecond = nextState.get(1, 0);
        // inputs.shoulderTorqueCurrent = this.systemInputs.get(0, 0);

        // Logger.recordOutput("arm/closedLoopReference", shoulderTalon.getClosedLoopReference().getValueAsDouble());

        // inputs.extensionLengthMeters = nextState.get(2, 0);
        // // extensionTalon.setPosition(inputs.extensionLengthMeters);

        // inputs.extensionLengthMetersPerSecond = nextState.get(3, 0);
        // inputs.extensionAppliedVolts = this.systemInputs.get(1, 0);
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
                                                        *Math.sin(centerOfMassMeters.getAngle().getRadians());

        double shoulderRadiansPerSecondSquared = shoulderMotorRadiansPerSecondSquared + shoulderGravityRadiansPerSecondSquared;
        shoulderRadiansPerSecondSquared -= Math.signum(shoulderDegreesPerSecond)*5; //friction

        

        double extensionMetersPerSecond = state.get(3, 0);
        double extensionVoltage = input.get(1, 0);
        double extensionCurrentAmps = extensionMotor.getCurrent(extensionMetersPerSecond, extensionVoltage);
        double extensionTorqueNM = extensionMotor.KtNMPerAmp * extensionCurrentAmps;


        //double extensionMetersPerSecondSquared = extensionKv * extensionVoltage;
        double extensionMetersPerSecondSquared = extensionTorqueNM / ArmConstants.extensionPulleyRadiusMeters;
        

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
        StatusCode code = shoulderTalon.setControl(new MotionMagicVoltage(Units.degreesToRotations(degrees)));
        this.targetShoulderAngleDegrees = degrees;
        // System.out.println("shoulder target angle set:" + code.getName());
    }

    @Override
    public void setExtensionTargetLength(double meters) {
        this.targetExtensionMeters = meters;
        //all units for extension are with meters
        StatusCode code = extensionTalon.setControl(new MotionMagicVoltage(meters));
        // System.out.println("extension target length set:" + code.getName());
    }



    @Override
    public void setShoulderMotorVolts(double volts) {
        systemInputs.set(1, 0, volts);
    }

}
