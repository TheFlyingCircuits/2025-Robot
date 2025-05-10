package frc.robot.subsystems.placerGrabber;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UniversalConstants.Direction;


// 2 neo 550's

public class PlacerGrabber extends SubsystemBase {

    /** center to center distance between the two holding positions for coral */
    public static final double innerWidthMeters = ArmConstants.wristWidthMeters;

    /** center to center distance between the two omni wheel axles */
    public static final double outerWidthMeters = ArmConstants.wristOuterWidthMeters;

    private PlacerGrabberIO io;
    private PlacerGrabberIOInputsAutoLogged inputs;

    private SimpleMotorFeedforward motorFeedforward;

    private PIDController frontNeoPID;
    private PIDController sideNeoPID;

    Debouncer leftDebouncer = new Debouncer(0.1, DebounceType.kBoth);
    Debouncer rightDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    boolean leftHasCoral;
    boolean rightHasCoral;

    LinearFilter frontCurrentMovingWindow = LinearFilter.singlePoleIIR(0.2, 0.02);
    double frontRollerAvgCurrent = 0;

    public PlacerGrabber(PlacerGrabberIO io) {
        this.io = io;
        inputs = new PlacerGrabberIOInputsAutoLogged();

        motorFeedforward = new SimpleMotorFeedforward(0, 0);

        frontNeoPID = new PIDController(0,0,0);

        sideNeoPID = new PIDController(0,0,0);
    }

    /** Sets the voltage of the front motor without feedback control. A positive voltage represents an intake. */
    public void setFrontRollerVolts(double volts) {
        io.setFrontNeoVolts(volts);
    }
    
    /** Sets the voltage of the side motor without feedback control. A positive voltage represents an intake. */
    public void setSideRollerVolts(double volts) {
        io.setSideNeoVolts(volts);
    }

    /** Sets the RPM of the front roller with feedback control. A positive rpm represents an intake. */
    public void setFrontRollerRPM(double rpm) {

        double feedforwardVolts = motorFeedforward.calculate(rpm);
        double pidVolts = frontNeoPID.calculate(inputs.frontRollerRPM, rpm);

        io.setFrontNeoVolts(feedforwardVolts + pidVolts);
    }


    public boolean doesHaveCoral() {
        return leftHasCoral || rightHasCoral;
    }

    public boolean hasSingleCoral() {
        return doesHaveCoral() && !doesHaveTwoCoral();
    }

    public boolean doesHaveTwoCoral() {
        return leftHasCoral && rightHasCoral;
    }

    public Direction sideCoralIsIn() {
        if (leftHasCoral)
            return Direction.left;
        else
             return Direction.right;
    }

    public boolean leftHasCoral() {
        return leftHasCoral;
    }
    public boolean rightHasCoral() {
        return rightHasCoral;
    }
    
    public double getFrontRollerAmps() {
        return inputs.frontMotorAppliedCurrent;
    }

    public double getFrontRollerAvgAmps() {
        // 6 as cutoff for intaking
        return frontRollerAvgCurrent;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        frontRollerAvgCurrent = frontCurrentMovingWindow.calculate(inputs.frontMotorAppliedCurrent);

        leftHasCoral = leftDebouncer.calculate(inputs.leftSensorSeesCoral);
        rightHasCoral = rightDebouncer.calculate(inputs.rightSensorSeesCoral);

        
        Logger.processInputs("placerGrabberInputs", inputs);

        Logger.recordOutput("placerGrabber/frontRollerRPM", inputs.frontRollerRPM);
        Logger.recordOutput("placerGrabber/sideRollerRPM", inputs.sideRollerRPM);
        Logger.recordOutput("placerGrabber/frontRollerAvgCurrent", frontRollerAvgCurrent);
    }

    public Command setPlacerGrabberVoltsCommand(double frontRollerVolts, double sideRollervolts) {
        return this.run(() -> {
            setFrontRollerVolts(frontRollerVolts);
            setSideRollerVolts(sideRollervolts);
        });
    }

    public Command stopInstantCommand() { return this.runOnce(() -> {
        setFrontRollerVolts(0);
        setSideRollerVolts(0);
    });}

    public boolean doesHaveCoralBasedOnAmps() {
        return this.frontRollerAvgCurrent > 8.0;
    }

    public Command intakeOrEjectOrStop( BooleanSupplier buttonPressed) {
        // return this.run(() -> {
        //     if (!this.doesHaveCoral()) {
        //         // intake if no corals
        //         this.setFrontRollerVolts(9);
        //         this.setSideRollerVolts(9);
        //     }
        //     else if (this.doesHaveTwoCoral()) {
        //         // spit if two corals
        //         this.setFrontRollerVolts(-9);
        //         this.setSideRollerVolts(-9);
        //     }
        //     else {
        //         // stop if single coral
        //         this.setFrontRollerVolts(0);
        //         this.setSideRollerVolts(0);
        //     }
        // });

        //return setPlacerGrabberVoltsCommand(9, 11);//.withTimeout(9.0/10.0)
        //.andThen(setPlacerGrabberVoltsCommand(0, 0).withTimeout(1.0/10.0)).repeatedly();

        return this.run( () -> {
            setSideRollerVolts(11);
            if (buttonPressed.getAsBoolean()){
                setFrontRollerVolts(-3);
            }
            else {
                setFrontRollerVolts(9);
            }
        });
    }
}
