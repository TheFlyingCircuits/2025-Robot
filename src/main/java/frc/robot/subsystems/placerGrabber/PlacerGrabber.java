package frc.robot.subsystems.placerGrabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.PlayingField.FieldConstants;


// 2 neo 550's

public class PlacerGrabber extends SubsystemBase {

    /** center to center distance between the two holding positions for coral */
    public static final double widthMeters = Units.inchesToMeters(7);

    private PlacerGrabberIO io;
    private PlacerGrabberIOInputsAutoLogged inputs;

    private SimpleMotorFeedforward motorFeedforward;

    private PIDController frontNeoPID;
    private PIDController sideNeoPID;

    LinearFilter frontCurrentMovingWindow = LinearFilter.singlePoleIIR(0.4, 0.02);
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
        return inputs.leftSensorSeesCoral || inputs.rightSensorSeesCoral;
    }

    public boolean hasSingleCoral() {
        return doesHaveCoral() && !doesHaveTwoCoral();
    }

    public boolean doesHaveTwoCoral() {
        return inputs.leftSensorSeesCoral && inputs.rightSensorSeesCoral;
    }

    public Direction sideCoralIsIn() {
        if (inputs.leftSensorSeesCoral)
            return Direction.left;
        else
             return Direction.right;
    }
    
    public double getFrontRollerAvgCurrent() {
        // 6 as cutoff for intaking
        return frontRollerAvgCurrent;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        frontRollerAvgCurrent = frontCurrentMovingWindow.calculate(inputs.frontMotorAppliedCurrent);


        
        Logger.processInputs("placerGrabberInputs", inputs);

        Logger.recordOutput("frontRollerRPM", inputs.frontRollerRPM);
        Logger.recordOutput("sideRollerRPM", inputs.sideRollerRPM);
        Logger.recordOutput("frontRollerAvgCurrent", frontRollerAvgCurrent);
    }

    public Command setPlacerGrabberVoltsCommand(double frontRollerVolts, double sideRollervolts) {
        return this.run(() -> {
            setFrontRollerVolts(frontRollerVolts);
            setSideRollerVolts(sideRollervolts);
        });
    }

    public boolean doesHaveCoralBasedOnAmps() {
        return this.frontRollerAvgCurrent > 8.0;
    }

    public Command intakeOrEjectOrStop() {
        return this.run(() -> {
            if (!this.doesHaveCoral()) {
                // intake if no corals
                this.setFrontRollerVolts(11);
                this.setSideRollerVolts(11);
            }
            else if (this.doesHaveTwoCoral()) {
                // spit if two corals
                this.setFrontRollerVolts(-9);
                this.setSideRollerVolts(-9);
            }
            else {
                // stop if single coral
                this.setFrontRollerVolts(0);
                this.setSideRollerVolts(0);
            }
        });
    }
}
