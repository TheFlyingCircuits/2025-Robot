package frc.robot.subsystems.placerGrabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// 2 neo 550's

public class PlacerGrabber extends SubsystemBase {

    private PlacerGrabberIO io;
    private PlacerGrabberIOInputsAutoLogged inputs;

    private SimpleMotorFeedforward motorFeedforward;

    private PIDController frontNeoPID;
    private PIDController sideNeoPID;

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

    /** Sets the RPM of the side roller with feedback control. A positive rpm represents an intake. */
    public void setSideRollerRPM(double rpm) {

        double feedforwardVolts = motorFeedforward.calculate(rpm);
        double pidVolts = sideNeoPID.calculate(inputs.sideRollerRPM, rpm);

        io.setFrontNeoVolts(feedforwardVolts + pidVolts);
    }

    public boolean doesHaveCoral() {
        return inputs.leftSensorSeesCoral || inputs.rightSensorSeesCoral;
    }

    public String sideCoralIsIn() {
        if (inputs.leftSensorSeesCoral) {
            return("left");
        } else {
            return("right");
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("placerGrabberInputs", inputs);

        Logger.recordOutput("frontRollerRPM", inputs.frontRollerRPM);
        Logger.recordOutput("sideRollerRPM", inputs.sideRollerRPM);
    }

    public Command setPlacerGrabberVoltsCommand(double frontRollerVolts, double sideRollervolts) {
        return this.run(() -> {
            setFrontRollerVolts(frontRollerVolts);
            setSideRollerVolts(sideRollervolts);
        });
    }
}
