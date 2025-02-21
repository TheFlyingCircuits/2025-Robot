package frc.robot.subsystems.placerGrabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// 2 neo 550's

public class PlacerGrabber extends SubsystemBase {

    private PlacerGrabberIO io;
    private static PlacerGrabberIOInputsAutoLogged inputs;

    private SimpleMotorFeedforward motorFeedforward;

    private PIDController frontNeoPID;
    private PIDController sideNeoPID;

    public PlacerGrabber(PlacerGrabberIO io) {
        this.io = io;
        inputs = new PlacerGrabberIOInputsAutoLogged();


        motorFeedforward = new SimpleMotorFeedforward(0, 0);

        frontNeoPID = new PIDController(1,0,0);

        sideNeoPID = new PIDController(1,0,0);
    }

    public void setFrontRollerRPM(double rpm) {

        double feedforwardVolts = motorFeedforward.calculate(rpm);
        double pidVolts = frontNeoPID.calculate(inputs.frontRollerRPM, rpm);

        io.setFrontNeoVolts(feedforwardVolts + pidVolts);
    }

    public void setSideRollerRPM(double rpm) {

        double feedforwardVolts = motorFeedforward.calculate(rpm);
        double pidVolts = sideNeoPID.calculate(inputs.sideRollerRPM, rpm);

        io.setFrontNeoVolts(feedforwardVolts + pidVolts);
    }

    public static boolean doesHaveCoral() {
        //TODO: need to change based on what canspark the censors are connected to
        
        if (inputs.censorOneSeesCoral || inputs.censorTwoSeesCoral) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("PlacerGrabberInputs", inputs);

        Logger.recordOutput("frontRollerRPM", inputs.frontRollerRPM);
        Logger.recordOutput("sideRollerRPM", inputs.sideRollerRPM);
    }
}
