package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AlignWithReef;

public class Autos extends RobotContainer{

    
    public Command rightSideAuto() {

        return new SequentialCommandGroup(
            FlyingCircuitUtils.followPath("Right Start to Back"),

            // new AlignWithReef(drivetrain, charlie::getRequestedFieldOrientedVelocity, FieldElement.ReefFace.FRONT).withTimeout(5), // autoscore

            FlyingCircuitUtils.followPath("Right Side to Right Source")
        );

    }
}
