package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autos extends RobotContainer{

    
    public Command rightSideAuto() {

        return new SequentialCommandGroup(
            FlyingCircuitUtils.followPath("Right Start to Back"),
            //score on reef,//
            FlyingCircuitUtils.followPath("Right Side to Right Source")
        );

    }
}
