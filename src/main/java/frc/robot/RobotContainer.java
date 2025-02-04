// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlignWithReef;
import frc.robot.Reefscape.FieldElement;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic shoud actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instlead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final HumanDriver charlie = new HumanDriver(0);
    public final HumanDriver ben = new HumanDriver(1);

    public final Drivetrain drivetrain;
    public final Arm arm;

    public RobotContainer() {

        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain( 
                new GyroIO(){},
                new SwerveModuleIO(){},
                new SwerveModuleIO(){},
                new SwerveModuleIO(){},
                new SwerveModuleIO(){},
                new VisionIOPhotonLib()
            );

            arm = new Arm(new ArmIO(){});

            
            /****** FOR NOODLE *******/
            // drivetrain = new Drivetrain( // fr 0.092041015625, br , 0.0419921875, fl -0.178955078125, bl -0.332763671875
            //     new GyroIOPigeon(),
            //     new SwerveModuleIONeo(7, 8, -0.184814453125, 0), 
            //     new SwerveModuleIONeo(5, 6, 0.044677734375, 3),
            //     new SwerveModuleIONeo(3, 4, -0.3349609375, 2),
            //     new SwerveModuleIONeo(1, 2,  0.088134765625, 1),
            //     new VisionIOPhotonLib()
            // );

        }
        else {
            drivetrain = new Drivetrain(
                new GyroIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new VisionIO() {}
            );

            arm = new Arm(new ArmIOSim());

        }
        
        
        drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(charlie.getRequestedFieldOrientedVelocity(), true);}));


        realBindings();

    }

    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        controller.y().onTrue(new InstantCommand(drivetrain::setRobotFacingForward));

        controller.rightBumper().whileTrue(
            new AlignWithReef(drivetrain, charlie::getRequestedFieldOrientedVelocity, FieldElement.ReefFace.FRONT));

    }
}
