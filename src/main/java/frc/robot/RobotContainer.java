// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
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

        }
        else {
            drivetrain = new Drivetrain(
                new GyroIO(){},
                new SwerveModuleIO(){},
                new SwerveModuleIO(){},
                new SwerveModuleIO(){},
                new SwerveModuleIO(){},
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

        controller.rightBumper().onTrue(arm.setShoulderTargetAngle(20));
    }
}
