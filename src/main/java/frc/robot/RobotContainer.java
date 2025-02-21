// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.ScoreOnReef;
import frc.robot.commands.leds.ReefFaceLED;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.placerGrabber.PlacerGrabberIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic shoud actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instlead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    protected final HumanDriver charlie = new HumanDriver(0);
    protected final HumanDriver ben = new HumanDriver(1);

    public final Drivetrain drivetrain;

    public final Arm arm;
    public final Wrist wrist;
    public final Leds leds;
    public final PlacerGrabber placerGrabber;


    
    public RobotContainer() {

        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            // drivetrain = new Drivetrain( 
            //     new GyroIO(){},
            //     new SwerveModuleIO(){},
            //     new SwerveModuleIO(){},
            //     new SwerveModuleIO(){},
            //     new SwerveModuleIO(){},
            //     new VisionIOPhotonLib()
            // );

            arm = new Arm(new ArmIO(){});
            wrist = new Wrist(new WristIO(){});
            placerGrabber = new PlacerGrabber(new PlacerGrabberIO(){});

            
            /****** FOR NOODLE *******/
            drivetrain = new Drivetrain( // fr 0.092041015625, br , 0.0419921875, fl -0.178955078125, bl -0.332763671875
                new GyroIOPigeon(),
                new SwerveModuleIONeo(7, 8, -0.184814453125, 0), 
                new SwerveModuleIONeo(5, 6, 0.044677734375, 3),
                new SwerveModuleIONeo(3, 4, -0.3349609375, 2),
                new SwerveModuleIONeo(1, 2,  0.088134765625, 1),
                new VisionIOPhotonLib()
            );

            leds = new Leds();

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
            wrist = new Wrist(new WristIO(){});
            placerGrabber = new PlacerGrabber(new PlacerGrabberIO(){});

            leds = new Leds();
        }
        
        
        drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(charlie.getRequestedFieldOrientedVelocity(), true);}));
        leds.setDefaultCommand(leds.defaultCommand());

        realBindings();
        triggers();

    }


    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        controller.y().onTrue(new InstantCommand(drivetrain::setRobotFacingForward));

        controller.x().onTrue(arm.setShoulderTargetAngleCommand(20));
        controller.a().onTrue(arm.setExtensionTargetLengthCommand(1));


        controller.rightBumper().whileTrue(
            new ScoreOnReef(
                drivetrain,
                arm,
                wrist,
                charlie::getRequestedFieldOrientedVelocity,
                () -> {return drivetrain.getClosestReefStalk().getBranch(3);},
                leds
            )
        );

        controller.rightTrigger()
            .whileTrue(
                intakeTowardsCoral(charlie::getRequestedFieldOrientedVelocity)
            );
    
    }

    private void triggers() {
        Trigger inScoringDistance = new Trigger(()-> { // becomes true when distance to nearest reef stalk is within x meters
            Transform2d distanceToNearestStalk = drivetrain.getClosestReefStalk().getPose2d().minus(drivetrain.getPoseMeters());
            return distanceToNearestStalk.getTranslation().getNorm() < 2;
        });
        inScoringDistance.whileTrue(new ReefFaceLED(leds,drivetrain));

        Trigger hasCoral = new Trigger(() -> placerGrabber.doesHaveCoral());
        hasCoral.onTrue(leds.coralControlledCommand());
        hasCoral.onFalse(leds.scoreCompleteCommand());

    }
    public Command scoreOnReefCommand(Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch) {
        return new ScoreOnReef(drivetrain, arm, wrist, translationController, reefBranch,leds);
    }

    private Command intakeTowardsCoral(Supplier<ChassisSpeeds> howToDriveWhenNoCoralDetected) {
        return drivetrain.run(() -> {

            // have driver stay in control when the intake camera doesn't see a note
            if (drivetrain.getBestCoralLocation().isEmpty()) {
                drivetrain.fieldOrientedDrive(howToDriveWhenNoCoralDetected.get(), true);
                return;
            }

            // drive towards the note when the intake camera does see a note.
            drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        });
    }

    public Command sourceIntakeInAuto() { // need to do
        return null;
    }
    
    public Command intakeTowardsCoralInAuto() {
        return drivetrain.run(() -> {
            drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        });
    }
}
