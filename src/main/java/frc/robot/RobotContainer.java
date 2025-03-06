// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.ScoreOnReef;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.leds.ReefFaceLED;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOKraken;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.placerGrabber.PlacerGrabberIO;
import frc.robot.subsystems.placerGrabber.PlacerGrabberIONeo;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIONeo;


public class RobotContainer {

    protected final HumanDriver charlie = new HumanDriver(0);
    protected final HumanDriver ben = new HumanDriver(1);

    public final Drivetrain drivetrain;
    private ArmIOKraken armIO;
    public final Arm arm;
    private WristIONeo wristIO;
    public final Wrist wrist;
    public final Leds leds;
    public final PlacerGrabber placerGrabber;

    private DigitalInput coastModeButton;




    Command waitUntil;
    
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

            armIO = new ArmIOKraken();
            arm = new Arm(armIO);


            PlacerGrabberIONeo placerGrabberIO = new PlacerGrabberIONeo();
            placerGrabber = new PlacerGrabber(placerGrabberIO);

            wristIO = new WristIONeo(placerGrabberIO.getLeftThroughboreEncoder());
            wrist = new Wrist(wristIO);

            leds = new Leds();

            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625
            
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(0, 1, -0.377686, 0, "FL"), 
                new SwerveModuleIOKraken(2, 3, 0.397705, 1, "FR"),
                new SwerveModuleIOKraken(4, 5, 0.238281, 2, "BL"),
                new SwerveModuleIOKraken(6, 7,  -0.370850, 3, "BR"),
                new VisionIOPhotonLib(){}
            );

            coastModeButton = new DigitalInput(0);


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
        
        
        //wait until wrist homes to retract arm
        arm.extension.setDefaultCommand(
            new WaitUntilCommand(() -> true)
                .andThen(arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters)));

        //wait until extension retracts to lower arm
        arm.shoulder.setDefaultCommand(
            new WaitUntilCommand(() -> arm.getExtensionMeters() <= ArmConstants.minExtensionMeters+0.5 && arm.getExtensionMetersPerSecond() < 0.1)
                .andThen(arm.shoulder.setTargetAngleCommand(45))
        );

        wrist.setDefaultCommand(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5));
        placerGrabber.setDefaultCommand(placerGrabber.setPlacerGrabberVoltsCommand(0, 0));

        realBindings();
        // testBindings();
        triggers();

    }

    private void testBindings() {
        CommandXboxController controller = charlie.getXboxController();
        // controller.a().onTrue(wrist.setTargetPositionCommand(0));
        // controller.b().onTrue(wrist.setTargetPositionCommand(45));
        // controller.x().onTrue(wrist.setTargetPositionCommand(90));
        // controller.y().onTrue(wrist.setTargetPositionCommand(135));


        // controller.povDown().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.minExtensionMeters+0.05));
        // controller.povRight().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.minExtensionMeters+0.3));
        // controller.povLeft().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.maxExtensionMeters-0.3));
        // controller.povUp().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.maxExtensionMeters-0.05));

        // controller.rightBumper().whileTrue(arm.run(() -> arm.setShoulderVoltage(2)));

        // controller.a().onTrue(arm.setShoulderTargetAngleCommand(ArmConstants.armMinAngleDegrees+5));
        // controller.b().onTrue(arm.setShoulderTargetAngleCommand(45));
        // controller.x().onTrue(arm.setShoulderTargetAngleCommand(90));
        // controller.y().onTrue(arm.setShoulderTargetAngleCommand(ArmConstants.armMaxAngleDegrees-5));


        //L2
        // controller.rightBumper().whileTrue(
        //     arm.setArmPositionCommand(new ArmPosition(38.2, 0, 0.73))
        //     .alongWith(wrist.setTargetPositionCommand(101)));

        //L3
        // controller.rightBumper().whileTrue(
        //     arm.setArmPositionCommand(new ArmPosition(57.8, 0, 0.97))
        //     .alongWith(wrist.setTargetPositionCommand(79)));


        //L4
        // controller.rightBumper().whileTrue(arm.shoulder.setTargetAngleCommand(71)
        //     .alongWith(
        //         new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() >= 45).andThen(arm.extension.setTargetLengthCommand(1.57)),
        //         wrist.setTargetPositionCommand(42)
        //     )
        // );


        //SCOREONREEF
        controller.rightBumper().whileTrue(
            scoreOnReefCommand(
                charlie::getRequestedFieldOrientedVelocity, 
                () -> ReefBranch.BRANCH_B3,
                () -> isFacingReef()));

        //SOURCE
        //35.23, 0.737, 53.5
        controller.b().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(6, 6)
            .alongWith(
                arm.shoulder.setTargetAngleCommand(35.23),
                arm.extension.setTargetLengthCommand(0.737),
                wrist.setTargetPositionCommand(53.5)
            ).until(() -> placerGrabber.doesHaveCoral())
        );



        //intake
        controller.rightTrigger()
            .whileTrue(
                intakeTowardsCoral(charlie::getRequestedFieldOrientedVelocity).until(() -> placerGrabber.doesHaveCoral())
            );


                    
        //eject coral
        controller.leftBumper().whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(9, 0).withTimeout(0.25)
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(9, -9))
            );


        // controller.rightTrigger().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(9, 6).until(placerGrabber::doesHaveCoral));
    }

    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        controller.y().onTrue(new InstantCommand(drivetrain::setRobotFacingForward));


        controller.rightBumper().whileTrue(
            new SourceIntake(drivetrain, arm, wrist, placerGrabber)
        );

        // controller.rightBumper().whileTrue(
        //     new ScoreOnReef(
        //         drivetrain,
        //         arm,
        //         wrist,
        //         charlie::getRequestedFieldOrientedVelocity,
        //         () -> {return drivetrain.getClosestReefStalk().getBranch(3);},
        //         leds,
        //         () -> placerGrabber.sideCoralIsIn(),
        //         () -> isFacingReef()
        //     )
        // );

        controller.rightTrigger()
            .whileTrue(
                intakeTowardsCoral(charlie::getRequestedFieldOrientedVelocity).until(() -> placerGrabber.doesHaveCoral())
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


        if (RobotBase.isReal()) {
            Trigger coastModeLimitSwitch = new Trigger(() -> coastModeButton.get() && DriverStation.isDisabled());
            coastModeLimitSwitch.toggleOnTrue( //toggles between coast mode and brake mode
                Commands.runOnce(() -> {
                    wristIO.toggleIdleMode();
                    armIO.toggleIdleMode();
                }).ignoringDisable(true)
            );
        }

    }

    /** Called by Robot.java, convenience function for logging. */
    public void periodic() {
        Logger.recordOutput("robotContainer/coastModeLimitSwitch", coastModeButton.get());
        Logger.recordOutput("robotContainer/isFacingReef", isFacingReef());
    }    


    public boolean isFacingReef() {
        if((drivetrain.getClosestReefFace().getOrientation2d().getCos() *
            drivetrain.getPoseMeters().getRotation().getCos()) + 
            (drivetrain.getClosestReefFace().getOrientation2d().getSin() *
            drivetrain.getPoseMeters().getRotation().getSin()) <= 0) {
            return true;
        } else {
            return false;
        }

    }

    public Command sourceIntakeCommand() {
        return new SourceIntake(drivetrain, arm, wrist, placerGrabber);
    }

    public Command scoreOnReefCommand(Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Supplier<Boolean> isFacingReef) {
        return new ScoreOnReef(drivetrain, arm, wrist, translationController, reefBranch, leds, () -> placerGrabber.sideCoralIsIn(), isFacingReef);
    }

    private Command intake() {
        return placerGrabber.setPlacerGrabberVoltsCommand(10, 8)
            .alongWith(
                arm.shoulder.setTargetAngleCommand(0),
                arm.extension.setTargetLengthCommand(0.77),
                wrist.setTargetPositionCommand(-1));
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
        }).alongWith(intake());
    }


    private Command setPlacerGrabberVolts(double sideRollerVolts, double frontRollerVolts) {
        return placerGrabber.run(() -> {
            placerGrabber.setSideRollerVolts(sideRollerVolts);
            placerGrabber.setFrontRollerVolts(frontRollerVolts);
        });
    }
    
    
    public Command intakeTowardsCoralInAuto() {
        return drivetrain.run(() -> {
            drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        });
    }
}
