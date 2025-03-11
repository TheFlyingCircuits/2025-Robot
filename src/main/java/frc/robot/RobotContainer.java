// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldElement;
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

    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    protected final HumanDriver amara = new HumanDriver(1);
    final CommandXboxController amaraController;

    public final Drivetrain drivetrain;
    private ArmIOKraken armIO;
    public final Arm arm;
    private WristIONeo wristIO;
    public final Wrist wrist;
    public final Leds leds;
    public final PlacerGrabber placerGrabber;

    private DigitalInput coastModeButton;
    
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

            wristIO = new WristIONeo();
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

            coastModeButton = new DigitalInput(0);
        }
        
        
        drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity(), true);}));
        // leds.setDefaultCommand(leds.defaultCommand());
        
        
        arm.extension.setDefaultCommand(arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters));

        //wait until extension retracts to lower arm
        arm.shoulder.setDefaultCommand(
            arm.shoulder.safeSetTargetAngleCommand(0)
        );

        wrist.setDefaultCommand(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)); // 10 volts
        placerGrabber.setDefaultCommand(placerGrabber.setPlacerGrabberVoltsCommand(0, 0));

        duncanController = duncan.getXboxController();
        amaraController = amara.getXboxController();

        testBindings();
        // realBindings();
        triggers();

    }

    private int desiredLevel = 2;
    private Direction desiredStalk = Direction.left;

    private void testBindings() {
        // duncanController.a().onTrue(wrist.setTargetPositionCommand(0));
        // duncanController.b().onTrue(wrist.setTargetPositionCommand(45));
        // duncanController.x().onTrue(wrist.setTargetPositionCommand(90));
        // duncanController.y().onTrue(wrist.setTargetPositionCommand(135));


        // duncanController.povDown().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.minExtensionMeters+0.05));
        // duncanController.povRight().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.minExtensionMeters+0.3));
        // duncanController.povLeft().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.maxExtensionMeters-0.3));
        // duncanController.povUp().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.maxExtensionMeters-0.05));

        // duncanController.rightBumper().whileTrue(arm.run(() -> arm.setShoulderVoltage(2)));

        // duncanController.a().onTrue(arm.setShoulderTargetAngleCommand(ArmConstants.armMinAngleDegrees+5));
        // duncanController.b().onTrue(arm.setShoulderTargetAngleCommand(45));
        // duncanController.x().onTrue(arm.setShoulderTargetAngleCommand(90));
        // duncanController.y().onTrue(arm.setShoulderTargetAngleCommand(ArmConstants.armMaxAngleDegrees-5));


        //L2
        // duncanController.rightBumper().whileTrue(
        //     arm.setArmPositionCommand(new ArmPosition(38.2, 0, 0.73))
        //     .alongWith(wrist.setTargetPositionCommand(101)));

        //L3
        // duncanController.rightBumper().whileTrue(
        //     arm.setArmPositionCommand(new ArmPosition(57.8, 0, 0.97))
        //     .alongWith(wrist.setTargetPositionCommand(79)));


        //L4
        // duncanController.rightBumper().whileTrue(arm.shoulder.setTargetAngleCommand(71)
        //     .alongWith(
        //         new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() >= 45).andThen(arm.extension.setTargetLengthCommand(1.57)),
        //         wrist.setTargetPositionCommand(42)
        //     )
        // );

        // duncanController.b().onTrue(new InstantCommand(() -> desiredLevel = 2));
        // duncanController.x().onTrue(new InstantCommand(() -> desiredLevel = 3));
        // duncanController.y().onTrue(new InstantCommand(() -> desiredLevel = 4));
        
        // ReefFace.FRONT_REEF_FACE.getBranches(2);

        //SCOREONREEF
        duncanController.rightBumper().whileTrue(
            scoreOnReefCommand(
                duncan::getRequestedFieldOrientedVelocity, 
                () -> drivetrain.getClosestReefStalk().getBranch(desiredLevel),
                this::isFacingReef));

        //SOURCE
        //35.23, 0.737, 53.5
        // duncanController.b().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(6, 6)
        //     .alongWith(
        //         arm.shoulder.setTargetAngleCommand(35.23),
        //         arm.extension.setTargetLengthCommand(0.737),
        //         wrist.setTargetPositionCommand(53.5)
        //     ).until(() -> placerGrabber.doesHaveCoral())
        // );



        //intake
        // duncanController.rightTrigger()
        //     .whileTrue(
        //         intakeTowardsCoral(duncan::getRequestedFieldOrientedVelocity).until(() -> placerGrabber.doesHaveCoral())
        //             .andThen(new PrintCommand("intake ended!!!!!!!!"))
        // );

        //CLIMB
        //13 wrist, 101.4 shoulder


        //alternate intake sequence for testing
        duncanController.rightTrigger()
            .whileTrue(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> placerGrabber.doesHaveCoral()),
                    intakeTowardsCoral(duncan::getRequestedFieldOrientedVelocity)
                ).andThen(new PrintCommand("intake ended!!!!!!!!"))
        );

                    
        //eject coral
        duncanController.leftBumper().whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(9, 0).until(() -> !placerGrabber.doesHaveCoral())
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(9, 0).withTimeout(0.5))
        );

        //CLIMB PREP
        duncanController.povUp().onTrue(
            new ParallelCommandGroup(
                arm.shoulder.setTargetAngleCommand(101.4),
                wrist.setTargetPositionCommand(0)
            )
        );

        //CLIMB PULL
        duncanController.povDown().whileTrue(
            arm.shoulder.run(() -> arm.setShoulderVoltage(-3))
                .alongWith(
                    wrist.setTargetPositionCommand(13),
                    arm.extension.setTargetLengthCommand(0.75))

        );

        //reset arm
        duncanController.x().onTrue(
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters)
                .alongWith(
                    arm.shoulder.setTargetAngleCommand(ArmConstants.armMinAngleDegrees),
                    wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)
                )   
        );
        
                
        duncanController.a().whileTrue(
            new ConditionalCommand(descoreAlgaeHigh(), descoreAlgaeLow(), () -> drivetrain.getClosestReefFace().getIfAlgaeL3())
        );

        // duncanController.rightTrigger().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(9, 6).until(placerGrabber::doesHaveCoral));
    }

    private void realBindings() {
        duncanController.y().onTrue(new InstantCommand(drivetrain::setRobotFacingForward));

        
        amaraController.rightBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.right));
        amaraController.leftBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.left));

        amaraController.a().onTrue(new InstantCommand(() -> desiredLevel = 2));
        amaraController.x().onTrue(new InstantCommand(() -> desiredLevel = 3));
        amaraController.y().onTrue(new InstantCommand(() -> desiredLevel = 4));

        duncanController.rightBumper().whileTrue(
            scoreOnReefCommand(
                duncan::getRequestedFieldOrientedVelocity, 
                this::getDesiredBranch,
                this::isFacingReef));

        duncanController.leftBumper().whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(9, 0).until(() -> !placerGrabber.doesHaveCoral())
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(9, 0).withTimeout(0.5))
        );

        //reset arm
        duncanController.x().onTrue(
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters)
                .alongWith(
                    arm.shoulder.setTargetAngleCommand(ArmConstants.armMinAngleDegrees),
                    wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)
                )   
        );
        



        // duncanController.rightBumper().whileTrue(
        //     new ScoreOnReef(
        //         drivetrain,
        //         arm,
        //         wrist,
        //         duncan::getRequestedFieldOrientedVelocity,
        //         () -> {return drivetrain.getClosestReefStalk().getBranch(3);},
        //         leds,
        //         () -> placerGrabber.sideCoralIsIn(),
        //         () -> isFacingReef()
        //     )
        // );

        duncanController.rightTrigger()
            .whileTrue(
                intakeTowardsCoral(duncan::getRequestedFieldOrientedVelocity).until(() -> placerGrabber.doesHaveCoral()).withName("intakeTowardsCoral")
            );
            
    
    }

    private void triggers() {
        Trigger inScoringDistance = new Trigger(()-> { // becomes true when distance to nearest reef stalk is within x meters
            Transform2d distanceToNearestStalk = drivetrain.getClosestReefStalk().getPose2d().minus(drivetrain.getPoseMeters());
            return distanceToNearestStalk.getTranslation().getNorm() < 2;
        });

        inScoringDistance.whileTrue(new ReefFaceLED(leds,drivetrain));

        Trigger hasCoral = new Trigger(() -> placerGrabber.doesHaveCoral());
        hasCoral.onTrue(leds.coralControlledCommand())
            .onTrue(duncan.rumbleController(1.0).withTimeout(0.5))
            .onFalse(leds.scoreCompleteCommand());


        inScoringDistance.and(hasCoral).and(DriverStation::isTeleop)
            .whileTrue(arm.shoulder.safeSetTargetAngleCommand(45).repeatedly());

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
    }    

    public ReefBranch getDesiredBranch() {
        if (desiredStalk == Direction.left)
            return drivetrain.getClosestReefFace().getBranches(desiredLevel)[0];
        else
            return drivetrain.getClosestReefFace().getBranches(desiredLevel)[1];
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

    public Command scoreOnReefCommand(Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Supplier<Boolean> isFacingReef) {
        ScoreOnReef align = new ScoreOnReef(drivetrain, arm, wrist, translationController, reefBranch, leds, () -> placerGrabber.sideCoralIsIn(), isFacingReef);
        Command waitForAlignment = new WaitUntilCommand(align::readyToScore);
        Command scoreCoral = scoreCoral();
        return align.raceWith(waitForAlignment.andThen(scoreCoral));
    }

    public Command scoreCoral() {
        return placerGrabber.setPlacerGrabberVoltsCommand(11, 0).until(() -> !placerGrabber.doesHaveCoral())
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(11, 0).withTimeout(1.5));
    }

    private Command intake() {
        return placerGrabber.setPlacerGrabberVoltsCommand(11, 11)
            .alongWith(
                arm.shoulder.setTargetAngleCommand(ArmConstants.armMinAngleDegrees),
                arm.extension.setTargetLengthCommand(0.77),
                wrist.setTargetPositionCommand(0));
    }

    private Command intakeTowardsCoral(Supplier<ChassisSpeeds> howToDriveWhenNoCoralDetected) {

        Command ledCommand = Commands.run(() -> {
            if (drivetrain.getBestCoralLocation().isEmpty()) {
                leds.orange();
            }
            else {
                leds.green();
            }
        });

        return drivetrain.run(() -> {
            // have driver stay in control when the intake camera doesn't see a note
            if (drivetrain.getBestCoralLocation().isEmpty()) {
                drivetrain.fieldOrientedDrive(howToDriveWhenNoCoralDetected.get(), true);
                return;
            }

            // drive towards the note when the intake camera does see a note.
            drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        }).alongWith(intake()).alongWith(ledCommand);
    }

    public Command descoreAlgaeLow() {
        return arm.shoulder.setTargetAngleCommand(34)
            .alongWith(
                new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() > 25).andThen(arm.extension.setTargetLengthCommand(0.77)),
                wrist.setTargetPositionCommand(WristConstants.minAngleDegrees),
                placerGrabber.setPlacerGrabberVoltsCommand(-10, 0),
                drivetrain.run(() -> {
                    Pose2d lineToDriveOn = new Pose2d(
                        drivetrain.getClosestReefFace().getPose2d().getTranslation(),
                        drivetrain.getClosestReefFace().getPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))
                    );

                    drivetrain.fieldOrientedDriveOnALine(
                        duncan.getRequestedFieldOrientedVelocity(), 
                        lineToDriveOn
                    );
                })
        );
    }

    public Command descoreAlgaeHigh() {
        return arm.shoulder.setTargetAngleCommand(47)
            .alongWith(
                new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() > 40).andThen(arm.extension.setTargetLengthCommand(0.93)),
                wrist.setTargetPositionCommand(0),
                placerGrabber.setPlacerGrabberVoltsCommand(10, 0),
                drivetrain.run(() -> {
                    Pose2d lineToDriveOn = new Pose2d(
                        drivetrain.getClosestReefFace().getPose2d().getTranslation(),
                        drivetrain.getClosestReefFace().getPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))
                    );

                    drivetrain.fieldOrientedDriveOnALine(
                        duncan.getRequestedFieldOrientedVelocity(), 
                        lineToDriveOn
                    );
                })
        );
    }

    /*** AUTO ***/

    public Command intakeTowardsCoralInAuto() {
            return drivetrain.run(() -> {
                if (drivetrain.getBestCoralLocation().isEmpty()) {
                    FieldElement sourceSide = drivetrain.getClosestSourceSide();
                    Transform2d pickupLocationRelativeToSource = new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.fromDegrees(180));
                    Pose2d targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
                    drivetrain.pidToPose(targetRobotPose2d, 3);
                } else {
                    drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
                    arm.shoulder.setTargetAngleCommand(0)
                    .alongWith(
                        wrist.setTargetPositionCommand(0))
                    .alongWith(
                        placerGrabber.setPlacerGrabberVoltsCommand(8,8));
    
                }
            }).raceWith(runUntilHasCoral());
        }

    private Command runUntilHasCoral() {
        return new WaitUntilCommand(() -> placerGrabber.doesHaveCoral());
    }


    private Command sourceIntakeIfDoesntHaveCoral() {
        return new SourceIntake(drivetrain, arm, wrist, placerGrabber).onlyIf(() -> !(placerGrabber.doesHaveCoral()));
    }
    
    private Command waitUntilInRangeOfSource() {
        return new WaitUntilCommand(() -> {
            Translation2d robotTranslation = drivetrain.getPoseMeters().getTranslation();
            return robotTranslation.getDistance(drivetrain.getClosestSourceSide().getLocation2d()) < 1;
        });
    }

    public Command rightSideAuto() {

        return new SequentialCommandGroup(
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_E4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_D4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(duncan::getRequestedRobotOrientedVelocity, () -> ReefBranch.BRANCH_C4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(duncan::getRequestedRobotOrientedVelocity, () -> ReefBranch.BRANCH_B4, () -> true)
        );

    }



    public Command leftSideAuto() {

        return new SequentialCommandGroup(
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_J4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_K4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_L4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_A4, () -> true)
        );

    }
}
