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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
import frc.robot.subsystems.arm.ArmPosition;
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
        leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true));
        
        
        arm.extension.setDefaultCommand(arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters));

        //wait until extension retracts to lower arm
        arm.shoulder.setDefaultCommand(
            arm.shoulder.safeSetTargetAngleCommand(0)
        );

        wrist.setDefaultCommand(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-10)); // 10 volts
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

        // front side scoring
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

        // pivot side scoring
        //L3
        // duncanController.b().whileTrue(
        //     arm.shoulder.setTargetAngleCommand(108)
        //         .alongWith(
        //             new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() >= 100)
        //                 .andThen(arm.extension.setTargetLengthCommand(0.83)),
        //             wrist.setTargetPositionCommand(105)
        //     ));

        // //L4
        // duncanController.y().whileTrue(
        //     arm.shoulder.setTargetAngleCommand(95)
        //         .alongWith(
        //             new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() >= 80)
        //                 .andThen(arm.extension.setTargetLengthCommand(1.59)),
        //             wrist.setTargetPositionCommand(144)
        //     ));

        //eject
        duncanController.leftBumper().whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(-9, 0).until(() -> !placerGrabber.doesHaveCoral())
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(-9, 0).withTimeout(0.5))
        );
                



        duncanController.b().onTrue(new InstantCommand(() -> desiredLevel = 2));
        duncanController.x().onTrue(new InstantCommand(() -> desiredLevel = 3));
        duncanController.y().onTrue(new InstantCommand(() -> desiredLevel = 4));
        

        // SCOREONREEF
        duncanController.rightBumper().whileTrue(
                scoreOnReefCommand(
                    duncan::getRequestedFieldOrientedVelocity, 
                    () -> drivetrain.getClosestReefStalk().getBranch(desiredLevel),
                    this::isFacingReef));
    

        //intake with chase
        // duncanController.rightTrigger()
        //     .whileTrue(
        //         intakeTowardsCoral(duncan::getRequestedFieldOrientedVelocity).until(() -> placerGrabber.doesHaveCoral())
        //             .andThen(new PrintCommand("intake ended!!!!!!!!"))
        // );


        //alternate intake sequence for testing
        duncanController.rightTrigger()
            .whileTrue(
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> placerGrabber.doesHaveCoral()),
                    intakeTowardsCoral(duncan::getRequestedFieldOrientedVelocity)
                ).withName("intakeTowardsCoral")
                
                .andThen(new PrintCommand("intake ended!!!!!!!!"))
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


        
    }

    private void realBindings() {
        amaraController.rightBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.right));
        amaraController.leftBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.left));

        amaraController.a().onTrue(new InstantCommand(() -> desiredLevel = 2));
        amaraController.x().onTrue(new InstantCommand(() -> desiredLevel = 3));
        amaraController.y().onTrue(new InstantCommand(() -> desiredLevel = 4));


        //ground intake
        duncanController.rightTrigger()
            .whileTrue(
                intake().until(() -> placerGrabber.doesHaveCoral()).withName("intake")
        );
        

        //reef score
        duncanController.rightBumper().whileTrue(
                scoreOnReefCommand(
                    duncan::getRequestedFieldOrientedVelocity, 
                    this::getDesiredBranch,
                    this::isFacingReef))
            .onFalse(
                drivetrain.run(() -> {
                    ChassisSpeeds driveBackwards = this.isFacingReef() ? new ChassisSpeeds(-0.5, 0, 0) : new ChassisSpeeds(0.5, 0, 0);
                    drivetrain.robotOrientedDrive(driveBackwards, true);
                })
            );

        //eject
        duncanController.leftBumper().whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(9, 0).until(() -> !placerGrabber.doesHaveCoral())
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(9, 0).withTimeout(0.5))
        );
        
        //descore algae
        duncanController.a().whileTrue(
            new ConditionalCommand(descoreAlgaeHigh(), descoreAlgaeLow(), () -> drivetrain.getClosestReefFace().getIfAlgaeL3())
        );

        //reset arm
        duncanController.x().onTrue(
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters)
                .alongWith(
                    arm.shoulder.setTargetAngleCommand(ArmConstants.armMinAngleDegrees),
                    wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)
                )   
        );

        //source intake
        duncanController.b().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(6, 6)
            .alongWith(
                arm.shoulder.setTargetAngleCommand(35.23),
                arm.extension.setTargetLengthCommand(0.737),
                wrist.setTargetPositionCommand(53.5)
            ).until(() -> placerGrabber.doesHaveCoral())
        );


        //reset gyro
        duncanController.y().onTrue(new InstantCommand(drivetrain::setRobotFacingForward));
        


        //climb prep
        duncanController.povUp().onTrue(
            new ParallelCommandGroup(
                arm.shoulder.setTargetAngleCommand(101.4),
                wrist.setTargetPositionCommand(0)
            )
        );

        //climb pull
        duncanController.povDown().whileTrue(
            arm.shoulder.run(() -> arm.setShoulderVoltage(-3))
                .alongWith(
                    wrist.setTargetPositionCommand(13),
                    arm.extension.setTargetLengthCommand(0.75))

        );
    }

    private void triggers() {
        Trigger inScoringDistance = new Trigger(()-> { // becomes true when distance to nearest reef stalk is within x meters
            Transform2d distanceToNearestStalk = drivetrain.getClosestReefStalk().getPose2d().minus(drivetrain.getPoseMeters());
            return distanceToNearestStalk.getTranslation().getNorm() < 2;
        });

        inScoringDistance.whileTrue(new ReefFaceLED(leds,drivetrain));

        Trigger hasCoral = new Trigger(() -> placerGrabber.doesHaveCoral());
        hasCoral.onTrue(new ScheduleCommand(leds.coralControlledCommand()))
            .onTrue(duncan.rumbleController(1.0).withTimeout(0.5))
            .onFalse(new ScheduleCommand(leds.scoreCompleteCommand()));


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

    /**** INTAKE ****/

    private Command intake() {
        return placerGrabber.setPlacerGrabberVoltsCommand(11, 11)
            .alongWith(
                arm.shoulder.safeSetTargetAngleCommand(ArmConstants.armMinAngleDegrees),
                arm.extension.setTargetLengthCommand(0.77),
                wrist.setTargetPositionCommand(0));
    }

    private Command intakeTowardsCoral(Supplier<ChassisSpeeds> howToDriveWhenNoCoralDetected) {

        Command ledCommand = new ScheduleCommand(leds.run(() -> {
            if (drivetrain.getBestCoralLocation().isEmpty()) {
                leds.orange();
            }
            else {
                leds.green();
            }
        }));

        return drivetrain.run(() -> {
            // have driver stay in control when the intake camera doesn't see a coral
            if (drivetrain.getBestCoralLocation().isEmpty()) {
                drivetrain.fieldOrientedDrive(howToDriveWhenNoCoralDetected.get(), true);
                return;
            }

            // drive towards the coral when the intake camera does see a coral.
            drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        }).alongWith(intake()).alongWith(ledCommand);
    }

    /**** SCORING ****/
    
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
        return align.raceWith(waitForAlignment.andThen(scoreCoral)).andThen(
            drivetrain.run(() -> {
                ChassisSpeeds driveBackwards = this.isFacingReef() ? new ChassisSpeeds(-0.5, 0, 0) : new ChassisSpeeds(0.5, 0, 0);
                drivetrain.robotOrientedDrive(driveBackwards, true);
            }).withTimeout(0.3)
        );
    }

    public Command scoreCoral() {
        return placerGrabber.run(() -> {
                double volts = this.isFacingReef() ? 11 : -11;
                placerGrabber.setFrontRollerVolts(volts);
            }).until(() -> !placerGrabber.doesHaveCoral()).andThen(placerGrabber.run(() -> {

                double volts = this.isFacingReef() ? 11 : -11;
                placerGrabber.setFrontRollerVolts(volts);

            }).withTimeout(0.25)
        );
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

    private Command homeArmWhileGoingToSource() {
        return new ParallelCommandGroup(
            arm.shoulder.safeSetTargetAngleCommand(0),
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
            wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5),
            drivetrain.run(() -> {
                Transform2d poseAdjustment = new Transform2d(2, 0, new Rotation2d());
                drivetrain.pidToPose(drivetrain.getClosestSourceSide().getPose2d().plus(poseAdjustment), 3.5);
            })
        ).until(() -> arm.getShoulderAngleDegrees() < 30);
    }


    private Command intakeTowardsCoralInAuto() {
            return drivetrain.run(() -> {
                if (drivetrain.getBestCoralLocation().isEmpty()) {
                    FieldElement sourceSide = drivetrain.getClosestSourceSide();
                    Transform2d pickupLocationRelativeToSource = new Transform2d(2, 0, Rotation2d.fromDegrees(180));
                    Pose2d targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
                    drivetrain.pidToPose(targetRobotPose2d, 1);
                } else {
                    drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
                }
            })
                .alongWith(intake())
                .raceWith(new WaitUntilCommand(() -> placerGrabber.doesHaveCoral()))
                .andThen(intake().withTimeout(0.25))
                .andThen(new InstantCommand(() -> {
                    placerGrabber.setFrontRollerVolts(0);
                    placerGrabber.setSideRollerVolts(0);
                }));
    }

    private Command driveTowardsReef() {
        return drivetrain.run(() -> {
                Transform2d poseAdjustment = new Transform2d(2, 0, new Rotation2d());
                drivetrain.pidToPose(drivetrain.getClosestSourceSide().getPose2d().plus(poseAdjustment), 3.5);
        }).alongWith(arm.shoulder.setTargetAngleCommand(45))
            .until(() -> {
                Translation2d sourceTranslation = drivetrain.getClosestSourceSide().getPose2d().getTranslation();
                return drivetrain.getPoseMeters().getTranslation().minus(sourceTranslation).getNorm() > 1.5;
        });
    }

    //DO NOT USE
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
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_J4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_K4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_L4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_A4, () -> true)
        );
    }

    public Command leftSideAuto() {
        return new SequentialCommandGroup(
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_J4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            driveTowardsReef(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_K4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            driveTowardsReef(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_L4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            driveTowardsReef(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_A4, () -> true)
        );

    }
}
