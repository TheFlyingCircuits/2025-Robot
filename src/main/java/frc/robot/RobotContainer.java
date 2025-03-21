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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.ScoreOnReef;
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
import frc.robot.subsystems.placerGrabber.PlacerGrabberIONeo;
import frc.robot.subsystems.placerGrabber.PlacerGrabberSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIONeo;
import frc.robot.subsystems.wrist.WristIOSim;


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
            wrist = new Wrist(new WristIOSim());
            placerGrabber = new PlacerGrabber(new PlacerGrabberSim());

            leds = new Leds();

            coastModeButton = new DigitalInput(0);
        }
        
        
        drivetrain.setDefaultCommand(drivetrain.drivetrainDefaultCommand(duncan::getRequestedFieldOrientedVelocity));
        leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true));
        
        
        arm.extension.setDefaultCommand(arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters));

        arm.shoulder.setDefaultCommand(
            arm.shoulder.shoulderDefaultCommand(
                () -> placerGrabber.doesHaveCoral() && drivetrain.inScoringDistance(),
                () -> isFacingReef(),
                () -> desiredLevel
            )
        );

        wrist.setDefaultCommand(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-10)); // 10 volts
        placerGrabber.setDefaultCommand(placerGrabber.setPlacerGrabberVoltsCommand(0, 0));

        duncanController = duncan.getXboxController();
        amaraController = amara.getXboxController();

        // testBindings();
        realBindings();
        triggers();

    }

    private int desiredLevel = 2;
    private Direction desiredStalk = Direction.left;

    private boolean visionAssistedIntakeInTeleop = true;

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
            placerGrabber.setPlacerGrabberVoltsCommand(-9, 0).withName("startEject").until(() -> !placerGrabber.doesHaveCoral()).withName("ejectUntilSensor")
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(-9, 0).withName("finishEject").withTimeout(0.5).withName("finishEjectWithTimeout")).withName("fullEjectCommand")
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


   

        //CLIMB PREP
        // duncanController.povUp().onTrue(
        //     new ParallelCommandGroup(
        //         arm.shoulder.setTargetAngleCommand(101.4),
        //         wrist.setTargetPositionCommand(0)
        //     )
        // );

        //CLIMB PULL
        // duncanController.povDown().whileTrue(
        //     arm.shoulder.run(() -> arm.setShoulderVoltage(-3))
        //         .alongWith(
        //             wrist.setTargetPositionCommand(13),
        //             arm.extension.setTargetLengthCommand(0.75))

        // );

        //reset arm
        duncanController.x().onTrue(
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters)
                .alongWith(
                    arm.shoulder.setTargetAngleCommand(ArmConstants.armMinAngleDegrees),
                    wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)
                )
        );

        //DESCORE ALGAE

        // duncanController.povDown().whileTrue(descoreAlgaeSwipe());
    }

    private void realBindings() {
        amaraController.rightBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.right));
        amaraController.leftBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.left));

        amaraController.b().onTrue(new InstantCommand(() -> {desiredLevel = 1; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.a().onTrue(new InstantCommand(() -> {desiredLevel = 2; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.x().onTrue(new InstantCommand(() -> {desiredLevel = 3; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.y().onTrue(new InstantCommand(() -> {desiredLevel = 4; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));


        amaraController.leftTrigger().onTrue(new InstantCommand(() -> {visionAssistedIntakeInTeleop = false;
            Logger.recordOutput("escapeHatch", visionAssistedIntakeInTeleop);}));
        amaraController.rightTrigger().onTrue(new InstantCommand(() -> {visionAssistedIntakeInTeleop = true;
            Logger.recordOutput("escapeHatch", visionAssistedIntakeInTeleop);}));


        //ground intake
        duncanController.rightTrigger()
            .whileTrue(
                new ConditionalCommand(
                intakeTowardsCoral(duncan::getRequestedFieldOrientedVelocity),
                intakeUntilCoralAcquired().alongWith(
                    drivetrain.drivetrainDefaultCommand(duncan::getRequestedFieldOrientedVelocity)),
                () -> visionAssistedIntakeInTeleop)
        );
        
        //trough score
        duncanController.leftTrigger()
            .whileTrue(
                troughScore()
            ).onFalse(
                scoreCoral(true)
                .raceWith(
                    arm.shoulder.setTargetAngleCommand(12.5),
                    arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
                    wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees)
                )
            );

        //reef score
        duncanController.rightBumper()
            .whileTrue(
                scoreOnReefCommand(
                    duncan::getRequestedFieldOrientedVelocity, 
                    this::getDesiredBranch,
                    this::isFacingReef)
                .deadlineFor( // allow command to end if we somehow score before seeing a tag
                    Commands.run(drivetrain::setPoseToVisionMeasurement).until(drivetrain::seesTag)
                )
            );

        //eject
        duncanController.leftBumper().whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(9, -9).until(() -> !placerGrabber.doesHaveCoral())
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(9, -9).withTimeout(0.5))
        );
        
        //descore algae
        duncanController.a().whileTrue(
            new ConditionalCommand(
                descoreAlgaeHigh(),
                descoreAlgaeLow(),
                () -> drivetrain.getClosestReefFace().isHighAlgae())
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
        // duncanController.b().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(6, 6)
        //     .alongWith(
        //         arm.shoulder.setTargetAngleCommand(35.23),
        //         arm.extension.setTargetLengthCommand(0.737),
        //         wrist.setTargetPositionCommand(53.5)
        //     ).until(() -> placerGrabber.doesHaveCoral())
        // );


        //reset gyro
        duncanController.y().onTrue(new InstantCommand(drivetrain::setPoseToVisionMeasurement).repeatedly().until(drivetrain::seesTag));
        // duncanController.y().onTrue(Commands.runOnce(drivetrain::setRobotFacingForward));
        


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

        // removes algae
        duncanController.a().whileTrue(
            new ConditionalCommand(descoreAlgaeHigh(), descoreAlgaeLow(), () -> drivetrain.getClosestReefFace().isHighAlgae())
        );
    }

    private void triggers() {

        Trigger hasCoral = new Trigger(() -> placerGrabber.doesHaveCoral());
        hasCoral.onTrue(leds.strobeCommand(Color.kWhite, 4, 0.5).ignoringDisable(true));
        hasCoral.onFalse(leds.strobeCommand(Color.kYellow, 4, 0.5).ignoringDisable(true));
        hasCoral.onTrue(duncan.rumbleController(1.0).withTimeout(0.5));

        if (RobotBase.isReal()) {
            Trigger coastModeLimitSwitch = new Trigger(() -> coastModeButton.get() && DriverStation.isDisabled());
            coastModeLimitSwitch.toggleOnTrue( //toggles between coast mode and brake mode
                Commands.runOnce(() -> {
                    wristIO.toggleIdleMode();
                    armIO.toggleIdleMode();
                }).ignoringDisable(true)
            );
        }

        // Trigger shouldEjectCoral = new Trigger(placerGrabber::doesHaveTwoCoral).and(DriverStation::isTeleop);
        // shouldEjectCoral.whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(-9, 0).andThen(placerGrabber.setPlacerGrabberVoltsCommand(-9, 0)).withTimeout(0.5));
    }

    /** Called by Robot.java, convenience function for logging. */
    public void periodic() {
        Logger.recordOutput("robotContainer/coastModeLimitSwitch", coastModeButton.get());

        ArmPosition desiredArmState = new ArmPosition();
        desiredArmState.shoulderAngleDegrees = arm.getTargetShoulderAngleDegrees();
        desiredArmState.extensionMeters = arm.getTargetExtensionMeters();
        desiredArmState.wristAngleDegrees = wrist.getTargetWristDegrees();
        AdvantageScopeDrawingUtils.logArmWireframe("arm/desiredWireframe", desiredArmState, drivetrain.getPoseMeters());

        ArmPosition measuredArmState = new ArmPosition();
        measuredArmState.shoulderAngleDegrees = arm.getShoulderAngleDegrees();
        measuredArmState.extensionMeters = arm.getExtensionMeters();
        measuredArmState.wristAngleDegrees = wrist.getWristAngleDegrees();
        AdvantageScopeDrawingUtils.logArmWireframe("arm/measuredWireframe", measuredArmState, drivetrain.getPoseMeters());

        AdvantageScopeDrawingUtils.drawBumpers("wireframeBumpers", drivetrain.getPoseMeters());
    }    

    /**** INTAKE ****/

    private Command intakeUntilCoralAcquired() {
        Command armToIntake = new ParallelCommandGroup(
            arm.shoulder.safeSetTargetAngleCommand(ArmConstants.armMinAngleDegrees),
            arm.extension.setTargetLengthCommand(0.77),
            wrist.setTargetPositionCommand(-3)
        ).withName("armToIntakePositionCommand");

        return armToIntake.raceWith(placerGrabber.intakeOrEjectOrStop().until(placerGrabber::doesHaveCoral))
        .withName("intakeUntilCoralAcquired").alongWith(new ScheduleCommand(leds.playIntakeAnimationCommand(drivetrain::seesAnyCoral)));

        // return armToIntake.alongWith(placerGrabber.setPlacerGrabberVoltsCommand(11, 11));

        // private Command intakeNote() {
        //     return new ScheduleCommand(leds.playIntakeAnimationCommand(() -> {return drivetrain.getBestNoteLocationFieldFrame().isPresent();}).withName("intake animation"))
        //         .alongWith(this.runIntake(false).until(intake::ringJustEnteredIntake));
        // }

    }

    private Command intakeTowardsCoral(Supplier<ChassisSpeeds> howToDriveWhenNoCoralDetected) {
        return drivetrain.driveTowardsCoralCommand(howToDriveWhenNoCoralDetected).withDeadline(intakeUntilCoralAcquired());

        // return drivetrain.run(() -> {
        //     // have driver stay in control when the intake camera doesn't see a coral
        //     if (drivetrain.getBestCoralLocation().isEmpty()) {
        //         drivetrain.fieldOrientedDrive(howToDriveWhenNoCoralDetected.get(), true);
        //         return;
        //     }

        //     // drive towards the coral when the intake camera does see a coral.
        //     drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        // }).raceWith(intakeUntilCoralAcquired());
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
        align.setName("alignToReef");
        Command manualScoreRequested = new WaitUntilCommand(duncanController.b());
        Command waitForAlignment = new WaitUntilCommand(align::readyToScore).withName("waitForAlignmentToReef");
        Command scoreCoral = scoreCoral(false).withName("scoreCoral");
        return align.raceWith(waitForAlignment.raceWith(manualScoreRequested).andThen(scoreCoral)).withName("alignWithReefRace").andThen(
            drivetrain.run(() -> {
                Logger.recordOutput("backingUp", true);
                ChassisSpeeds driveBackwards = this.isFacingReef() ? new ChassisSpeeds(-0.5, 0, 0) : new ChassisSpeeds(0.5, 0, 0);
                drivetrain.robotOrientedDrive(driveBackwards, true);
            }).withTimeout(0.3).andThen(() -> {Logger.recordOutput("backingUp", false);}).withName("dirveBackFromReefCommand")
        ).withName("fullScoreOnReefCommand");
    }

    public Command scoreCoral(boolean troughScore) {

        if (troughScore) {
            return placerGrabber.run(() -> {
                placerGrabber.setFrontRollerVolts(8);
            }).withTimeout(0.12)
                .andThen(placerGrabber.setPlacerGrabberVoltsCommand(8, -8))
                .withTimeout(0.5);
        }
        
        else {
            return placerGrabber.run(() -> {
                    double volts = 11;
                    volts = this.isFacingReef() ? volts : -volts;
                    placerGrabber.setFrontRollerVolts(volts);
                }).until(() -> !placerGrabber.doesHaveCoral()).andThen(placerGrabber.run(() -> {
                    double volts = 11;
                    volts = this.isFacingReef() ? volts : -volts;
                    placerGrabber.setFrontRollerVolts(volts);
                }).withTimeout(0.25)
            );
        }
    }

    public Command troughScore() {
        return drivetrain.run(() -> {
            Pose2d facePose = drivetrain.getClosestReefFace().getPose2d();
            Translation2d translationToReef = drivetrain.getPoseMeters().getTranslation().minus(facePose.getTranslation());
            Rotation2d angleToReefCenter = translationToReef.getAngle().minus(facePose.getRotation());
            Transform2d positionShift = new Transform2d(
                Units.inchesToMeters(17) + Units.inchesToMeters(4.5+1),
                angleToReefCenter.getSin() * translationToReef.getNorm(),
                Rotation2d.k180deg
            );

            drivetrain.pidToPose(facePose.plus(positionShift), 0.5);
        }).alongWith(
                arm.shoulder.setTargetAngleCommand(ArmPosition.frontL1.shoulderAngleDegrees),
                arm.extension.setTargetLengthCommand(ArmPosition.frontL1.extensionMeters),
                wrist.setTargetPositionCommand(ArmPosition.frontL1.wristAngleDegrees)
        );
    }

    public Command descoreAlgaeHigh() {

        Command driveToReef = drivetrain.run(() -> {
                Pose2d reefFacePose = drivetrain.getClosestReefFace().getPose2d();
                Transform2d adjustmentRelativeToFace = new Transform2d(
                    DrivetrainConstants.frameWidthMeters/2 + Units.inchesToMeters(3.5) + FieldConstants.coralOuterDiameterMeters,
                    0,
                    Rotation2d.k180deg);

                drivetrain.pidToPose(reefFacePose.plus(adjustmentRelativeToFace), 2);
            });


        Command waitUntilCloseEnough = new WaitUntilCommand(() -> {
            return drivetrain.getTranslationError() < 0.2;
        });

        Command prepArm = arm.shoulder.setTargetAngleCommand(51)
            .alongWith(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5));

        Command waitThenPunch = new WaitUntilCommand(() -> {
            return arm.getShoulderAngleDegrees() > 45 && arm.getShoulderAngleDegrees() < 56;})
                .andThen(arm.extension.setTargetLengthCommand(1.27))
                .until(() -> arm.getExtensionMeters() > 1.15);

        Command swingUp = arm.shoulder.setTargetAngleCommand(73)
            .alongWith(
                arm.extension.setTargetLengthCommand(1.27),
                wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)
            );

        return driveToReef.raceWith(
            waitUntilCloseEnough.andThen(prepArm),
            waitThenPunch //this command will end, the others never do
        ).andThen(new ScheduleCommand(swingUp));           
    }

    public Command descoreAlgaeLow() {

        Command driveToReef = drivetrain.run(() -> {
                Pose2d reefFacePose = drivetrain.getClosestReefFace().getPose2d();
                Transform2d adjustmentRelativeToFace = new Transform2d(
                    DrivetrainConstants.frameWidthMeters/2 + Units.inchesToMeters(3.5) + FieldConstants.coralOuterDiameterMeters,
                    0,
                    Rotation2d.k180deg);

                drivetrain.pidToPose(reefFacePose.plus(adjustmentRelativeToFace), 2);
            });


        Command waitUntilCloseEnough = new WaitUntilCommand(() -> {
            return drivetrain.getTranslationError() < 0.2;
        });

        Command prepArm = arm.shoulder.setTargetAngleCommand(35)
            .alongWith(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5));

        Command waitThenPunch = new WaitUntilCommand(() -> {
            return arm.getShoulderAngleDegrees() > 26 && arm.getShoulderAngleDegrees() < 46;})
            .andThen(arm.extension.setTargetLengthCommand(0.90))
            .until(() -> arm.getExtensionMeters() > 0.8);

        Command swingUp = arm.shoulder.setTargetAngleCommand(73)
            .alongWith(
                arm.extension.setTargetLengthCommand(0.90),
                wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5)
            );

        return driveToReef.raceWith(
            waitUntilCloseEnough.andThen(prepArm),
            waitThenPunch //this command will end, the others never do
        ).andThen(new ScheduleCommand(swingUp));           
    }


    /*** AUTO ***/

    private Command homeArmWhileGoingToSource() {
        return new ParallelCommandGroup(
            arm.shoulder.safeSetTargetAngleCommand(0),
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
            wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5),
            drivetrain.run(() -> {
                Transform2d poseAdjustment = new Transform2d(2, 0, new Rotation2d());
                Translation2d desiredTranslation = drivetrain.getClosestSourceSide().getPose2d().plus(poseAdjustment).getTranslation();

                Rotation2d desiredRotation = FlyingCircuitUtils.getAllianceDependentValue(Rotation2d.k180deg, Rotation2d.kZero, Rotation2d.kZero);
                drivetrain.pidToPose(new Pose2d(desiredTranslation, desiredRotation), 3.5);
            })
        ).until(() -> arm.getShoulderAngleDegrees() < 40);
    }


    private Command intakeTowardsCoralInAuto() {
            return drivetrain.run(() -> {
                drivetrain.enableRotationAroundIntake();
                if (drivetrain.getBestCoralLocation().isEmpty()) {
                    // can't see coral
                    FieldElement sourceSide = drivetrain.getClosestSourceSide();
                    Transform2d pickupLocationRelativeToSource = new Transform2d(2, 0, Rotation2d.k180deg);
                    Pose2d targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
                    drivetrain.pidToPose(targetRobotPose2d, 1);
                } else {
                    // can see coral
                    // drivetrain.driveTowardsCoral(new ChassisSpeeds()); // auto is accounted for within this function
                    drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
                }
            })
            .raceWith(intakeUntilCoralAcquired()).withName("intakeTowardsCoralInAuto").withDeadline(new InstantCommand(() -> {drivetrain.resetCenterOfRotation();}));
    }
    

    private Command driveTowardsReef() {
        return drivetrain.run(() -> {
                Transform2d poseAdjustment = new Transform2d(2,0, new Rotation2d());
                drivetrain.pidToPose(drivetrain.getClosestSourceSide().getPose2d().plus(poseAdjustment), 3.5);
        }).alongWith(arm.shoulder.setTargetAngleCommand(45))
            .until(() -> {
                Translation2d sourceTranslation = drivetrain.getClosestSourceSide().getPose2d().getTranslation();
                return drivetrain.getPoseMeters().getTranslation().minus(sourceTranslation).getNorm() > 1.5;
        });
    }

    public Command autoChoosingAuto() {
        return new ConditionalCommand(leftSideAuto(), rightSideAuto(), () -> {
            FieldElement closestFaceOnLeft = FieldElement.BACK_LEFT_REEF_FACE;
            FieldElement closestFaceOnRight = FieldElement.BACK_RIGHT_REEF_FACE;

            double distanceToLeft = closestFaceOnLeft.getLocation2d().getDistance(drivetrain.getPoseMeters().getTranslation());
            double distanceToRight = closestFaceOnRight.getLocation2d().getDistance(drivetrain.getPoseMeters().getTranslation());

            return distanceToLeft < distanceToRight;
        }).withName("autoChosingAuto");
    }

    public Command leftSideAuto() {
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

    public Command rightSideAuto() {
        return new SequentialCommandGroup(
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_E4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            driveTowardsReef(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_D4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            driveTowardsReef(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_C4, () -> true),
            homeArmWhileGoingToSource(),
            intakeTowardsCoralInAuto(),
            driveTowardsReef(),
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> ReefBranch.BRANCH_B4, () -> true)
        );

    }
}
