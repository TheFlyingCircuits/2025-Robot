// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.ChickenHead;
import frc.robot.commands.RemoveAlgae;
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
import frc.robot.subsystems.wrist.WristIONeo;
import frc.robot.subsystems.wrist.WristIOSim;


public class RobotContainer {

    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    protected final HumanDriver amara = new HumanDriver(1);
    final CommandXboxController amaraController;
    private int desiredLevel = 2;
    private Direction desiredStalk = Direction.left;
    private boolean visionAssistedIntakeInTeleop = false;

    public final Drivetrain drivetrain;
    public final Arm arm;
    public final Wrist wrist;
    public final Leds leds;
    public final PlacerGrabber placerGrabber;
    private DigitalInput coastModeButton = new DigitalInput(0);
    
    public RobotContainer() {

        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(0, 1, -0.377686, 0, "FL"), 
                new SwerveModuleIOKraken(2, 3, 0.397705, 1, "FR"),
                new SwerveModuleIOKraken(4, 5, 0.238281, 2, "BL"),
                new SwerveModuleIOKraken(6, 7,  -0.370850, 3, "BR"),
                new VisionIOPhotonLib(){}
            );

            arm = new Arm(new ArmIOKraken());
            wrist = new Wrist(new WristIONeo());
            placerGrabber = new PlacerGrabber(new PlacerGrabberIONeo());
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
            wrist = new Wrist(new WristIOSim());
            placerGrabber = new PlacerGrabber(new PlacerGrabberSim());
            leds = new Leds();
        }
        
        
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true).withName("ledsDefaultCommand"));
        
        
        arm.extension.setDefaultCommand(arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters).withName("extensionDefaultCommand"));

        arm.shoulder.setDefaultCommand(arm.shoulder.safeSetTargetAngleCommand(0).withName("shoulderDefaultCommand"));

        wrist.setDefaultCommand(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-10).withName("wristDefaultCommand")); // 10 volts
        placerGrabber.setDefaultCommand(placerGrabber.setPlacerGrabberVoltsCommand(0, 0).withName("placerGrabberDefaultCommmand"));

        duncanController = duncan.getXboxController();
        amaraController = amara.getXboxController();

        // testBindings();
        realBindings();
        triggers();

    }


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
                    drivetrain::isFacingReef));
    

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
        // bumpers to select left/right stalk
        amaraController.rightBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.right));
        amaraController.leftBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.left));

        // face buttons to select desired level
        amaraController.b().onTrue(new InstantCommand(() -> {desiredLevel = 1; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.a().onTrue(new InstantCommand(() -> {desiredLevel = 2; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.x().onTrue(new InstantCommand(() -> {desiredLevel = 3; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.y().onTrue(new InstantCommand(() -> {desiredLevel = 4; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));

        if (RobotBase.isSimulation()) { // use a single controller in sim for convenience
            duncanController.povDown().onTrue(new InstantCommand(() -> {desiredLevel = 1; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
            duncanController.povLeft().onTrue(new InstantCommand(() -> {desiredLevel = 2; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
            duncanController.povRight().onTrue(new InstantCommand(() -> {desiredLevel = 3; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
            duncanController.povUp().onTrue(new InstantCommand(() -> {desiredLevel = 4; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        }

        // escape hatch for aim assist while intaking
        amaraController.leftTrigger().onTrue(new InstantCommand(() -> {
            visionAssistedIntakeInTeleop = false;
            Logger.recordOutput("escapeHatch", visionAssistedIntakeInTeleop);
        }));
        amaraController.rightTrigger().onTrue(new InstantCommand(() -> {
            visionAssistedIntakeInTeleop = true;
            Logger.recordOutput("escapeHatch", visionAssistedIntakeInTeleop);
        }));

        if (RobotBase.isSimulation()) {
            visionAssistedIntakeInTeleop = true;
        }


        // ground intake
        duncanController.rightTrigger().and(() -> !visionAssistedIntakeInTeleop).whileTrue(
            intakeUntilCoralAcquired()
        );
        duncanController.rightTrigger().and(() -> visionAssistedIntakeInTeleop).whileTrue(
            intakeUntilCoralAcquired().deadlineFor(new SequentialCommandGroup(
                driverFullyControlDrivetrain().until(this::armInPickupPose),
                driveTowardsCoralTeleop()
            ))
        );
        
        // trough score
        duncanController.leftTrigger().whileTrue(troughScore());
        duncanController.leftTrigger().onFalse(
            scoreCoral(true)
            .raceWith(
                arm.shoulder.setTargetAngleCommand(12.5),
                arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
                wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees)
            )
        );

        // reef score
        duncanController.rightBumper().whileTrue(
            scoreOnReefCommand(
                duncan::getRequestedFieldOrientedVelocity, 
                this::getDesiredBranch,
                drivetrain::isFacingReef)
            .deadlineFor( // allow command to end if we somehow score before seeing a tag
                Commands.run(drivetrain::setPoseToVisionMeasurement).until(drivetrain::seesTag)
            )
        );
        // duncanController.rightBumper().whileTrue(
        //     new ChickenHead(drivetrain, duncan::getRequestedFieldOrientedVelocity, arm, wrist, placerGrabber, () -> FieldElement.STALK_A.getBranch(desiredLevel))
        //     // new ChickenHead(drivetrain, duncan::getRequestedFieldOrientedVelocity, arm, wrist, placerGrabber, () -> drivetrain.getClosestReefStalk().getBranch(desiredLevel))
        // );

        // eject
        duncanController.leftBumper().whileTrue(Commands.sequence(
            placerGrabber.setPlacerGrabberVoltsCommand(9, -9).until(() -> !placerGrabber.doesHaveCoral()),
            placerGrabber.setPlacerGrabberVoltsCommand(9, -9).withTimeout(0.5)
        ));

        // remove algae, then score
        duncanController.a().whileTrue(Commands.sequence(
            removeAlgae(),
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, this::getDesiredBranch, drivetrain::isFacingReef)
        ));

        // reset everything
        duncanController.x().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

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
        


        if (RobotBase.isReal()) { // climb buttons used for reef target level selection in sim
            // climb prep
            duncanController.povUp().onTrue(new ParallelCommandGroup(
                arm.shoulder.setTargetAngleCommand(101.4),
                wrist.setTargetPositionCommand(0)
            ));

            // climb pull
            duncanController.povDown().whileTrue(new ParallelCommandGroup(
                arm.shoulder.run(() -> arm.setShoulderVoltage(-3)),
                wrist.setTargetPositionCommand(13),
                arm.extension.setTargetLengthCommand(0.75)
            ));
        }
    }

    private void triggers() {
        // Coral acquisition
        Trigger hasCoral = new Trigger(() -> placerGrabber.doesHaveCoral());
        hasCoral.onTrue(leds.strobeCommand(Color.kWhite, 4, 0.5).ignoringDisable(true));
        hasCoral.onFalse(leds.strobeCommand(Color.kYellow, 4, 0.5).ignoringDisable(true));
        hasCoral.onTrue(duncan.rumbleController(1.0).withTimeout(0.5));

        // Coast Mode Switch
        Trigger coastModeLimitSwitch = new Trigger(() -> coastModeButton.get() && DriverStation.isDisabled());
        coastModeLimitSwitch.onTrue(Commands.runOnce(() -> { //toggles between coast mode and brake mode
            wrist.toggleIdleMode();
            arm.toggleIdleMode();
        }).ignoringDisable(true));

        // Arm pre-aiming
        Trigger shouldPreAim = hasCoral.and(drivetrain::inScoringDistance)
                               .and(DriverStation::isTeleop)
                               .and(() -> arm.extension.getDefaultCommand().isScheduled());
        
        shouldPreAim.whileTrue(arm.shoulder.waitForRetraction().andThen(arm.shoulder.run(() -> {

            if (!drivetrain.isFacingReef()) {
                arm.setShoulderTargetAngle(60);
                return;
            }

            double preAimAngle = ArmPosition.getPreset(desiredLevel, drivetrain.isFacingReef()).shoulderAngleDegrees;
            arm.setShoulderTargetAngle(preAimAngle);
        })));


        // Pickup in sim
        Command simulatePickup = Commands.run(() -> {
            boolean leftShouldPickup = drivetrain.simulatedIntakeIsNearCoral(Direction.left);
            boolean rightShouldPickup = drivetrain.simulatedIntakeIsNearCoral(Direction.right);

            // latches to true, doesn't turn false till eject (see PlacerGrabberSim.java)
            if (leftShouldPickup) {
                SmartDashboard.putBoolean("placerGrabberSimLeftSensor", leftShouldPickup);
            }
            if (rightShouldPickup) {
                SmartDashboard.putBoolean("placerGrabberSimRightSensor", rightShouldPickup);
            }
        });
        if (RobotBase.isSimulation()) {
            Trigger shouldSimulatePickup = new Trigger(this::armInPickupPose);
            shouldSimulatePickup.whileTrue(simulatePickup);
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
        AdvantageScopeDrawingUtils.logArmWireframe("arm/desiredWireframe", desiredArmState, drivetrain.getPoseMeters(), placerGrabber.leftHasCoral(), placerGrabber.rightHasCoral());

        ArmPosition measuredArmState = new ArmPosition();
        measuredArmState.shoulderAngleDegrees = arm.getShoulderAngleDegrees();
        measuredArmState.extensionMeters = arm.getExtensionMeters();
        measuredArmState.wristAngleDegrees = wrist.getWristAngleDegrees();
        AdvantageScopeDrawingUtils.logArmWireframe("arm/measuredWireframe", measuredArmState, drivetrain.getPoseMeters(), placerGrabber.leftHasCoral(), placerGrabber.rightHasCoral());

        AdvantageScopeDrawingUtils.drawBumpers("wireframeBumpers", drivetrain.getPoseMeters());
    }    


    private Command driverFullyControlDrivetrain() { return drivetrain.run(() -> {
        drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity(), true);
        Logger.recordOutput("drivetrain/runningDefaultCommand", true);
        }).finallyDo(() -> {
            Logger.recordOutput("drivetrain/runningDefaultCommand", false);
        }).withName("driverFullyControlDrivetrain");
    }



    /**** INTAKE ****/

    private Command intakeUntilCoralAcquired() {
        Command armToIntake = new ParallelCommandGroup(
            arm.shoulder.safeSetTargetAngleCommand(ArmConstants.armMinAngleDegrees),
            arm.extension.setTargetLengthCommand(0.77),
            wrist.setTargetPositionCommand(-3)
        ).withName("armToIntakePositionCommand");

        Command spinIntakeWheels = placerGrabber.intakeOrEjectOrStop();
        Command stopIntakeWheels = placerGrabber.runOnce(() -> {
            placerGrabber.setFrontRollerVolts(0);
            placerGrabber.setSideRollerVolts(0);
        });

        return new ParallelCommandGroup(
            armToIntake,
            spinIntakeWheels,
            new ScheduleCommand(leds.playIntakeAnimationCommand(drivetrain::seesAnyCoral))
        ).until(placerGrabber::doesHaveCoral).andThen(stopIntakeWheels).withName("intakeUntilCoralAcquired");
        // stopping the intake wheels here ensures the intake stops when this command is part
        // of a composition (meaning the default command won't take over after this command is finished).
    }
    public boolean armInPickupPose() {
        boolean sholderInPickupPose = Math.abs(arm.getShoulderAngleDegrees()) < 5;
        boolean wristInPickupPose = Math.abs(wrist.getWristAngleDegrees()) < 5;
        return sholderInPickupPose && wristInPickupPose;
    }

    private Command driveTowardsCoralTeleop() { return drivetrain.run(() -> {
        // driver maintains control when the intake cam doesn't see any coral
        Optional<Translation3d> coral = drivetrain.getClosestCoralToEitherIntake();
        if (coral.isEmpty()) {
            drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity(), true);
            return;
        }

        Pose2d pickupPose = drivetrain.getOffsetCoralPickupPose(coral.get());

        // TODO: choose level of assistance
        drivetrain.fieldOrientedDriveWhileAiming(duncan.getRequestedFieldOrientedVelocity(), pickupPose.getRotation());
        // drivetrain.fieldOrientedDriveOnALine(duncan.getRequestedFieldOrientedVelocity(), pickupPose);
        // drivetrain.pidToPose(pickupPose, 2.0);
    }).finallyDo(drivetrain::resetCenterOfRotation);}

    /**** SCORING ****/
    
    public ReefBranch getDesiredBranch() {
        if (desiredStalk == Direction.left)
            return drivetrain.getClosestReefFace().getLeftStalk().getBranch(desiredLevel);
        else
            return drivetrain.getClosestReefFace().getRightStalk().getBranch(desiredLevel);
    }

    public Command scoreOnReefCommand(Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Supplier<Boolean> isFacingReef) {
        ScoreOnReef align = new ScoreOnReef(drivetrain, arm, wrist, translationController, reefBranch, leds, placerGrabber::sideCoralIsIn, isFacingReef);
        align.setName("alignToReef");
        Command manualScoreRequested = new WaitUntilCommand(duncanController.b());
        Command waitForAlignment = new WaitUntilCommand(align::readyToScore).withName("waitForAlignmentToReef");
        Command scoreCoral = scoreCoral(false).withName("scoreCoral");
        Command backup = drivetrain.run(() -> {
            Logger.recordOutput("scoreOnReef/backingUp", true);
            ChassisSpeeds driveBackwards = drivetrain.isFacingReef() ? new ChassisSpeeds(-0.5, 0, 0) : new ChassisSpeeds(0.5, 0, 0);
            drivetrain.robotOrientedDrive(driveBackwards, true);
        }).finallyDo(() -> Logger.recordOutput("scoreOnReef/backingUp", false)).withName("driveBackFromReefCommand");
        return align.raceWith(waitForAlignment.raceWith(manualScoreRequested).andThen(scoreCoral)).withName("alignWithReefRace")
               .andThen(backup.withTimeout(0.3)).withName("fullScoreOnReefCommand"); // TODO: backup may not be desirable in auto?
    }

    public Command scoreCoral(boolean troughScore) {
        Command sendToTrough = Commands.sequence(
            placerGrabber.run(() -> placerGrabber.setFrontRollerVolts(8)).withTimeout(0.12),
            placerGrabber.setPlacerGrabberVoltsCommand(8, -8).withTimeout(0.5)
        );

        if (troughScore) {
            return sendToTrough;
        }

        Supplier<Command> sendToReef = () -> {return placerGrabber.run(() -> {
            double volts = 11;
            volts = drivetrain.isFacingReef() ? volts : -volts;
            placerGrabber.setFrontRollerVolts(volts);
        });};

        return Commands.sequence(
            sendToReef.get().until(() -> !placerGrabber.doesHaveCoral()),
            sendToReef.get().withTimeout(0.25) // TODO: maybe too long for auto?
        );
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

    private Command removeAlgae() { return new ConditionalCommand(
        new RemoveAlgae(drivetrain, arm, wrist, true),
        new RemoveAlgae(drivetrain, arm, wrist, false), 
        () -> drivetrain.getClosestReefFace().isHighAlgae()
    );}


    /*** AUTO ***/

    private Command homeArmWhileGoingToSource() {
        return new ParallelCommandGroup(
            arm.shoulder.safeSetTargetAngleCommand(0),
            arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
            wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5),
            drivetrain.run(() -> {
                Transform2d poseAdjustment = new Transform2d(2, 0, new Rotation2d());
                Translation2d desiredTranslation = drivetrain.getClosestLoadingStation().getPose2d().plus(poseAdjustment).getTranslation();

                Rotation2d desiredRotation = FlyingCircuitUtils.getAllianceDependentValue(Rotation2d.k180deg, Rotation2d.kZero, Rotation2d.kZero);
                drivetrain.pidToPose(new Pose2d(desiredTranslation, desiredRotation), 3.5);
            })
        ).until(() -> arm.getShoulderAngleDegrees() < 40);
    }


    private Command driveTowardsCoralInAuto() { return drivetrain.run(() -> {
        Optional<Translation3d> coral = drivetrain.getClosestCoralToEitherIntake();
        if (coral.isEmpty()) {
            // can't see coral, just drive to source
            FieldElement sourceSide = drivetrain.getClosestLoadingStation();
            Transform2d pickupLocationRelativeToSource = new Transform2d(Units.feetToMeters(2), 0, Rotation2d.k180deg);
            Pose2d targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
            drivetrain.pidToPose(targetRobotPose2d, 1);
        } else {
            // can see coral, drive towards it
            Pose2d pickupPose = drivetrain.getOffsetCoralPickupPose(coral.get());
            drivetrain.pidToPose(pickupPose, 1.0);
        }
    }).finallyDo(drivetrain::resetCenterOfRotation);}
    

    private Command driveTowardsReef() {
        return drivetrain.run(() -> {
                Transform2d poseAdjustment = new Transform2d(2,0, new Rotation2d());
                drivetrain.pidToPose(drivetrain.getClosestLoadingStation().getPose2d().plus(poseAdjustment), 3.5);
        }).alongWith(arm.shoulder.setTargetAngleCommand(45))
            .until(() -> {
                Translation2d sourceTranslation = drivetrain.getClosestLoadingStation().getPose2d().getTranslation();
                return drivetrain.getPoseMeters().getTranslation().minus(sourceTranslation).getNorm() > 1.5;
        });
    }

    public Command autoChoosingAuto() {
        Command leftSideAuto = firstFifteenSecondsCommand(
            ReefBranch.BRANCH_J4,
            ReefBranch.BRANCH_K4,
            ReefBranch.BRANCH_L4,
            ReefBranch.BRANCH_A4
        );

        Command rightSideAuto = firstFifteenSecondsCommand(
            ReefBranch.BRANCH_E4,
            ReefBranch.BRANCH_D4,
            ReefBranch.BRANCH_C4,
            ReefBranch.BRANCH_B4
        );

        BooleanSupplier startingOnLeft = () -> {return drivetrain.getClosestLoadingStation() == FieldElement.LEFT_LOADING_STATION;};

        return new ConditionalCommand(leftSideAuto, rightSideAuto, startingOnLeft);
    }

    public Command firstFifteenSecondsCommand(ReefBranch... targetBranches) {
        SequentialCommandGroup output = new SequentialCommandGroup();
        for (ReefBranch targetBranch : targetBranches) { output.addCommands(
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> targetBranch, () -> true),
            homeArmWhileGoingToSource(),
            intakeUntilCoralAcquired().deadlineFor(driveTowardsCoralInAuto())
        );}
        return output;
    }
}
