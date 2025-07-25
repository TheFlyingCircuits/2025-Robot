// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.AimAtBarge;
import frc.robot.commands.AimAtReef;
import frc.robot.commands.PickupAlgae;
import frc.robot.commands.RemoveAlgae;
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
    private boolean hasAlgae = false;
    private boolean proscessorScoringMode = false;

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
                new VisionIO() {} //VisionConstants.useNewSingleTagCodeFromBuckeye ? new VisionIO() {} : new VisionIOPhotonLib()
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

        setDefaultCommands();
        


        duncanController = duncan.getXboxController();
        amaraController = amara.getXboxController();

        realBindings();
        triggers();
    }

    private void realBindings() {
        // bumpers to select left/right stalk
        amaraController.rightBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.right));
        amaraController.leftBumper().onTrue(new InstantCommand(() -> desiredStalk = Direction.left));

        // face buttons to select desired level
        // amaraController.b().onTrue(new InstantCommand(() -> {desiredLevel = 1; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.a().onTrue(new InstantCommand(() -> {desiredLevel = 2; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.x().onTrue(new InstantCommand(() -> {desiredLevel = 3; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
        amaraController.y().onTrue(new InstantCommand(() -> {desiredLevel = 4; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));

        if (RobotBase.isSimulation()) { // use a single controller in sim for convenience
            // duncanController.povDown().onTrue(new InstantCommand(() -> {desiredLevel = 1; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
            duncanController.povLeft().onTrue(new InstantCommand(() -> {desiredLevel = 2; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
            duncanController.povRight().onTrue(new InstantCommand(() -> {desiredLevel = 3; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));
            duncanController.povUp().onTrue(new InstantCommand(() -> {desiredLevel = 4; Logger.recordOutput("amaraDesiredLevel", desiredLevel);}));

                        // climb prep

        }

        // escape hatch for aim assist while intaking
        amaraController.leftTrigger().onTrue(new InstantCommand(() -> visionAssistedIntakeInTeleop = false));
        amaraController.rightTrigger().onTrue(new InstantCommand(() -> visionAssistedIntakeInTeleop = true));

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


        // FOR TESTING LOLLIPOP PICKUP
        // duncanController.rightTrigger().and(() -> visionAssistedIntakeInTeleop).whileTrue(
        //     intakeUntilCoralAcquired().deadlineFor(new SequentialCommandGroup(
        //         driverFullyControlDrivetrain().until(this::armInPickupPose),
        //         lollipopPickupInAuto(FieldElement.RIGHT_LOLLIPOP, true)
        //     ))
        // );

        
        // trough score
        duncanController.leftTrigger().whileTrue(troughScore());
        duncanController.leftTrigger().onFalse(
            scoreCoral(true)
            .raceWith(
                arm.shoulder.setTargetAngleCommand(ArmPosition.frontL1.shoulderAngleDegrees),
                arm.extension.setTargetLengthCommand(ArmPosition.frontL1.extensionMeters),
                wrist.setTargetPositionCommand(ArmPosition.frontL1.wristAngleDegrees)
            )
        );

        // reef score
        duncanController.rightBumper().and(() -> !hasAlgae).whileTrue(
            scoreOnReefCommand(
                duncan::getRequestedFieldOrientedVelocity, 
                this::getDesiredBranch,
                drivetrain::isFacingReef)
            .deadlineFor( // using "deadlineFor" instead of "alongWith" allows the command to end if we somehow score before seeing a tag
                Commands.run(drivetrain::fullyTrustVisionNextPoseUpdate)
                // Note: pressing Y again while Larry was dancing was enought to stop the dance.
                //       This indicates that trusting the cameras more up close may eliminate some
                //       of the dancing?
                // TODO: check practice field log around 5:00pm on April 3rd 2025 for example of dancing
            ).andThen(stowArm().alongWith(backAwayFromReef(0.5)).withTimeout(0.3))
        );

        duncanController.rightBumper().and(() -> hasAlgae).whileTrue(
            scoreOnBarge()
        ).onFalse(
            new InstantCommand(() -> this.hasAlgae = false)
            .alongWith(
                arm.shoulder.safeSetTargetAngleCommand(0),
                wrist.setTargetPositionCommand(WristConstants.homeAngleDegrees)
            )
        );

        // duncanController.rightBumper().whileTrue(
        //     // new ChickenHead(drivetrain, duncan::getRequestedFieldOrientedVelocity, arm, wrist, placerGrabber, () -> FieldElement.STALK_B.getBranch(desiredLevel))
        //     new ChickenHead(drivetrain, duncan::getRequestedFieldOrientedVelocity, arm, wrist, placerGrabber, () -> drivetrain.getClosestReefStalk().getBranch(desiredLevel))
        //     // new ChickenHead(drivetrain, duncan::getRequestedFieldOrientedVelocity, arm, wrist, placerGrabber, this::getDesiredBranch)
        // );
        // duncanController.rightBumper().onTrue(new DashboardControlArm(arm, wrist));
        // duncanController.b().onTrue(scoreCoral(false));

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

        //algae pickup
        duncanController.b().and(() -> !hasAlgae).whileTrue(
            pickupAlgae()
        ).onFalse(
            new InstantCommand(() -> this.hasAlgae = true)
        );
        
        duncanController.b().and(() -> hasAlgae).whileTrue(
            processorScore() // DON'T actually score with this use to eject algae of get out of algae mode press start for score
        ).onFalse(
            new InstantCommand(() -> this.hasAlgae = false)
        );

        duncanController.back().and(() -> wrist.getWristAngleDegrees() > 5).onTrue(
            arm.shoulder.setTargetAngleCommand(13.71) // wrist -51, extend .787, shoulder 13.711
            .alongWith(arm.extension.setTargetLengthCommand(0.787))
            .alongWith(placerGrabber.setPlacerGrabberVoltsCommand(10, 0))
            .alongWith(new ConditionalCommand(wrist.setTargetPositionCommand(-51), new InstantCommand(), 
            () -> ((arm.getShoulderAngleDegrees() > 12) && (arm.getExtensionMeters() > .72))))
        );

        duncanController.back().and(() -> wrist.getWristAngleDegrees() < 5).whileTrue(
            new InstantCommand(() -> this.hasAlgae = true, wrist)
        );
        
        duncanController.start().and(() -> proscessorScoringMode).and(() -> (arm.getShoulderAngleDegrees() < 20)).whileTrue(
            placerGrabber.setPlacerGrabberVoltsCommand(-11, 0)
            .finallyDo(() -> this.proscessorScoringMode = false)
            .alongWith(new InstantCommand(() -> this.hasAlgae = false))
        );

        duncanController.start().and(() -> (!proscessorScoringMode) || (arm.getShoulderAngleDegrees() > 20)).whileTrue(
            new InstantCommand()
        ).onFalse(new InstantCommand(() -> this.proscessorScoringMode = true));

        new Trigger(() -> proscessorScoringMode).whileTrue(
            arm.shoulder.setTargetAngleCommand(3)
            .alongWith(arm.extension.setTargetLengthCommand(0.77))
            .alongWith(wrist.setTargetPositionCommand(0))
        );



        // reset everything
        duncanController.x().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));



        // reset gyro and recover from collisions that cause big wheel slip
        duncanController.y().onTrue(reSeedRobotPose());
        // duncanController.y().onTrue(Commands.runOnce(drivetrain::setRobotFacingForward));
        


        if (RobotBase.isReal()) { // climb buttons are re-purposed as reef target level selection in sim
            // climb prep
            duncanController.povUp().onTrue(new ParallelCommandGroup(
                arm.shoulder.setTargetAngleCommand(101.4),
                wrist.setTargetPositionCommand(0)
            ));

            // climb pull
            duncanController.povDown().onTrue(new ParallelCommandGroup(
                arm.shoulder.run(() -> arm.setShoulderVoltage(-5)),
                wrist.setTargetPositionCommand(13),
                arm.extension.setTargetLengthCommand(0.75)
            ));
        }
    }
    public void setDefaultCommands() {
        arm.extension.setDefaultCommand(
            new ConditionalCommand(
                arm.extension.setTargetLengthCommand(.77),
                arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
                () -> this.hasAlgae).withName("extensionDefaultCommand"));
            
        arm.shoulder.setDefaultCommand(
            new ConditionalCommand(
                arm.shoulder.setTargetAngleCommand(80),
                arm.shoulder.safeSetTargetAngleCommand(0),
                () -> this.hasAlgae).withName("shoulderDefaultCommand"));

        wrist.setDefaultCommand(
            new ConditionalCommand(
                wrist.setTargetPositionCommand(0),
                wrist.setTargetPositionCommand(WristConstants.homeAngleDegrees),
                () -> this.hasAlgae).withName("wristDefaultCommand")); // 10 volts
        
        placerGrabber.setDefaultCommand(
            new ConditionalCommand(
                placerGrabber.setPlacerGrabberVoltsCommand(1, 0),
                placerGrabber.setPlacerGrabberVoltsCommand(0, 0),
                () -> this.hasAlgae).withName("placerGrabberDefaultCommmand"));
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true).withName("ledsDefaultCommand"));
    }

    private Command reSeedRobotPose() {return Commands.run(() -> {
        drivetrain.fullyTrustVisionNextPoseUpdate();
        drivetrain.allowTeleportsNextPoseUpdate();
    }).until(drivetrain::seesAcceptableTag).ignoringDisable(true);}

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
                arm.setShoulderTargetAngle(90);
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
        Logger.recordOutput("coralTracking/enabledInTeleop", visionAssistedIntakeInTeleop);
        Logger.recordOutput("robotContainer/hasAlgae", hasAlgae);

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

    private Command stowArm() { return Commands.parallel(
        new ConditionalCommand(
            arm.shoulder.setTargetAngleCommand(80), 
            arm.shoulder.setTargetAngleCommand(0),
            () -> this.hasAlgae),
        arm.extension.setTargetLengthCommand(ArmConstants.minExtensionMeters),
        new ConditionalCommand(
            wrist.setTargetPositionCommand(0), 
            wrist.setTargetPositionCommand(WristConstants.homeAngleDegrees),
            () -> this.hasAlgae)
    );}



    /**** INTAKE ****/

    private Command intakeUntilCoralAcquired() {
        Command armToIntake = new ParallelCommandGroup(
            arm.shoulder.safeSetTargetAngleCommand(ArmConstants.armMinAngleDegrees),
            arm.extension.setTargetLengthCommand(0.77),
            wrist.setTargetPositionCommand(0) //wrist.setDutyCycleCommand()
        ).withName("armToIntakePositionCommand");

        Command spinIntakeWheels = placerGrabber.intakeOrEjectOrStop(duncanController.b());
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
        boolean sholderInPickupPose = Math.abs(arm.getShoulderAngleDegrees()) < 45;
        boolean wristInPickupPose = Math.abs(wrist.getWristAngleDegrees()) < 30;
        return sholderInPickupPose && wristInPickupPose;
    }

    private Command driveTowardsCoralTeleop() { return drivetrain.run(() -> {
        // driver maintains control when the intake cam doesn't see any coral,
        // or when using the right stick to change targets.
        drivetrain.setIntakeToActualSize();
        Optional<Pose3d> coral = drivetrain.getClosestCoralToEitherIntake();
        ChassisSpeeds driverRequest = duncan.getRequestedFieldOrientedVelocity();
        // TODO: tune override ratio
        boolean significantRotationRequested = Math.abs(driverRequest.omegaRadiansPerSecond) > 0.005;
        boolean driverOverridingSelectedCoral = coral.isPresent() && significantRotationRequested;
        Logger.recordOutput("coralTracking/driverOverridingSelectedTarget", driverOverridingSelectedCoral);
        if (coral.isEmpty() || driverOverridingSelectedCoral) {
            drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity(), true);
            return;
        }


        // TODO: choose level of assistance
        Pose2d pickupPose = drivetrain.getOffsetCoralPickupPose(coral.get());
        drivetrain.fieldOrientedDriveWhileAiming(duncan.getRequestedFieldOrientedVelocity(), pickupPose.getRotation());
        // drivetrain.fieldOrientedDriveOnALine(duncan.getRequestedFieldOrientedVelocity(), pickupPose);
        // drivetrain.pidToPose(pickupPose, 1.0);
    }).finallyDo(drivetrain::resetCenterOfRotation);}

    private Command pickupAlgae() {

        return new ConditionalCommand(
            new PickupAlgae(true, arm, placerGrabber, drivetrain, wrist, duncan::getRequestedFieldOrientedVelocity),
            new PickupAlgae(false, arm, placerGrabber, drivetrain, wrist, duncan::getRequestedFieldOrientedVelocity),
            () -> drivetrain.getClosestReefFace().isHighAlgae()
        ).until(() -> placerGrabber.getFrontRollerAvgAmps() > 12)
            .andThen(backAwayFromReef(0.75)); //this command should never end so that hasAlgae can switch to false
            
    }

    /**** SCORING ****/
    
    public ReefBranch getDesiredBranch() {
        if (desiredStalk == Direction.left)
            return drivetrain.getClosestReefFace().getLeftStalk().getBranch(desiredLevel);
        else
            return drivetrain.getClosestReefFace().getRightStalk().getBranch(desiredLevel);
    }

    public Command scoreOnReefCommand(Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Supplier<Boolean> isFacingReef) {
        AimAtReef aim = new AimAtReef(drivetrain, arm, wrist, translationController, reefBranch, leds, placerGrabber::sideCoralIsIn, isFacingReef);
        // ChickenHead aim = new ChickenHead(drivetrain, duncan::getRequestedFieldOrientedVelocity, arm, wrist, placerGrabber, () -> drivetrain.getClosestReefStalk().getBranch(desiredLevel));
        Command succCoral = placerGrabber.startEnd(
            () -> placerGrabber.setSideRollerVolts(8),
            () -> placerGrabber.setSideRollerVolts(0)
        ).withTimeout(0.25);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToScore);
        Command scoreCoral = scoreCoral(false);
        Command manualScoreRequested = new WaitUntilCommand(duncanController.b());

        //suck the coral while we are waiting for alignment to avoid command errors
        Command waitPlusSucc = waitForAlignment.raceWith(manualScoreRequested).deadlineFor(succCoral);

        return aim.withDeadline(waitPlusSucc.andThen(scoreCoral));
    }

    public Command backAwayFromReef(double speedMetersPerSecond) { return drivetrain.run(() -> {
        Rotation2d awayFromReef = drivetrain.getClosestReefFace().getOrientation2d();

        double velocityX = speedMetersPerSecond * awayFromReef.getCos();
        double velocityY = speedMetersPerSecond * awayFromReef.getSin();

        drivetrain.fieldOrientedDrive(new ChassisSpeeds(velocityX, velocityY, 0), true);
        Logger.recordOutput("drivetrain/backingAwayFromReef", true);
    }).finallyDo(() -> Logger.recordOutput("drivetrain/backingAwayFromReef", false));}

    public Command scoreCoral(boolean troughScore) {
        Command sendToTrough = Commands.sequence(
            placerGrabber.run(() -> placerGrabber.setFrontRollerVolts(12)).withTimeout(0.5),
            // placerGrabber.setPlacerGrabberVoltsCommand(8, -8).withTimeout(0.5),
            placerGrabber.stopInstantCommand() // <- allows placerGrabber to actually stop when this command
                                               //    is part of a composition, meaning the placerGrabber won't
                                               //    go back to its default command until the entire composition
                                               //    is finished.
        );

        if (troughScore) {
            return sendToTrough;
        }

        Supplier<Command> sendToBranch = () -> {return placerGrabber.run(() -> {
            double volts = 11;
            volts = drivetrain.isFacingReef() ? volts : -volts;
            placerGrabber.setFrontRollerVolts(volts);
        });};

        return Commands.sequence(
            sendToBranch.get().until(() -> !placerGrabber.doesHaveCoral()),
            // sendToBranch.get().withTimeout(0.25), // TODO: maybe too long for auto?
            placerGrabber.stopInstantCommand()
        );
    }

    public Command troughScore() {
        return drivetrain.run(() -> {
            Pose2d facePose = drivetrain.getClosestReefFace().getPose2d();
            Translation2d translationToReef = drivetrain.getPoseMeters().getTranslation().minus(facePose.getTranslation());
            Rotation2d angleToReefCenter = translationToReef.getAngle().minus(facePose.getRotation());

            
            Transform2d positionShift;
            if (placerGrabber.leftHasCoral()) {
                positionShift = new Transform2d(
                    Units.inchesToMeters(17) + Units.inchesToMeters(4.5),
                    Units.inchesToMeters(4),
                    Rotation2d.fromDegrees(-30).plus(Rotation2d.k180deg)
                );
            }
            else {
                positionShift = new Transform2d(
                    Units.inchesToMeters(17) + Units.inchesToMeters(4.5),
                    Units.inchesToMeters(-4),
                    Rotation2d.fromDegrees(30).plus(Rotation2d.k180deg)
                );
            }

            // Rotation2d angleShift = placerGrabber.leftHasCoral() ? Rotation2d.fromDegrees(-30) : Rotation2d.fromDegrees(30);
            // angleShift = angleShift.plus(Rotation2d.k180deg);

            // Transform2d positionShift = new Transform2d(
            //     Units.inchesToMeters(17) + Units.inchesToMeters(4.5+1),
            //     angleToReefCenter.getSin() * translationToReef.getNorm(),
            //     angleShift
            // );

            drivetrain.pidToPose(facePose.plus(positionShift), 1);
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

    private Command processorScore() {
        return arm.shoulder.setTargetAngleCommand(0)
            .alongWith(arm.extension.setTargetLengthCommand(0.77))
            .alongWith(wrist.setTargetPositionCommand(0))
            .alongWith(
                new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() < 10)
                    .andThen(placerGrabber.setPlacerGrabberVoltsCommand(-11, 0)))
            .finallyDo(() -> this.hasAlgae = false);
    }

    private Command scoreOnBarge() {
        AimAtBarge aim = new AimAtBarge(drivetrain, arm, wrist, placerGrabber, duncan::getRequestedFieldOrientedVelocity);
        Command scoreAlgae = placerGrabber.setPlacerGrabberVoltsCommand(-11, 0).withTimeout(0.5);
        return aim.alongWith(new WaitUntilCommand(aim::readyToScore).andThen(scoreAlgae));
    }


    /*** AUTO ***/

    public Command pivotSideAutoCommand(ReefBranch... targetBranches) {
        BooleanSupplier clearOfReef = () -> {
            FieldElement closestFace = drivetrain.getClosestReefFace();
            double distanceFromReef = drivetrain.getPoseMeters().relativeTo(closestFace.getPose2d()).getX();

            double requiredBumperMetersFromReef = Units.inchesToMeters(12);
            double requiredRobotMetersFromReef = requiredBumperMetersFromReef + (DrivetrainConstants.bumperWidthMeters/2.0);

            return distanceFromReef > requiredRobotMetersFromReef;
        };

        SequentialCommandGroup output = new SequentialCommandGroup();
        for (ReefBranch targetBranch : targetBranches) { output.addCommands(
            scoreOnReefCommand(() -> new ChassisSpeeds(), () -> targetBranch, () -> false), // requires everything
            backAwayFromReef(0.75).until(clearOfReef), // requires only drivetrain
            stowArm().until(()->arm.extension.isSafelyRetracted()).andThen(intakeUntilCoralAcquired()).deadlineFor(driveTowardsCoralInAuto()) // intake requires arm,wrist,placerGrabber, drive requires drive
            // intakeUntilCoralAcquired().deadlineFor(driveTowardsCoralInAuto())
            // ^ a bit tricky because the lower value for safe retraction prevents the
            //   shoulder from ever getting to setpoint in intake() unless the arm started
            //   out retracted. Should prob just tune the safe extension point, push it out
            //   a bit further while still staying safe.
        );}
        return output;
    }

    /**
     * Creates an auto command to score on the given target branches and pickup from the lollipops.
     * @param targetBranches - ArrayList of ReefBranches of branches to score on in the order they are provided.
     * @param lollipops - ArrayList of FieldElements of lollipops to pickup from in that order.
     *  If this is shorter than targetBranches, the robot will default to source pickups.
     */
    public Command lollipopAutoCommand(ArrayList<ReefBranch> targetBranches, ArrayList<FieldElement> lollipops, boolean isRightSideAuto) {
        BooleanSupplier clearOfReef = () -> {
            FieldElement closestFace = drivetrain.getClosestReefFace();
            double distanceFromReef = drivetrain.getPoseMeters().relativeTo(closestFace.getPose2d()).getX();

            double requiredBumperMetersFromReef = Units.inchesToMeters(12);
            double requiredRobotMetersFromReef = requiredBumperMetersFromReef + (DrivetrainConstants.bumperWidthMeters/2.0);

            return distanceFromReef > requiredRobotMetersFromReef;
        };

        SequentialCommandGroup output = new SequentialCommandGroup();
        for (int i = 0; i < targetBranches.size(); i++) {

            ReefBranch targetBranch = targetBranches.get(i);

            Command intakeDriveCommand;
            if (i < lollipops.size()) intakeDriveCommand = lollipopPickupInAuto(lollipops.get(i), isRightSideAuto);
            else intakeDriveCommand = driveTowardsCoralInAuto();

            output.addCommands(
                scoreOnReefCommand(() -> new ChassisSpeeds(), () -> targetBranch, () -> false), // requires everything
                backAwayFromReef(0.75).until(clearOfReef), // requires only drivetrain
                stowArm().until(() -> arm.extension.isSafelyRetracted())
                    .andThen(intakeUntilCoralAcquired()).deadlineFor(intakeDriveCommand) // intake requires arm,wrist,placerGrabber, drive requires drive
                // intakeUntilCoralAcquired().deadlineFor(driveTowardsCoralInAuto())
                // ^ a bit tricky because the lower value for safe retraction prevents the
                //   shoulder from ever getting to setpoint in intake() unless the arm started
                //   out retracted. Should prob just tune the safe extension point, push it out
                //   a bit further while still staying safe.
        );}
        return output;
    }

    private Command lollipopPickupInAuto(FieldElement lollipop, boolean isRightSideAuto) { return drivetrain.run(() -> {
        drivetrain.setIntakeToActualSize();

        double maxMetersPerSecond = 1;
        if (this.armInPickupPose()) {
            maxMetersPerSecond = 3;
        }

        Optional<Pose3d> coral = drivetrain.getClosestCoralToEitherIntake();

        //can't see coral, drive towards the lollipop's theoretical position while pointing at it
        if (coral.isEmpty() || !this.armInPickupPose()) {
            return;
        }
        else {
            // can see coral, drive towards it
            // Pose2d pickupPose = drivetrain.getOffsetCoralPickupPose(coral.get());
            Pose2d pickupPose = drivetrain.getLollipopPickupPose(coral.get());
            drivetrain.pidToPose(pickupPose, 0.8);
        }


        //Adjust lollipop pose in order to always pickup with left side
        // Translation2d adjustedLollipopTranslation = lollipop.getPose2d().getTranslation();
        // Translation2d adjustment = new Translation2d(0.5, 0);//ArmConstants.wristOuterWidthMeters*2.);
        // if (DriverStation.getAlliance().get() == Alliance.Red) adjustedLollipopTranslation.plus(adjustment);
        // else adjustedLollipopTranslation.minus(adjustment);

        // Translation2d translationToLollipop = adjustedLollipopTranslation.minus(drivetrain.getPoseMeters().getTranslation());
        // Rotation2d angleToPointIn = translationToLollipop.getAngle();

        // if (translationToLollipop.getNorm() < DrivetrainConstants.bumperWidthMeters + 0.2) {
        //     maxMetersPerSecond = 0.5;
        // }

        // //5 for first pickup
        // //-28 for second pickup
        // if (lollipop.equals(FieldElement.RIGHT_LOLLIPOP) || lollipop.equals(FieldElement.LEFT_LOLLIPOP)) {
        //     angleToPointIn = Rotation2d.fromDegrees(5);
        // }
        // else if (lollipop.equals(FieldElement.MIDDLE_LOLLIPOP)) {
        //     angleToPointIn = Rotation2d.fromDegrees(-28);
        // }

        // if (isRightSideAuto) angleToPointIn.times(-1);
        // if (DriverStation.getAlliance().get() == Alliance.Blue) angleToPointIn.plus(Rotation2d.k180deg);
        
        // if (translationToLollipop.getNorm() < 0.2) {
        //     angleToPointIn = Rotation2d.fromDegrees(-90);
        // }

        // Pose2d desiredPose = new Pose2d(adjustedLollipopTranslation, angleToPointIn);

        // drivetrain.pidToPose(desiredPose, maxMetersPerSecond);
            

    }).finallyDo(drivetrain::resetCenterOfRotation);}

    private Command driveTowardsCoralInAuto() { return drivetrain.run(() -> {
        drivetrain.setIntakeToWideSize();
        Optional<Pose3d> coral = drivetrain.getClosestCoralToEitherIntake();
        double maxMetersPerSecond = 1;
        if (this.armInPickupPose()) {
            maxMetersPerSecond = 2.1;
        }

        if (coral.isEmpty() || !this.armInPickupPose()) {
            // can't see coral, just drive to source
            FieldElement sourceSide = drivetrain.getClosestLoadingStation();

            // go to closest slot to the reef
            double offsetY_loadingStationFrame = 4*FieldConstants.centerToCenterMetersBetweenLoadingStationSlots;
            if (sourceSide == FieldElement.RIGHT_LOADING_STATION) {
                offsetY_loadingStationFrame *= -1;
            }

            // far enough away from the loading station to see dropped coral in our fov
            double metersFromLoadingStation = 2.0 * DrivetrainConstants.bumperWidthMeters;

            // generate the target in field coords
            Transform2d pickupLocationRelativeToSource = new Transform2d(metersFromLoadingStation, offsetY_loadingStationFrame, Rotation2d.k180deg);
            Pose2d targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
            drivetrain.pidToPose(targetRobotPose2d, maxMetersPerSecond);
        } else {
            // can see coral, drive towards it
            // Pose2d pickupPose = drivetrain.getOffsetCoralPickupPose(coral.get());
            Pose2d pickupPose = drivetrain.getStrafingPickupPose(coral.get());
            drivetrain.pidToPose(pickupPose, 1.35);
        }
    }).finallyDo(drivetrain::resetCenterOfRotation);}
    

    public Command autoChoosingAuto() {

        Command leftSideAuto = pivotSideAutoCommand(
            ReefBranch.BRANCH_J4,
            ReefBranch.BRANCH_K4,
            ReefBranch.BRANCH_L4,
            ReefBranch.BRANCH_A4
        );

        Command rightSideAuto = pivotSideAutoCommand(
            ReefBranch.BRANCH_E4,
            ReefBranch.BRANCH_D4,
            ReefBranch.BRANCH_C4,
            ReefBranch.BRANCH_B4
        );

        Command leftSideLollipopAuto = lollipopAutoCommand(
            new ArrayList<>(Arrays.asList(
                ReefBranch.BRANCH_J4,
                ReefBranch.BRANCH_L4,
                ReefBranch.BRANCH_A4,
                ReefBranch.BRANCH_K4
            )),
            new ArrayList<>(Arrays.asList(
                FieldElement.LEFT_LOLLIPOP,
                FieldElement.MIDDLE_LOLLIPOP
            )),
            false
        );

        Command rightSideLollipopAuto = lollipopAutoCommand(
            new ArrayList<>(Arrays.asList(
                ReefBranch.BRANCH_E4,
                ReefBranch.BRANCH_C4,
                ReefBranch.BRANCH_B4,
                ReefBranch.BRANCH_D4
            )),
            new ArrayList<>(Arrays.asList(
                FieldElement.RIGHT_LOLLIPOP,
                FieldElement.MIDDLE_LOLLIPOP
            ))  ,
            true  
        );

        BooleanSupplier startingOnLeft = () -> {return drivetrain.getClosestLoadingStation() == FieldElement.LEFT_LOADING_STATION;};

        return new ConditionalCommand(leftSideAuto, rightSideAuto, startingOnLeft);
        // return new ConditionalCommand(leftSideLollipopAuto, rightSideLollipopAuto, startingOnLeft);
    }

    private Command homeArmWhileGoingToSource() { return stowArm().alongWith(drivetrain.run(() -> {
        Transform2d poseAdjustment = new Transform2d(2, 0, new Rotation2d());
        Translation2d desiredTranslation = drivetrain.getClosestLoadingStation().getPose2d().plus(poseAdjustment).getTranslation();

        Rotation2d desiredRotation = FlyingCircuitUtils.getAllianceDependentValue(Rotation2d.k180deg, Rotation2d.kZero, Rotation2d.kZero);
        drivetrain.pidToPose(new Pose2d(desiredTranslation, desiredRotation), 3.5);
    })).until(() -> arm.getShoulderAngleDegrees() < 40);}
}
