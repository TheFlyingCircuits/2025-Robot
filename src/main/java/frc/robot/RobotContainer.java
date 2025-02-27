// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.ScoreOnReef;
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
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIONeo;


public class RobotContainer {

    protected final HumanDriver charlie = new HumanDriver(0);
    protected final HumanDriver ben = new HumanDriver(1);

    public final Drivetrain drivetrain;
    private ArmIOKraken armIO;
    public final Arm arm;
    public final Wrist wrist;
    public final Leds leds;
    public final PlacerGrabber placerGrabber;

    private DigitalInput coastModeButton;

    private boolean isFacingForward = true;
    private String sideCoralIsIntaked = "left";


    
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
            wrist = new Wrist(new WristIONeo(placerGrabberIO.getLeftThroughboreEncoder()));

            leds = new Leds();

            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625
            
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(0, 1, -0.377686, 0, "FL"), 
                new SwerveModuleIOKraken(2, 3, 0.397705, 1, "FR"),
                new SwerveModuleIOKraken(4, 5, 0.238281, 2, "BL"),
                new SwerveModuleIOKraken(6, 7,  -0.370850, 3, "BR"),
                new VisionIO(){}
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
        arm.setDefaultCommand(arm.setShoulderTargetAngleCommand(0));
        wrist.setDefaultCommand(wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees));
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


        controller.a().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.minExtensionMeters+0.05));
        controller.b().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.minExtensionMeters+0.3));
        controller.x().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.maxExtensionMeters-0.3));
        controller.y().onTrue(arm.setExtensionTargetLengthCommand(ArmConstants.maxExtensionMeters-0.05));


        controller.rightTrigger().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(9, 6).until(placerGrabber::doesHaveCoral));
        controller.rightBumper().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(-6, 0));
        controller.leftBumper().whileTrue(placerGrabber.setPlacerGrabberVoltsCommand(6, 0));
    }

    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        controller.y().onTrue(new InstantCommand(drivetrain::setRobotFacingForward));



        controller.rightBumper().whileTrue(
            new ScoreOnReef(
                drivetrain,
                arm,
                wrist,
                charlie::getRequestedFieldOrientedVelocity,
                () -> {return drivetrain.getClosestReefStalk().getBranch(3);},
                leds,
                () -> placerGrabber.sideCoralIsIn(),
                () -> isFacingReef()
            )
        );

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



        Trigger coastModeLimitSwitch = new Trigger(() -> coastModeButton.get());
        coastModeLimitSwitch.toggleOnTrue( //toggles between coast mode and brake mode
            Commands.run(() -> {
                armIO.setIdleMode(NeutralModeValue.Coast);
            }).finallyDo(() -> {
                System.out.println("set to brake mode");
                armIO.setIdleMode(NeutralModeValue.Brake);
            }).ignoringDisable(true)
        );
    }

    /** Called by Robot.java, brief convenience function. */
    public void periodic() {
        Logger.recordOutput("coastModeLimitSwitch", coastModeButton.get());
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

    public Command scoreOnReefCommand(Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch) {
        return new ScoreOnReef(drivetrain, arm, wrist, translationController, reefBranch, leds, () -> placerGrabber.sideCoralIsIn(), () -> isFacingReef());
    }

    private Command intake() {
        return arm.setShoulderTargetAngleCommand(0)
            .alongWith(
                wrist.setTargetPositionCommand(0))
            .alongWith(
                setPlacerGrabberVolts(3, 3));
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

    public Command sourceIntakeInAuto() { // need to do
        return null;
    }
    
    public Command intakeTowardsCoralInAuto() {
        return drivetrain.run(() -> {
            drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
        });
    }

}
