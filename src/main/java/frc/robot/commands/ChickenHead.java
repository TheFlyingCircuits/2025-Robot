package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

public class ChickenHead extends Command {

    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private Arm arm;
    private Wrist wrist;
    private PlacerGrabber placerGrabber;
    private Supplier<Pose3d> desiredCoralPose_fieldFrame;

    public ChickenHead(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, Arm arm, Wrist wrist, PlacerGrabber intakeWheels, Supplier<Pose3d> desiredCoralPose_fieldFrame) {
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.arm = arm;
        this.wrist = wrist;
        this.placerGrabber = intakeWheels;
        this.desiredCoralPose_fieldFrame = desiredCoralPose_fieldFrame;
        if (desiredCoralPose_fieldFrame == null) { this.desiredCoralPose_fieldFrame = () -> {
            return FieldElement.BRANCH_A2.getPose().transformBy(new Transform3d(FieldConstants.coralLengthMeters/2.0, 0, 0, new Rotation3d(Math.PI, 0, 0)));
        };}

        // intakeWheels aren't added to requirements because we never write to them,
        // we only read from them.
        // super.addRequirements(drivetrain, arm.shoulder, arm.extension, wrist);
        super.addRequirements(drivetrain);
        super.setName("ChickenHeadCommand");
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Logger.recordOutput("chickenHead/targteCoralPose_fieldFrame", new Pose3d[] {this.desiredCoralPose_fieldFrame.get()});
        this.aimDriveAtReef();
    }

    @Override
    public void end(boolean wasInterrupted) {
        // clear advantage scope viz
        Logger.recordOutput("chickenHead/driveAlignmentLine", new Pose2d[0]);
        Logger.recordOutput("chickenHead/lineThroughLeftCoral", new Pose2d[0]);
        Logger.recordOutput("chickenHead/lineThroughRightCoral", new Pose2d[0]);
        Logger.recordOutput("chickenHead/targetWristPose_fieldFrame", new Pose3d[0]);
        Logger.recordOutput("chickenHead/targteCoralPose_fieldFrame", new Pose3d[0]);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void aimDriveAtReef() {
        // maybe just draw line from held coral opening to desired coral opening? Just like intaking?
        // need to be mindful of the undefined angle case (desiredLocation == measuredLocation)
        // It's not possible in the intake version because the desiredLocation is on the bumper
        // and our coral filtering makes it so that coral can't be detected on the bumper (its less than the min distance).
        Translation2d targetWristLocation = this.getTargetWristPose_fieldFrame().getTranslation().toTranslation2d();
        Translation2d vectorFromDriveToWrist = targetWristLocation.minus(drivetrain.getPoseMeters().getTranslation());
        drivetrain.fieldOrientedDriveWhileAiming(translationController.get(), vectorFromDriveToWrist.getAngle());

        Pose2d alignmentLine = new Pose2d(targetWristLocation, vectorFromDriveToWrist.getAngle());
        Logger.recordOutput("chickenHead/driveAlignmentLine", new Pose2d[] {
            alignmentLine.plus(new Transform2d(10, 0, Rotation2d.kZero)),
            alignmentLine.plus(new Transform2d(-10, 0, Rotation2d.kZero))
        });

        Transform2d lineThroughLeftCoral_driveFrame = new Transform2d(0, ArmConstants.intakeToLeftCoral.getY(), Rotation2d.kZero);
        Transform2d lineThroughRightCoral_driveFrame = new Transform2d(0, ArmConstants.intakeToRightCoral.getY(), Rotation2d.kZero);

        Pose2d lineThroughLeftCoral_fieldFrame = drivetrain.getPoseMeters().plus(lineThroughLeftCoral_driveFrame);
        Pose2d lineThroughRightCoral_fieldFrame = drivetrain.getPoseMeters().plus(lineThroughRightCoral_driveFrame);

        Logger.recordOutput("chickenHead/lineThroughLeftCoral", new Pose2d[] {
            lineThroughLeftCoral_fieldFrame.plus(new Transform2d(10, 0, Rotation2d.kZero)),
            lineThroughLeftCoral_fieldFrame.plus(new Transform2d(-10, 0, Rotation2d.kZero))
        });

        Logger.recordOutput("chickenHead/lineThroughRightCoral", new Pose2d[] {
            lineThroughRightCoral_fieldFrame.plus(new Transform2d(10, 0, Rotation2d.kZero)),
            lineThroughRightCoral_fieldFrame.plus(new Transform2d(-10, 0, Rotation2d.kZero))
        });
    }

    private Pose3d getTargetWristPose_fieldFrame() {
        Translation3d coralLocation_wristFrame = ArmConstants.intakeToLeftCoral;
        if (placerGrabber.rightHasCoral()) {
            coralLocation_wristFrame = ArmConstants.intakeToRightCoral;
        }

        Transform3d coralPose_wristFrame = new Transform3d(coralLocation_wristFrame, Rotation3d.kZero);
        Transform3d wristPose_coralFrame = coralPose_wristFrame.inverse();

        Pose3d wristPose_fieldFrame = desiredCoralPose_fieldFrame.get().plus(wristPose_coralFrame);
        Logger.recordOutput("chickenHead/targetWristPose_fieldFrame", new Pose3d[] {wristPose_fieldFrame});
        return wristPose_fieldFrame;
    }
}
