package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;

public class Autos extends RobotContainer{
    
    private String typeOfIntake = "Ground";
    

    private boolean readyToIntake = false;

    private Command intakeTowardsCoralInAuto() {
        return drivetrain.run(() -> {
            if (drivetrain.getBestCoralLocation().isEmpty()) {
                FieldElement sourceSide = drivetrain.getClosestSourceSide();
                Transform2d pickupLocationRelativeToSource = new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.fromDegrees(180));
                Pose2d targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
                drivetrain.pidToPose(targetRobotPose2d);
            } else {
                drivetrain.driveTowardsCoral(drivetrain.getBestCoralLocation().get());
            }
        });
    }

    private Command doesSeeCoralCommand() {
        return new WaitUntilCommand(
            () -> {return drivetrain.doesSeeCoral();})
            .finallyDo(() -> {
                if (drivetrain.doesSeeCoral()) {
                    typeOfIntake = "Ground";
                }
                else typeOfIntake = "Source";
            });
        }

    private Command intake() {
        return new ConditionalCommand(
            intakeTowardsCoralInAuto(),
            sourceIntakeCommand(),
            () -> {return typeOfIntake == "Ground";});
    }

    private Command sourceIntakeIfDoesntHaveCoral() {
        return sourceIntakeCommand().onlyIf(placerGrabber::doesHaveCoral);
    }
    
    private Command waitUntilInRangeOfSource() {
        return new WaitUntilCommand(() -> {
            Translation2d robotTranslation = drivetrain.getPoseMeters().getTranslation();
            return robotTranslation.getDistance(drivetrain.getClosestSourceSide().getLocation2d()) < 1;
        });
    }

    public Command rightSideAuto() {

        return new SequentialCommandGroup(
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_E4),
            intake().withDeadline(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_D4),
            intake().withDeadline(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_C4),
            intake().withDeadline(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_B4)
        );

    }

    public Command leftSideAuto() {

        return new SequentialCommandGroup(
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_J4),
            intake().withDeadline(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_K4),
            intake().withDeadline(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_L4),
            intake().withDeadline(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_A4)
        );

    }
}
