package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.commands.SourceIntake;

public class Autos extends RobotContainer{
    

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
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_C4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_B4, () -> true)
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
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_L4, () -> true),
            intakeTowardsCoralInAuto().raceWith(waitUntilInRangeOfSource()),
            sourceIntakeIfDoesntHaveCoral(),
            scoreOnReefCommand(duncan::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_A4, () -> true)
        );

    }
}
