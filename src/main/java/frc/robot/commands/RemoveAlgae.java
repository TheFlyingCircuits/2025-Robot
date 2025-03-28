package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.wrist.Wrist;

public class RemoveAlgae extends ParallelRaceGroup {

    private Drivetrain drivetrain;
    private Arm arm;
    private Wrist wrist;

    public RemoveAlgae(Drivetrain drivetrain, Arm arm, Wrist wrist, boolean isHighAlgae) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.wrist = wrist;

        super.addCommands(
            driveToReef(), // never ends
            wrist.setTargetPositionCommand(WristConstants.maxAngleDegrees-5), // never ends
            new SequentialCommandGroup(
                waitForDriveToBeCloseToReef(),
                aimArmAtAlgae(isHighAlgae),
                punch(isHighAlgae),
                swingUp() // ends eventually, ending the race group
            )
        );
    }

    private Command driveToReef() { return drivetrain.run(() -> {
        Pose2d reefFacePose = drivetrain.getClosestReefFace().getPose2d();

        double oneCoralAwayFromFace = (DrivetrainConstants.bumperWidthMeters / 2.0) + FieldConstants.coralOuterDiameterMeters;
        Transform2d adjustmentRelativeToFace = new Transform2d(oneCoralAwayFromFace, 0, Rotation2d.k180deg);

        drivetrain.pidToPose(reefFacePose.plus(adjustmentRelativeToFace), 2);
    });}

    private Command waitForDriveToBeCloseToReef() { return new WaitUntilCommand(() -> {
        boolean translationGood = Math.abs(drivetrain.getTranslationError()) < Units.inchesToMeters(2);
        boolean rotationGood = Math.abs(drivetrain.getAngleError()) < 2.0; // degrees
        return translationGood && rotationGood;
    });}

    private Command aimArmAtAlgae(boolean isHighAlgae) {
        double targetDegrees = isHighAlgae ? 51 : 35;
        BooleanSupplier aimComplete = () -> {return Math.abs(arm.getShoulderAngleDegrees()-targetDegrees) < 5;};
        return arm.shoulder.setTargetAngleCommand(targetDegrees).until(aimComplete);
    }

    private Command punch(boolean isHighAlgae) {
        double targetLength = isHighAlgae ? 1.27 : 0.90;
        BooleanSupplier punchComplete = () -> {return Math.abs(arm.getExtensionMeters()-targetLength) < 0.01;};
        return arm.extension.setTargetLengthCommand(targetLength).until(punchComplete);
    }

    private Command swingUp() {
        double swingDegrees = 73;
        BooleanSupplier swingComplete = () -> {return Math.abs(arm.getShoulderAngleDegrees()-swingDegrees) < 5;};
        return arm.shoulder.setTargetAngleCommand(73).until(swingComplete);
    }
}