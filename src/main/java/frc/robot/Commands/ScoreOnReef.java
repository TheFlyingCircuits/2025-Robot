package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.wrist.Wrist;

public class ScoreOnReef extends Command {

    private Drivetrain drivetrain;
    private Arm arm;
    private Wrist wrist;
    private Supplier<ChassisSpeeds> translationController;
    private Supplier<ReefBranch> reefBranch;
    private Leds leds;

    /**
     *  @param translationController - ChassisSpeeds supplier for driver input while scoring.
     */
    public ScoreOnReef(Drivetrain drivetrain, Arm arm, Wrist wrist, Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Leds leds) {
    
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.wrist = wrist;
        this.translationController = translationController;
        this.reefBranch=reefBranch;
        this.leds = leds;
        addRequirements(drivetrain, arm, wrist);
    }

    // bumper 37 inches
    private Pose2d adjustedReefScoringPose(Pose2d stalkPose) {
        double adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(22);
        //TODO: adjust based on left/right intake in the grabby
        //TODO: make this potentially a pivot-side score
        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, 0.0,new Rotation2d());
        return  stalkPose.plus(targetPoseToRobotRelativeToStalk);
    }

    /**
     * Generates an ArmPosition object representing the position of the arm to score on the desired branch.
     * This is calculated based on the position of the robot relative to the branch.
     */
    private ArmPosition calculateArmScoringPosition() {
        Translation2d robotTranslation = drivetrain.getPoseMeters().getTranslation();
        Translation3d branchTranslation = reefBranch.get().getLocation();

        Translation2d horizontalTranslation = robotTranslation.minus(branchTranslation.toTranslation2d());
        double horizontalExtensionMeters = horizontalTranslation.getNorm();

        double verticalExtensionMeters = branchTranslation.getZ();

        horizontalExtensionMeters -= 0.1; //TODO: tweak these
        verticalExtensionMeters += 0.2;

        double targetWristAngleDegrees = 0;
        
        switch (reefBranch.get().getLevel()) { //TODO: fill in with actual values and handle pivot-side scoring cases
            case 1:
                targetWristAngleDegrees = 40;
                break;
            case 2:
                targetWristAngleDegrees = 40;
                break;
            case 3:
                targetWristAngleDegrees = 40;
                break;
            case 4:
                targetWristAngleDegrees = 40;
                break;
        }

        return ArmPosition.generateArmPosition(
            new Pose2d(
                horizontalExtensionMeters,
                verticalExtensionMeters,
                Rotation2d.fromDegrees(targetWristAngleDegrees)
            )
        );

    }

    @Override
    public void execute() {
        Pose2d targetPose;
        targetPose = adjustedReefScoringPose(reefBranch.get().getStalk().getPose2d());
        Rotation2d adjustedRotation = targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        drivetrain.pidToPose(new Pose2d(targetPose.getTranslation(), adjustedRotation));

        ArmPosition desiredArmPosition = calculateArmScoringPosition();
        arm.setArmPosition(desiredArmPosition);
        wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees);

        leds.progressBar(arm.getExtensionMeters() / desiredArmPosition.extensionMeters);
    }
}
