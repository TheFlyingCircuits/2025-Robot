package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Reefscape.FieldConstants;
import frc.robot.Reefscape.FieldElement.ReefBranch;
import frc.robot.Reefscape.FieldElement.ReefFace;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignWithReef extends Command{
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private int stalk;
    private Supplier<ReefBranch> reefBranch;
    private Supplier<ReefFace> reefFace;


    /**
     *  @param translationController
     */
// bumper 37 inches
    public Pose2d adjustedReefScoringPose(Pose2d stalkPose) {
        double adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(22);
        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, 0.0,new Rotation2d());
        return  stalkPose.plus(targetPoseToRobotRelativeToStalk);
    }

    public AlignWithReef(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch) {
    
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.reefBranch=reefBranch;

        if (translationController != null) {
            super.addRequirements(drivetrain);
        }
    }

    @Override
    public void execute() {
        Pose2d targetPose;
        targetPose = adjustedReefScoringPose(reefBranch.get().stalk.getPose2d());
        Rotation2d adjustedRotation = targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        drivetrain.pidToPose(new Pose2d(targetPose.getTranslation(), adjustedRotation));
    }
}
