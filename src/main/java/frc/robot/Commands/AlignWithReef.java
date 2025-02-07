package frc.robot.Commands;

import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Reefscape.FieldConstants;
import frc.robot.Reefscape.FieldElement.ReefFace;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignWithReef extends Command{
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private int stalk;
    private int branch;
    private ReefFace reefFace;


    /**
     *  @param translationController
     */
// bumper 37 inches
    public Pose2d adjustedReefScoringPose(Pose2d stalkPose) {
        double adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(37);
        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, 0.0,new Rotation2d());
        stalkPose.plus(targetPoseToRobotRelativeToStalk);
        return stalkPose;
    }

    public AlignWithReef(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, int stalk, int branch, ReefFace reefFace ) {
    
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.stalk=stalk;
        this.branch=branch;
        this.reefFace=reefFace;

        if (translationController != null) {
            super.addRequirements(drivetrain);
        }
    }

    @Override
    public void execute() {
        Pose2d targetPose = reefFace.stalks[stalk].branches[branch].getPose2d();
        targetPose = adjustedReefScoringPose(targetPose);
        Rotation2d adjustedRotation = targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        drivetrain.beeLineToPose(new Pose2d(targetPose.getTranslation(), adjustedRotation));
    }
}
