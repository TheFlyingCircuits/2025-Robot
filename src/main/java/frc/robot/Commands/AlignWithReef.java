package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignWithReef extends Command{
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private int stalk;
    private int branch;


    /**
     *  @param translationController
     */

    public ReefFace getClosestReefFace() {
        ReefFace[] reefFaces = FieldElement.ALL_REEF_FACES;

        double closestDistance = 100000000000000.00;
        ReefFace closestReefFace = reefFaces[0];
        for(int i=0; i<5; i++) {
            // distance formula HAVE NOT TESTED
            double distance = Math.sqrt((Math.pow((drivetrain.getPoseMeters().getX() - reefFaces[i].getPose2d().getX()), 2) 
            + Math.pow((drivetrain.getPoseMeters().getY() - reefFaces[i].getPose2d().getY()), 2)));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestReefFace = reefFaces[i];
                System.out.println(distance);
            }
        }

        return closestReefFace;
    }

    public AlignWithReef(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, int stalk, int branch) {
    
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.stalk=stalk;
        this.branch=branch;

        if (translationController != null) {
            super.addRequirements(drivetrain);
        }
    }

    @Override
    public void execute() {
        ReefFace face = getClosestReefFace();
        Pose2d targetPose = face.getStalks()[stalk].getBranch(branch).getPose2d();

        Rotation2d adjustedRotation = targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
    }
}
