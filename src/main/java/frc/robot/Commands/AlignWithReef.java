package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Reefscape.FieldElement;
import frc.robot.Reefscape.FieldElement.ReefFace;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignWithReef extends Command{
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private FieldElement target;\
    private int stalk;


    private final FieldElement[] reefFaces = {ReefFace.FRONT, ReefFace.FRONT_LEFT, ReefFace.FRONT_RIGHT, ReefFace.BACK, ReefFace.BACK_LEFT, ReefFace.BACK_RIGHT};

    /**
     *  @param translationController
     */

    public FieldElement.ReefFace getClosestReefFace() {

        double closestDistance = 100000000000000.00;
        FieldElement closestReefFace = reefFaces[0];
        for(int i=1; i<5; i++) {
            // distance formula HAVE NOT TESTED
            double distance = Math.sqrt((Math.pow((drivetrain.getPoseMeters().getX() - reefFaces[i].getPose2d().getX()), 2) 
            + Math.pow((drivetrain.getPoseMeters().getY() - reefFaces[i].getPose2d().getY()), 2)));

            if (distance < closestDistance) {
                closestReefFace = reefFaces[i];
            }
        }

        return closestReefFace;
    }

    public AlignWithReef(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, int stalk, int branch) {
    
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.target = target;
        .stalk=stalk;

        if (translationController != null) {
            super.addRequirements(drivetrain);
        }
    }

    @Override
    public void execute() {
        FieldElement.ReefFace closestFace = getClosestReefFace();
        Pose2d targetPose = closestFace.stalks[stalk].branches[branch];

        Rotation2d adjustedRotation = targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
    }
}
