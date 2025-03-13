package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;

public class Vision {

    private TagCamera[] tagCameras;
    private ColorCamera intakeCamera;
    private Optional<Translation3d> closestCoral_fieldFrame = Optional.empty();



    public Vision() {
        tagCameras = new TagCamera[] {
            new TagCamera("intakeSideTagCam", new Transform3d()),
            new TagCamera("pivotSideTagCam", new Transform3d()),
        };

        intakeCamera = new ColorCamera("intakeCam", new Transform3d());
    }


    // private Optional<Translation3d> findClosestValidCoral(SwerveDrivePoseEstimator fusedPoseEstimator) {
    //     // keep track of which gamepieces are accepted/rejected for logging/debugging
    //     List<Translation3d> validCorals = new ArrayList<>();
    //     List<Translation3d> invalidCorals = new ArrayList<>();
    //     Optional<Translation3d> closestValidCoral = Optional.empty();
    //     double metersToClosestCoral = -1;

    //     // find where the robot was when the camera took the picture
    //     Optional<Pose2d> latencyCompensatedPose = fusedPoseEstimator.sampleAt(intakeCamera.getMostRecentFrame().timestampSeconds);
    //     Pose3d robotPose = latencyCompensatedPose.isPresent() ? new Pose3d(latencyCompensatedPose.get()) : new Pose3d(fusedPoseEstimator.getEstimatedPosition());

    //     // iterate though each coral that was seen in the picture to find the one that was closest to the robot.
    //     for (Translation3d coralLocation_robotFrame : intakeCamera.getMostRecentFrame().gamepieceLocations_robotCoords) {
    //         // find the location of the coral on the field
    //         Translation3d coralLocation_fieldFrame = coralLocation_robotFrame.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

    //         // don't track coral that's outside the field perimeter
    //         boolean inField = (0 <= coralLocation_fieldFrame.getX()) && (coralLocation_fieldFrame.getX() <= FieldConstants.maxX)
    //                        && (0 <= coralLocation_fieldFrame.getY()) && (coralLocation_fieldFrame.getY() <= FieldConstants.maxY);
    //         if (!inField) {
    //             invalidCorals.add(coralLocation_fieldFrame);
    //             continue;
    //         }

    //         // don't track coral that's too far away from the robot
    //         double maxMetersFromRobot = 2.5;
    //         double metersFromRobot = coralLocation_robotFrame.toTranslation2d().getNorm();
    //         if (metersFromRobot > maxMetersFromRobot) {
    //             invalidCorals.add(coralLocation_fieldFrame);
    //             continue;
    //         }
                      
    //         // see if this coral is closer than our current closest
    //         validCorals.add(coralLocation_fieldFrame);
    //         if (closestValidCoral.isEmpty() || (metersFromRobot < metersToClosestCoral)) {
    //             closestValidCoral = Optional.of(coralLocation_fieldFrame);
    //             metersToClosestCoral = metersFromRobot;
    //         }
    //     }

    //     Logger.recordOutput("Vision/"+intakeCamera.name+"/validCoralsFieldFrame", validCorals.toArray(new Translation3d[0]));
    //     Logger.recordOutput("Vision/"+intakeCamera.name+"/invalidCoralsFieldFrame", invalidCorals.toArray(new Translation3d[0]));
    //     return closestValidCoral;
    // }

    public Optional<Translation3d> getClosestCoralFieldCoords() {
        return closestCoral_fieldFrame;
    }

    public void periodic(SwerveDrivePoseEstimator fusedPoseEstimator, SwerveDrivePoseEstimator wheelsOnlyPoseEstimator) {
        // intakeCamera.periodic();
        // closestCoral_fieldFrame = this.findClosestCoral(fusedPoseEstimator);
    }




















    public double mostRecentSighting(int tagID) {
        // double timestampSeconds = -1;
        // for (TagCamera cam : tagCameras) {
        //     if (cam.mostRecentSighting(tagID) > timestampSeconds) {
        //         timestampSeconds = cam.mostRecentSighting(tagID);
        //     }
        // }
        // return timestampSeconds;
        return 0;
    }
    public double mostRecentSighting(FieldElement fieldElement) {
        return mostRecentSighting(fieldElement.getTagID());
    }

    public boolean hasSeenRecently(int tagID, double howRecentlySeconds) {
        return (Timer.getTimestamp() - mostRecentSighting(tagID)) <= howRecentlySeconds;
    }
    public boolean hasSeenRecently(FieldElement fieldElement, double howRecentlySeconds) {
        return hasSeenRecently(fieldElement.getTagID(), howRecentlySeconds);
    }

    public void periodic() {
        
    }
    
}
