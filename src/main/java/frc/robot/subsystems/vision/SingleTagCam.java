package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.PlayingField.FieldConstants;

// TODO: add quick camera placement detector by putting robot at known pose
// TODO: add demo tag stuff.
public class SingleTagCam {

    private final PhotonCamera cam;
    private Translation3d camLocation_robotFrame;
    private Rotation3d camOrientation_robotFrame;


    public SingleTagCam(String name, Translation3d camLocation_robotFrame, Rotation3d camOrientation_robotFrame) {
        cam = new PhotonCamera(name);
        this.camLocation_robotFrame = camLocation_robotFrame;
        this.camOrientation_robotFrame = camOrientation_robotFrame;

        // TODO: customize simSettings instead of using a preset
        //       (not strictly necessary, but could be nice).
        SimCameraProperties simSettings = SimCameraProperties.PI4_LIFECAM_640_480();
        PhotonCameraSim simCamera = new PhotonCameraSim(cam, simSettings);

        // minTargetAreaPixels was set based on the smallest apriltag we should probably detect.
        // IRL, the tags are squares made up of 100 tiles (36 data bit tiles, 28 tiles of black border
        // around the data bits, and then 36 tiles of white border around the black border).
        // Each individual tile on an apriltag should probably land on at least a 3x3 block
        // of pixels in the image plane to be detected reliably.
        int apriltagTiles = 100;
        int pixelsPerTile = 3 * 3;
        simCamera.setMinTargetAreaPixels(apriltagTiles * pixelsPerTile);
        FieldConstants.simulatedTagLayout.addCamera(simCamera, new Transform3d(camLocation_robotFrame, camOrientation_robotFrame));
    }
    public SingleTagCam(String name, Pose3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }
    public SingleTagCam(String name, Transform3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }
    
    
    public List<SingleTagPoseObservation> getFreshPoseObservations() {
        List<SingleTagPoseObservation> output = new ArrayList<>();

        // See if we've gotten any new frames since last time
        List<PhotonPipelineResult> freshFrames = cam.getAllUnreadResults();
        if (freshFrames.size() == 0) {
            return output;
        }

        for (PhotonPipelineResult frame : freshFrames) {
            for (PhotonTrackedTarget tag : frame.targets) {
                Pose3d robotPose = this.getRobotPoseFromSingleTag(tag);
                double timestamp = frame.getTimestampSeconds();
                int tagID = tag.fiducialId;
                double tagToCamDistance = tag.getBestCameraToTarget().getTranslation().getNorm();
                double ambiguity = tag.poseAmbiguity;

                SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(cam.getName(), robotPose, timestamp, tagID, tagToCamDistance, ambiguity);
                output.add(poseObservation);
            }
        }

        Logger.recordOutput("tagCams/"+cam.getName()+"/singleTagPoseObservations", output.toArray(new SingleTagPoseObservation[0]));
        return output;
    }

    public String getName() {
        return cam.getName();
    }


    private Pose3d getRobotPoseFromSingleTag(PhotonTrackedTarget singleTag) {

        Transform3d camPose_robotFrame = new Transform3d(camLocation_robotFrame, camOrientation_robotFrame);
        Transform3d robotPose_camFrame = camPose_robotFrame.inverse();


        Transform3d tagPose_fieldFrame = FieldConstants.tagPoseAsTransform(singleTag.fiducialId);
        Transform3d camPose_tagFrame = singleTag.getBestCameraToTarget().inverse();
        Transform3d camPose_fieldFrame = tagPose_fieldFrame.plus(camPose_tagFrame);
        Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);

        return new Pose3d(robotPose_fieldFrame.getTranslation(), robotPose_fieldFrame.getRotation());
    }
}

