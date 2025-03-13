package frc.robot.subsystems.vision;

import java.lang.StackWalker.Option;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.VendorWrappers.FotonCamera;

// TODO: add quick camera placement detector by putting robot at known pose
// TODO: add demo tag stuff.
public class TagCamera {

    private final FotonCamera cam;
    private Translation3d camLocation_robotFrame;
    private Rotation3d camOrientation_robotFrame;

    // "Sensor" readings
    // private PoseObservation cachedFrame;
    // private boolean hasNewData = false;
    private Optional<PoseObservation> mostRecentPoseObservation = Optional.empty();
    private double[] mostRecentSightings = new double[FieldConstants.tagLayout.getTags().size()];
    
    // private Optional<FieldElement> focus = Optional.empty();


    public TagCamera(String name, Translation3d camLocation_robotFrame, Rotation3d camOrientation_robotFrame) {
        cam = new FotonCamera(name);
        this.camLocation_robotFrame = camLocation_robotFrame;
        this.camOrientation_robotFrame = camOrientation_robotFrame;
        Arrays.fill(mostRecentSightings, -1); // Use the convention that unseen tags correspond to a timestamp of -1

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
    public TagCamera(String name, Pose3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }
    public TagCamera(String name, Transform3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }


    public void periodic() {
        Logger.processInputs("TagCams/"+cam.getName(), cam);

        PhotonPipelineResult mostRecentFrame = cam.getLatestResult();

        if (!mostRecentFrame.hasTargets()) {
            // early exit
        }
    }
    
    
    public Optional<PoseObservation> getFreshPoseEstimate() {
        // See if we've gotten any new frames since last time
        List<PhotonPipelineResult> freshFrames = cam.getAllUnreadResults();
        if (freshFrames.size() == 0) {
            // stay with last logged in advantage scope to avoid flickering
            // Logger.processInputs(cam.getName()+"/mostRecentPoseEstimate", new PoseObservation());
            return Optional.empty();
        }

        // If our coprocessor is running very fast, then PhotonVision may have generated
        // multiple results since the last robot loop. However, we only bother with processing
        // the most recent result due to limitations with wpilib's current structured logging support.
        // In particular, wpilib is unable to log arrays of values (e.g. PoseObservations) where each of
        // those values themselves has a variable length array as a member (e.g. the different tags contributing to each pose observation).
        // In practice, this isn't a problem because we don't currently expect our coprocessor (raspberry pi 4)
        // to be running apriltag detection faster than 50 fps.
        PhotonPipelineResult mostRecentFrame = freshFrames.get(freshFrames.size()-1);
        if (!mostRecentFrame.hasTargets()) {
            Logger.processInputs(cam.getName()+"/mostRecentPoseEstimate", new PoseObservation());
            return Optional.empty();
        }

        // Repackage the data from PhotonVision into our custom PoseObservation type.
        PoseObservation freshPoseEstimate = new PoseObservation(
            this.robotPoseFromPhotonData(mostRecentFrame),
            mostRecentFrame.getTimestampSeconds(),
            this.getTagsUsed(mostRecentFrame),
            this.getAvgTagToCamMeters(mostRecentFrame),
            mostRecentFrame.multitagResult.isPresent() ? 0 : mostRecentFrame.targets.get(0).poseAmbiguity,
            this.camPoseFromPhotonData(mostRecentFrame)
        );

        // Update recent sightings
        for (int i = 0; i < freshPoseEstimate.tagsUsed.length; i += 1) {
            int tagID = freshPoseEstimate.tagsUsed[i];
            mostRecentSightings[tagID-1] = freshPoseEstimate.timestampSeconds;
        }

        // log values, then return output
        Logger.processInputs(cam.getName()+"/mostRecentPoseEstimate", freshPoseEstimate);
        Logger.processInputs(cam.getName(), new LoggableInputs() {
            public void toLog(LogTable table) { table.put("mostRecentSightings", mostRecentSightings); }
            public void fromLog(LogTable table) { mostRecentSightings = table.get("mostRecentSightings", mostRecentSightings); }
        });

        return Optional.of(freshPoseEstimate);
    }

    public double mostRecentSighting(int tagID) {
        return mostRecentSightings[tagID-1];
    }

    public boolean hasSeenRecently(int tagID, double howRecentlySeconds) {
        return (Timer.getTimestamp() - this.mostRecentSighting(tagID)) <= howRecentlySeconds;
    }

    public String getName() {
        return cam.getName();
    }


    private Pose3d robotPoseFromPhotonData(PhotonPipelineResult data) {

        Transform3d camPose_robotFrame = new Transform3d(camLocation_robotFrame, camOrientation_robotFrame);
        Transform3d robotPose_camFrame = camPose_robotFrame.inverse();

        if (data.multitagResult.isPresent()) {
            Transform3d camPose_fieldFrame = data.multitagResult.get().estimatedPose.best;
            Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);

            return new Pose3d(robotPose_fieldFrame.getTranslation(), robotPose_fieldFrame.getRotation());
        }
        else if (data.hasTargets()) {
            PhotonTrackedTarget singleTag = data.getTargets().get(0);

            Transform3d tagPose_fieldFrame = FieldConstants.tagPoseAsTransform(singleTag.fiducialId);
            Transform3d camPose_tagFrame = singleTag.getBestCameraToTarget().inverse();
            Transform3d camPose_fieldFrame = tagPose_fieldFrame.plus(camPose_tagFrame);
            Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);

            return new Pose3d(robotPose_fieldFrame.getTranslation(), robotPose_fieldFrame.getRotation());
        }

        // TODO: add demo tag stuff
        return new Pose3d(); // <- should never get here
    }

    private Pose3d camPoseFromPhotonData(PhotonPipelineResult data) {

        if (data.multitagResult.isPresent()) {
            Transform3d camPose_fieldFrame = data.multitagResult.get().estimatedPose.best;

            return new Pose3d(camPose_fieldFrame.getTranslation(), camPose_fieldFrame.getRotation());
        }
        else if (data.hasTargets()) {
            PhotonTrackedTarget singleTag = data.getTargets().get(0);

            Transform3d tagPose_fieldFrame = FieldConstants.tagPoseAsTransform(singleTag.fiducialId);
            Transform3d camPose_tagFrame = singleTag.getBestCameraToTarget().inverse();
            Transform3d camPose_fieldFrame = tagPose_fieldFrame.plus(camPose_tagFrame);

            return new Pose3d(camPose_fieldFrame.getTranslation(), camPose_fieldFrame.getRotation());
        }

        // TODO: add demo tag stuff
        return new Pose3d(); // <- should never get here
    }

    private Pose3d robotPoseFromCamPose(Pose3d camPose_fieldFrame) {
        Transform3d camPose_robotFrame = new Transform3d(camLocation_robotFrame, camOrientation_robotFrame);
        Transform3d robotPose_camFrame = camPose_robotFrame.inverse();

        return camPose_fieldFrame.plus(robotPose_camFrame);
    }


    private int[] getTagsUsed(PhotonPipelineResult data) {
        int[] output = new int[data.targets.size()];
        for (int i = 0; i < output.length; i += 1) {
            output[i] = data.targets.get(i).fiducialId;
        }
        return output;
    }


    private double getAvgTagToCamMeters(PhotonPipelineResult data) {
        double totalTagToCamMeters = 0;
        for (PhotonTrackedTarget tag : data.targets) {
            totalTagToCamMeters += tag.bestCameraToTarget.getTranslation().getNorm();
        }
        return totalTagToCamMeters / data.targets.size();
    }
}
