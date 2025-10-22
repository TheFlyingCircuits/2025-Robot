package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.PlayingField.FieldConstants;


public class VisionIOPhotonLib implements VisionIO {
    

    List<PhotonCamera> tagCameras;
    List<PhotonPoseEstimator> poseEstimators;

    // PhotonCamera intakeCamera;

    public VisionIOPhotonLib() {

        // intakeCamera = new PhotonCamera("intakeCam");

        System.gc();
        
        tagCameras = new ArrayList<PhotonCamera>();
        for (String name : VisionConstants.tagCameraNames) {
            tagCameras.add(new PhotonCamera(name));
        }

        /* When in demo mode, the apriltags will probably be pitched/rolled a bit
         * relative to their normal vertical orientation because they will be held
         * by a person running the demo rather than being mounted to a wall.
         * The tags may also be at a different height than normal.
         * 
         * In order to still measure the robot's "field oreinted pose" accurately,
         * we must inform the pose estimators of the new pitch/roll/height of the tags
         * by updating the TagLayout. However, I've discovered through testing that
         * updated tag layouts involving more than one tag are only taken into account
         * when running pose estimation on the rio itself, and aren't taken into account
         * when running pose estimation on a co-processor. To get around this, we use
         * an alternative pose estimation strategy when in demo mode.
         */
        PoseStrategy estimationStrat = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        // Todo: learn about "averaging" 3D orientations?
        //       it seems like it's not super straight forward,
        //       but I don't have time for a rabbit hole right now.
        // estimationStrat = PoseStrategy.AVERAGE_BEST_TARGETS;

        poseEstimators = new ArrayList<PhotonPoseEstimator>();
        for (int i = 0; i < tagCameras.size(); i++) {
            poseEstimators.add(
                new PhotonPoseEstimator(
                    VisionConstants.aprilTagFieldLayout,
                    estimationStrat,
                    VisionConstants.tagCameraTransforms[i]
                )
            );
        }

    }

    /**
     * Calculates a matrix of standard deviations of the vision pose estimate, in meters and degrees. 
     * This is a function of the distance from the camera to the april tag.
     * @param distToTargetMeters - Distance from the camera to the apriltag.
     * @return A vector of the standard deviations given distance in X (m), Y (m), and Rotation (Rad)
     */

    
    // private void makeTagCamsAgree(Pose2d knownRobotPose) {
    //     makeTagCamsAgree(new Pose3d(knownRobotPose));
    // }



    private void makeTagCamsAgree(Pose3d knownRobotPose) {
        for (PhotonCamera tagCam : tagCameras) {
            List<PhotonPipelineResult> newFrames = tagCam.getAllUnreadResults();
            if (newFrames.size() == 0) {
                continue;
            }

            PhotonPipelineResult mostRecentFrame = newFrames.get(newFrames.size()-1);

            Transform3d camPose_fieldFrame = new Transform3d();
            if (mostRecentFrame.multitagResult.isPresent()) {
                camPose_fieldFrame = mostRecentFrame.multitagResult.get().estimatedPose.best;
            }
            else if (mostRecentFrame.hasTargets()) {
                // single tag
                PhotonTrackedTarget singleTag = mostRecentFrame.targets.get(0);
                Transform3d tagPose_camFrame = singleTag.bestCameraToTarget;
                Transform3d camPose_tagFrame = tagPose_camFrame.inverse();
                Pose3d tagPose_fieldFrame = Constants.VisionConstants.aprilTagFieldLayout.getTagPose(singleTag.fiducialId).get();
                Transform3d tagTransform_fieldFrame = new Transform3d(tagPose_fieldFrame.getTranslation(), tagPose_fieldFrame.getRotation());
                camPose_fieldFrame = tagTransform_fieldFrame.plus(camPose_tagFrame);
            }
            else {
                continue;
            }

            Pose3d camPose_fieldFrame_asPose = new Pose3d(camPose_fieldFrame.getTranslation(), camPose_fieldFrame.getRotation());
            Pose3d camPose_robotFrame = camPose_fieldFrame_asPose.relativeTo(knownRobotPose);
            Logger.recordOutput("tagCamAgree/"+tagCam.getName()+"/pose", camPose_robotFrame);
            Logger.recordOutput("tagCamAgree/"+tagCam.getName()+"/rollDegrees", Units.radiansToDegrees(camPose_robotFrame.getRotation().getX()));
            Logger.recordOutput("tagCamAgree/"+tagCam.getName()+"/pitchDegrees", Units.radiansToDegrees(camPose_robotFrame.getRotation().getY()));
            Logger.recordOutput("tagCamAgree/"+tagCam.getName()+"/yawDegrees", Units.radiansToDegrees(camPose_robotFrame.getRotation().getZ()));
        }
    }


    /**
     * Generates a VisionMeasurement object based off of a camera and its pose estimator.
     * @param camera - PhotonCamera object of the camera you want a result from.
     * @param estimator - PhotonPoseEstimator that MUST correspond to the PhotonCamera.
     * @return - Optional VisionMeasurement. This is empty if the camera does not see a reliable target.
     */
    private Optional<VisionMeasurement> updateTagCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        VisionMeasurement output = new VisionMeasurement();

        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

        if (pipelineResults.isEmpty()) return Optional.empty();

        //TODO: don't just use the first result in the list
        Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update(pipelineResults.get(0));
        if (poseEstimatorResult.isEmpty()) {
            return Optional.empty();
        }
        EstimatedRobotPose poseEstimate = poseEstimatorResult.get();
        List<PhotonTrackedTarget> seenTags = poseEstimate.targetsUsed;
        

        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (seenTags.size() == 1 && seenTags.get(0).getPoseAmbiguity() > 0.2) {
            return Optional.empty();
        }
        
        for (PhotonTrackedTarget tag : seenTags) {
            double distance = tag.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
            output.averageTagDistanceMeters += distance/seenTags.size();
        }

        // don't add vision measurements that are too far away
        // for reference: it is 6 meters from speaker tags to wing.
        if (output.averageTagDistanceMeters > 5 && seenTags.size() == 1) {
            return Optional.empty();
        }


        output.robotFieldPose = poseEstimate.estimatedPose.toPose2d();
        output.timestampSeconds = poseEstimate.timestampSeconds;
        output.cameraName = camera.getName();
        output.tagsUsed = new int[seenTags.size()];
        for (int i = 0; i < seenTags.size(); i += 1) {
            output.tagsUsed[i] = seenTags.get(i).getFiducialId();
        }

        return Optional.of(output);
    }


    public static Translation3d intakeCameraCoordsFromRobotCoords(Translation3d robotCoords) {
        Transform3d robotAxesFromCamPerspective = VisionConstants.robotToCoralCamera.inverse();
        return robotCoords.rotateBy(robotAxesFromCamPerspective.getRotation()).plus(robotAxesFromCamPerspective.getTranslation());
    }

    public static Translation3d robotCoordsFromIntakeCameraCoords(Translation3d intakeCamCoords) {
        Transform3d camAxesFromRobotPerspective = VisionConstants.robotToCoralCamera;
        return intakeCamCoords.rotateBy(camAxesFromRobotPerspective.getRotation()).plus(camAxesFromRobotPerspective.getTranslation());
    }


    public void makeTagCamsAgree() {
        int tagID = (int)SmartDashboard.getNumber("tagCamsAgree/Face", 9);
        SmartDashboard.putNumber("tagCamsAgree/Face", tagID);
        Pose3d calibrationFace = FieldConstants.tagLayout.getTagPose(tagID).get();

        double inchesBack = SmartDashboard.getNumber("tagCamsAgree/inchesBack", 10.1);
        SmartDashboard.putNumber("tagCamsAgree/inchesBack", inchesBack);
        double metersBack = Units.inchesToMeters(inchesBack);
        double pushOutDistanceMeters = metersBack + (DrivetrainConstants.frameWidthMeters/2.0);

        boolean facingReef = SmartDashboard.getBoolean("tagCamsAgree/facingReef", true);
        SmartDashboard.putBoolean("tagCamsAgree/facingReef", facingReef);
        Rotation3d rotationFromFace = facingReef ? new Rotation3d(Rotation2d.k180deg) : 
            new Rotation3d(Rotation2d.kZero); 

        double zValueOffset = 0;

        Transform3d offset = new Transform3d(pushOutDistanceMeters, 0, zValueOffset, rotationFromFace);
        Logger.recordOutput("vision/calibrationPose", calibrationFace.plus(offset));
        // I am assuming we are going to use a tag that is flat with no pitch
        // will edit the z value and make sure if works because we will have a tag at
        // any height for the jig so if we use like the source tag will have to say the robot is floating
        this.makeTagCamsAgree(calibrationFace.plus(offset));
    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {


        inputs.visionMeasurements = new ArrayList<VisionMeasurement>();
        
        for (int i = 0; i < tagCameras.size(); i++) {
            Optional<VisionMeasurement> camResult = updateTagCamera(
                tagCameras.get(i), poseEstimators.get(i)
            );

            if (camResult.isPresent()) {
                inputs.visionMeasurements.add(camResult.get());
            }
        }

        //sorts visionMeasurements by standard deviations in the x direction, biggest to smallest
        // Collections.sort(inputs.visionMeasurements, new Comparator<VisionMeasurement>() {
        //     @Override
        //     public int compare(VisionMeasurement o1, VisionMeasurement o2) {
        //         return -Double.compare(o1.stdDevs.get(0,0), o2.stdDevs.get(0,0));
        //     }
        // });

        inputs.visionMeasurements.sort(new Comparator<VisionMeasurement>() {
            public int compare(VisionMeasurement a, VisionMeasurement b) {
                return Double.compare(a.timestampSeconds, b.timestampSeconds);
            }
        });

        inputs.detectedCoralsRobotFrame = new ArrayList<>();

        // this.makeTagCamsAgree();
    }
}
