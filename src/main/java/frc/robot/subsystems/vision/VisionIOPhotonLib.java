package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonLib implements VisionIO {
    

    List<PhotonCamera> tagCameras;
    List<PhotonPoseEstimator> poseEstimators;

    PhotonCamera intakeCamera;

    public VisionIOPhotonLib() {

        intakeCamera = new PhotonCamera("intakeCam");

        System.gc();

        tagCameras = Arrays.asList(
            new PhotonCamera(VisionConstants.tagCameraNames[0]),
            new PhotonCamera(VisionConstants.tagCameraNames[1])
            // new PhotonCamera(VisionConstants.tagCameraNames[3])
            // new PhotonCamera(VisionConstants.tagCameraNames[3])
        );

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
                    // tagCameras.get(i)
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
    private Matrix<N3, N1> getVisionStdDevs(double distToTargetMeters, boolean useMultitag) {

        double slopeStdDevMetersPerMeterX;
        double slopeStdDevMetersPerMeterY;



        // previous working vision
        
        if (useMultitag) {
            slopeStdDevMetersPerMeterX = 0.004;
            slopeStdDevMetersPerMeterY = 0.009;
        }
        else {
            slopeStdDevMetersPerMeterX = 0.008;
            slopeStdDevMetersPerMeterY = 0.02;
        }

        // previous linear model
        return VecBuilder.fill(
            slopeStdDevMetersPerMeterX*distToTargetMeters,
            slopeStdDevMetersPerMeterY*distToTargetMeters,
            99999
        );


        
        // double squareFactor = 0.5;
        // return VecBuilder.fill(0.04, 0.04, 99999);

        // // square model
        // return VecBuilder.fill(
        //     squareFactor*slopeStdDevMetersPerMeterX*Math.pow(distToTargetMeters, 3),
        //     squareFactor*slopeStdDevMetersPerMeterY*Math.pow(distToTargetMeters, 3),
        //     99999
        // );
    }


    /**
     * Generates a VisionMeasurement object based off of a camera and its pose estimator.
     * @param camera - PhotonCamera object of the camera you want a result from.
     * @param estimator - PhotonPoseEstimator that MUST correspond to the PhotonCamera.
     * @return - Optional VisionMeasurement. This is empty if the camera does not see a reliable target.
     */
    private Optional<VisionMeasurement> updateTagCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        VisionMeasurement output = new VisionMeasurement();

        Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update(camera.getLatestResult());
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
        if (output.averageTagDistanceMeters > 7) {
            return Optional.empty();
        }


        output.robotFieldPose = poseEstimate.estimatedPose.toPose2d();
        output.timestampSeconds = poseEstimate.timestampSeconds;
        output.stdDevs = getVisionStdDevs(output.averageTagDistanceMeters, (seenTags.size() > 1));  //different standard devs for different methods of detecting apriltags
        output.cameraName = camera.getName();
        output.tagsUsed = new int[seenTags.size()];
        for (int i = 0; i < seenTags.size(); i += 1) {
            output.tagsUsed[i] = seenTags.get(i).getFiducialId();
        }

        Logger.recordOutput("demoMode/robotFullPose", poseEstimate.estimatedPose);

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

    private List<Translation3d> updateIntakeCamera() {

        List<Translation3d> detectedCorals = new ArrayList<Translation3d>();
        if (!intakeCamera.isConnected()) {
            Logger.recordOutput("intakeCamConnected", false);
            return detectedCorals; // can't see anything if unplugged.
                                   // shouldn't default to most recent frame.
        }
        Logger.recordOutput("intakeCamConnected", true);

        PhotonPipelineResult intakeCameraResult = intakeCamera.getLatestResult();

        for (PhotonTrackedTarget target : intakeCameraResult.getTargets()) {
            // Negate the pitch and yaw that's reported by photon vision because
            // their convention isn't consistent with a right handed coordinate system.
            double coralYawDegrees = -target.getYaw();
            double coralPitchDegrees = -target.getPitch();

            if (coralPitchDegrees > 15) continue;

            // Use the reported pitch and yaw to calculate a unit vector in the camera
            // frame that points towards the coral.
            Rotation3d directionOfCoral = new Rotation3d(0, Math.toRadians(coralPitchDegrees), Math.toRadians(coralYawDegrees));
            Translation3d unitTowardsCoral = new Translation3d(1, directionOfCoral);

            // Start the process of finding the full 3D distance from the camera to the coral
            // by finding the coordinates of the normal vector of the floor,
            // as seen in the camera frame.
            Translation3d robotOrigin_robotFrame = new Translation3d(0, 0, 0.1);
            Translation3d aboveTheFloor_robotFrame = new Translation3d(0, 0, 1);
            Translation3d robotOrigin_camFrame = intakeCameraCoordsFromRobotCoords(robotOrigin_robotFrame);
            Translation3d aboveTheFloor_camFrame = intakeCameraCoordsFromRobotCoords(aboveTheFloor_robotFrame);
            Translation3d floorNormal_camFrame = aboveTheFloor_camFrame.minus(robotOrigin_camFrame);

            // Find where the vector that points from the camera to the coral intersects
            // the plane of the floor.
            Translation3d floorAnchor = robotOrigin_camFrame;
            double distanceToCoral = floorAnchor.toVector().dot(floorNormal_camFrame.toVector())
                                    / unitTowardsCoral.toVector().dot(floorNormal_camFrame.toVector());

            // extend the original unit vector to the intersection point in the plane
            Translation3d coral_camFrame = unitTowardsCoral.times(distanceToCoral);
            Translation3d coral_robotFrame = robotCoordsFromIntakeCameraCoords(coral_camFrame);

            if (coral_robotFrame.getNorm() > 3 || coral_robotFrame.getNorm() < Units.inchesToMeters(DrivetrainConstants.frameWidthMeters/2+14)) {
                continue;
            };

            detectedCorals.add(coral_robotFrame);
        }


            
        return detectedCorals;
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

        inputs.detectedCoralsRobotFrame = updateIntakeCamera();
    }
}
