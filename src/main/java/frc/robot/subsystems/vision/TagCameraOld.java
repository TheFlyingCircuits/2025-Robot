package frc.robot.subsystems.vision;

import java.util.ArrayList;
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
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.PlayingField.FieldConstants;

public class TagCameraOld implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toLog'");
    }

    @Override
    public void fromLog(LogTable table) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
    }

    // private PhotonCamera cam;
    // private PhotonPoseEstimator poseCalculator;
    // private List<PoseObservation> poseObservations = new ArrayList<>();

    // private VisionSystemSim simulatedFieldLayout = new VisionSystemSim("layout");


    // public TagCameraOld(String name, Transform3d camPose_robotFrame) {
    //     cam = new PhotonCamera(name);

    //     poseCalculator = new PhotonPoseEstimator(
    //         FieldConstants.tagLayout,
    //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //         camPose_robotFrame
    //     );

    //     simulatedFieldLayout.addAprilTags(FieldConstants.tagLayout);
    //     SimCameraProperties simSettings = SimCameraProperties.PI4_LIFECAM_640_480();
    //     simSettings.setExposureTimeMs((1.0/120.0)*1000);
    //     simSettings.setFPS(60);
    //     PhotonCameraSim simCamera = new PhotonCameraSim(cam, simSettings);
    //     // simCamera.setMaxSightRange(5);
    //     int sideLengthPixels = 10 * 3; // tag is 6x6 data bits, but add 4 to the width for the black and white border. Then require each tag pixel take up at least 3 camera pixels.
    //     simCamera.setMinTargetAreaPixels(sideLengthPixels*sideLengthPixels);
    //     simulatedFieldLayout.addCamera(simCamera, camPose_robotFrame);
    // }

    // public Optional<PoseObservation> getNextPoseObservation() {
    //     if (poseObservations.isEmpty()) {
    //         return Optional.empty();
    //     }
    //     return Optional.of(poseObservations.remove(0));
    // }

    // public boolean hasUnreadPoseObservations() {
    //     return poseObservations.size() > 0;
    // }


    // public void periodic() {
    //     // If our camera coprocessor happens to be processing frames faster than the
    //     // main robot code on the rio, then it's possible to get more than one pose
    //     // observation from the camera in a single iteration of the main loop.
    //     //
    //     // While this isn't a problem on its own, it doesn't mesh well with the
    //     // structured logging capabilites of wpilib and advantage kit at the moment.
    //     // In particular, wpilib and advantage kit will only let you log a VLA (variable length array)
    //     // of a custom datatype if it doesn't have any VLAs as members.
    //     //
    //     // In our case, we'd like to log a list of PoseObservations each loop,
    //     // where each PoseObservation itself has a list of the tags used to get the pose.
    //     poseObservations.clear(); // <- this is prob safer for if camera is ready way before robot code. we don't want to build up tons of obvervstions! I made this mistake in sim!
    //     for (PhotonPipelineResult frame : cam.getAllUnreadResults()) {
    //         Optional<EstimatedRobotPose> poseAndMetadata = poseCalculator.update(frame);

    //         if (poseAndMetadata.isPresent()) {
    //             poseObservations.add(new PoseObservation(poseAndMetadata.get()));
    //         }
    //     }


    //     Logger.processInputs(cam.getName(), this);
    // }

    // public void simPeriodic(Pose2d wheelsOnlyPose) {
    //     simulatedFieldLayout.update(wheelsOnlyPose);
    // }

    // @Override
    // public void toLog(LogTable table) {
    //     table.put("numObservations", poseObservations.size());
    //     for (int i = 0; i < poseObservations.size(); i += 1) {
    //         table.put("Observation"+(i+1), poseObservations.get(i));
    //     }
    // }

    // @Override
    // public void fromLog(LogTable table) {
    //     int numObservations = table.get("numObservations", 0);

    //     for (int i = 0; i < numObservations; i += 1) {
    //         PoseObservation defaultValue = new PoseObservation(new Pose3d(), -1, new int[0], -1, 0);
    //         PoseObservation valueFromLog = table.get("Observation"+(i+1), defaultValue);
    //         poseObservations.add(valueFromLog);
    //     }
    // }

    // private void manuallyGetPose() {
    //     for (PhotonPipelineResult frame : cam.getAllUnreadResults()) {

    //         if (frame.multitagResult.isPresent()) {
    //             Transform3d camPose_fieldFrame = frame.multitagResult.get().estimatedPose.best;
    //             Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);
    //             // record data
    //         }
    //         else if (frame.hasTargets()) {
    //             PhotonTrackedTarget singleTag = frame.getTargets().get(0);

    //             Transform3d tagPose_fieldFrame = FieldConstants.tagPoseAsTransform(singleTag.fiducialId);
    //             Transform3d camPose_tagFrame = singleTag.getBestCameraToTarget().inverse();
    //             Transform3d camPose_fieldFrame = tagPose_fieldFrame.plus(camPose_tagFrame);
    //             Transform3d robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame);
    //             // record data
    //         }
    //     }
    // }


    // robotTowardsRobotX_camFrame = robotTowardsRobotX_robotFrame.rotatedBy(robotOrientation_camFrame);
    // robotTowardsRobotX_fieldFrame = robotTowardsRobotX_camFrame.rotatedBy(camOrientation_fieldFrame);
    //
    // robotXHat_fieldCoords = [camAxes_fieldCoords][robotAxes_camCoords]xHat
    //
    // this is the opposite order of robotPose_fieldFrame = camPose_fieldFrame.plus(robotPose_camFrame)
    // which internally does robotOrientation_fieldFrame = camOrientationFieldFrame.roatedBy(robotOrientation_camFrame);
    //
    // Part of the difference is in the types. First example pumps a vector through a rotation matrix.
    // second example rotates one rotation matrix by another rotation matrix. I think this has
    // to do with the fact that wpilib matrix.rotateBy(otherMatrix) is extrinsic, and extrinsic
    // is equivalent to reversed order intrinsic, which is what I'm doing?
    //
    // matricies by default perform active transformations. when doing reference frame transfers you want
    // passive transformations. The vector stays the same, but the coordinates are rotated.
    // In particular, the coordinates are rotated in the opposite direction you would do an active transform?
    //
    // 3b1b linear algebra ch. 13 has the answers I think.
}
