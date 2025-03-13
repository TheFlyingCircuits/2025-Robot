package frc.robot;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionMeasurement;

public class ReferenceFrameTracker {
    // // alternative idea for letting everyone have pose information
    // // without having to pass the drivetrain around everywhere or
    // // use functional interfaces. Based on ROS tf2.

    // public enum ReferenceFrame {
    //     FIELD, ROBOT, PIVOT, WRIST, INTAKE_CAM;

    //     private List<ReferenceFrame> neighbors;
    //     private List<Pose3d> poses;

    //     public Pose3d getPoseIn(ReferenceFrame perspective) {
    //         // A*
    //         Queue<ReferenceFrame> frontier = new ArrayDeque<>();
    //         Queue<Pose3d> accumulatedTransforms = new ArrayDeque<>();
    //         frontier.add(this);
    //         accumulatedTransforms.add(new Pose3d());
    //         Set<ReferenceFrame> visited = EnumSet.of(this);

    //         while (!frontier.isEmpty()) {
    //             ReferenceFrame current = frontier.remove();
    //             Pose3d accumulatedTransform = accumulatedTransforms.remove();

    //             if (current == perspective) {
    //                 return;
    //             }
    //             for (int i = 0; i < neighbors.size(); i += 1) {
    //                 ReferenceFrame neighbor = current.neighbors.get(i);
    //                 Pose3d poseOfNeighbor = current.poses.get(i);
    //                 if (!visited.contains(neighbor)) {
    //                     frontier.add(neighbor);
    //                     visited.add(neighbor);
    //                 }
    //             }
    //         }


    //     }

    //     public void setPose(Pose3d pose, ReferenceFrame perspective) {
    //         parent = perspective;
    //         pose = poseInParentFrame;
    //     }
    // }

    // private SwerveDrivePoseEstimator fusedPoseEstimator;
    // private SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;


    // public Pose3d getRobotPose() {
    //     fusedPoseEstimator.
    //     return new Pose3d(fusedPoseEstimator.getEstimatedPosition());
    // }
    // public Translation3d getRobotLocation() {
    //     return getRobotPose().getTranslation();
    // }
    // public Rotation3d getRobotOrientation() {
    //     return getRobotPose().getRotation();
    // }

    // public Pose2d getRobotPose2d() {
    //     return fusedPoseEstimator.getEstimatedPosition();
    // }
    // public Translation2d getRobotLocation2d() {
    //     return getRobotPose2d().getTranslation();
    // }
    // public Rotation2d getRobotOrientation2d() {
    //     return getRobotPose2d().getRotation();
    // }

    // public Pose3d getRobotPose(double timestampSeconds) {
    //     Optional<Pose2d> interpolatedPose = fusedPoseEstimator.sampleAt(timestampSeconds);
    //     if (interpolatedPose.isPresent()) {
    //         return new Pose3d(interpolatedPose.get());
    //     }
    //     return getRobotPose();
    // }
    // public Translation3d getRobotLocation(double timestampSeconds) {
    //     return getRobotPose(timestampSeconds).getTranslation();
    // }
    // public Rotation3d getRobotOrientation(double timestampSeconds) {
    //     return getRobotPose(timestampSeconds).getRotation();
    // }

    // public Pose2d getRobotPose2d(double timestampSeconds) {
    //     Optional<Pose2d> interpolatedPose = fusedPoseEstimator.sampleAt(timestampSeconds);
    //     if (interpolatedPose.isPresent()) {
    //         return interpolatedPose.get();
    //     }
    //     return getRobotPose2d();
    // }
    // public Translation2d getRobotLocation2d(double timestampSeconds) {
    //     return getRobotPose2d(timestampSeconds).getTranslation();
    // }
    // public Rotation2d getRobotOrientation2d(double timestampSeconds) {
    //     return getRobotPose2d(timestampSeconds).getRotation();
    // }



    // public void updateRobotPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    //     fusedPoseEstimator.update(gyroAngle, modulePositions);
    //     wheelsOnlyPoseEstimator.update(gyroAngle, modulePositions);
    // }

    // public void updateRobotPose(PoseObservation visionMeasurement) {
    //     // calculate standard deviations based on tag distance
    //     double slopeStdDevMeters_PerMeterX = 0.008;
    //     double slopeStdDevMeters_PerMeterY = 0.008;

    //     if (visionMeasurement.tagsUsed.length > 1) {
    //         slopeStdDevMeters_PerMeterX = 0.004;
    //         slopeStdDevMeters_PerMeterY = 0.009;
    //     }

    //     Matrix<N3, N1> standardDeviations = VecBuilder.fill(slopeStdDevMeters_PerMeterX * visionMeasurement.avgTagToCamMeters,
    //                                                         slopeStdDevMeters_PerMeterY * visionMeasurement.avgTagToCamMeters,
    //                                                         99999);

    //     fusedPoseEstimator.addVisionMeasurement(visionMeasurement.robotPose.toPose2d(), visionMeasurement.timestampSeconds, standardDeviations);
    // }

    // public void resetRobotPose(Pose2d newPose) {
    //     fusedPoseEstimator.resetPose(newPose);
    // }

    // public ReferenceFrameTracker() {
    //     fusedPoseEstimator = new SwerveDrivePoseEstimator(
    //         DrivetrainConstants.swerveKinematics, 
    //         gyroInputs.robotYawRotation2d,
    //         getModulePositions(),
    //         new Pose2d(),
    //         stateStdDevs,
    //         visionStdDevs
    //     );

    //     wheelsOnlyPoseEstimator = new SwerveDrivePoseEstimator(
    //         DrivetrainConstants.swerveKinematics,
    //         gyroInputs.robotYawRotation2d,
    //         getModulePositions(), 
    //         new Pose2d());
    // }



    // private void updatePoseEstimator() {
    //     double totalAccelMetersPerSecondSquared = Math.hypot(gyroInputs.robotAccelX, gyroInputs.robotAccelY);
    //     totalAccelMetersPerSecondSquared = Math.hypot(totalAccelMetersPerSecondSquared, gyroInputs.robotAccelZ);

    //     Logger.recordOutput("drivetrain/accelMagnitude", totalAccelMetersPerSecondSquared);

    //     // if (totalAccelMetersPerSecondSquared > 10) {
    //     //     hasBeenBumped = true;
    //     // }

    //     // if (hasBeenBumped && !visionInputs.visionMeasurements.isEmpty()) {
    //     //     hasBeenBumped = false;
    //     //     setTranslationToVisionMeasurement();
    //     // }

    //     fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
    //     wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());


    //     List<Pose2d> trackedTags = new ArrayList<Pose2d>();
    //     for (VisionMeasurement visionMeasurement : visionInputs.visionMeasurements) {


    //         Translation2d visionTranslation = visionMeasurement.robotFieldPose.getTranslation();
    //         Translation2d estimatedTranslation = fusedPoseEstimator.getEstimatedPosition().getTranslation();

    //         // Dont' allow the robot to teleport (Can cause problems when we get bumped)
    //         double teleportToleranceMeters = 2.0;
    //         if (visionTranslation.getDistance(estimatedTranslation) > teleportToleranceMeters) { 
    //             continue;
    //         }

    //         // This measurment passes all our checks, so we add it to the fusedPoseEstimator
    //         fusedPoseEstimator.addVisionMeasurement(
    //             visionMeasurement.robotFieldPose, 
    //             visionMeasurement.timestampSeconds, 
    //             visionMeasurement.stdDevs
    //         );

    //         // Log which tags have been used.
    //         for (int id : visionMeasurement.tagsUsed) {
    //             Pose2d tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(id).get().toPose2d();
    //             trackedTags.add(tagPose);

    //             // if (id == FieldElement.getSpeakerTagID()) {
    //             //     mostRecentSpeakerTagMeasurements.add(visionMeasurement);

    //             //     if (mostRecentSpeakerTagMeasurement == null) {
    //             //         mostRecentSpeakerTagMeasurement = visionMeasurement;
    //             //     }

    //             //     if (visionMeasurement.timestampSeconds > mostRecentSpeakerTagMeasurement.timestampSeconds) {
    //             //         mostRecentSpeakerTagMeasurement = visionMeasurement;
    //             //     }
    //             // }
    //         }
    //     }

    //     Logger.recordOutput("drivetrain/trackedTags", trackedTags.toArray(new Pose2d[0]));
    // }
    
}
