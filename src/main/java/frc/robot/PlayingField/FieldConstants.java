package frc.robot.PlayingField;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // See Team Update 12
    public static final double maxX = tagLayout.getFieldLength();
    public static final double maxY = tagLayout.getFieldWidth();
    public static final Translation2d midField = new Translation2d(maxX / 2.0, maxY / 2.0);

    public static Pose3d tagPose(int tagID) {
        return tagLayout.getTagPose(tagID).get();
    }

    public static Transform3d tagPoseAsTransform(int tagID) {
        Pose3d pose = tagLayout.getTagPose(tagID).get();
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    // Reef Geometry taken from the official field drawings (pages 102-106)
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-GameSpecific.pdf

    /** The center-to-center distance between two stalks on the same face of the reef. */
    public static final double stalkSeparationMeters = Units.inchesToMeters(12.94);

    /** The heights of the highest point on each branch level. */
    public static final double[] branchHeightMeters = {0, // The lowest level on the reef is called L1 in the manual (as opposed to L0), so we have a dummy value at index 0 so that our indicies can match the nomenclature.
                                                       Units.inchesToMeters(17.88), // Highest point of L1 (this is just the height of one of the reef faces, which isn't technically branch, but I still think it fits here).
                                                       Units.inchesToMeters(31.72), // Highest point of L2
                                                       Units.inchesToMeters(47.59), // Highest point of L3
                                                       Units.inchesToMeters(71.87)  // Highest point of L4
                                                      };
    
    /** The horizontal distance between a reef face and the outermost part of each of its branches. */
    public static final double[] branchInsetMeters = {0,
                                                      0, // L1 has no inset from the reef face, because its furthest point out is at the reef face!
                                                      Units.inchesToMeters(1.61),
                                                      Units.inchesToMeters(1.61),
                                                      Units.inchesToMeters(1.18)
                                                     };

    /** The angle that each branch level makes with the floor (unsigned / all positive) */
    public static final double[] branchPitchRadians = {0,
                                                       0,
                                                       Units.degreesToRadians(35),
                                                       Units.degreesToRadians(35),
                                                       Units.degreesToRadians(90)
                                                      };

    public static final double branchDiameterMeters = Units.inchesToMeters(1.66); // outer diameter, because that's all that matters for gameplay!
    public static final double coralInnerDiameterMeters = Units.inchesToMeters(4);
    public static final double coralOuterDiameterMeters = Units.inchesToMeters(4.5);
    public static final double branchRadiusMeters = branchDiameterMeters / 2.0;
    public static final double coralInnerRadiusMeters = coralInnerDiameterMeters / 2.0;
    public static final double coralOuterRadiusMeters = coralOuterDiameterMeters / 2.0;
    public static final double coralLengthMeters = Units.inchesToMeters(11.875);

    /** When viewing the stalk so that the branches are facing to the right, this is the
     *  width of the bounding box you'd need to completely encase the stalk.
     *  <pre>&nbsp;
     *         | |
     *         | |<---
     *         | |
     *        / /
     *       / /
     *      / /
     *     / /
     *     | |
     *     | |
     *     | | / /
     *     | |/ /
     *     | / /
     *     |  /
     *     | |
     * --->| | / /
     *     | |/ /
     *     | / /
     *     |  /
     *     | |
     * </pre>
     *
     */
    public static final double stalkDepthMeters = Units.inchesToMeters(11.66);

    /** The horizontal distance between a reef face and the center of the hole that holds a stalk. */
    public static final double stalkInsetMeters = (branchInsetMeters[4] + stalkDepthMeters) - branchRadiusMeters;

    /** Used for photon vision simulation. */
    public static final VisionSystemSim simulatedTagLayout = new VisionSystemSim("simulatedTagLayout");
    public static final VisionSystemSim simulatedCoralLayout = new VisionSystemSim("simulatedCoralLayout");
    static {
        simulatedTagLayout.addAprilTags(FieldConstants.tagLayout);

        // add some coral in front of the loading stations
        // (for simplicity, coral will appear as a rectangular prisim to the simulated cameras)
        TargetModel simulatedCoralShape = new TargetModel(FieldConstants.coralLengthMeters, FieldConstants.coralOuterDiameterMeters, FieldConstants.coralOuterDiameterMeters);
        int[] loadingStationTagIDs = {1, 13, 2, 12};  // left red, left blue, right red, right blue
        
        for (int loadingStationTagID : loadingStationTagIDs) {
            Pose3d loadingStationPose = FieldConstants.tagLayout.getTagPose(loadingStationTagID).get();
            double coralZ_loadingStationFrame = -loadingStationPose.getZ() + FieldConstants.coralOuterRadiusMeters;

            // 3 corals per loading station
            Transform3d[] coralPoses_loadingStationFrame = { new Transform3d(0.5, 0.25*0, coralZ_loadingStationFrame, new Rotation3d()),
                                                             new Transform3d(0.5, -0.25*0, coralZ_loadingStationFrame, new Rotation3d()),
                                                             new Transform3d(-0.5, 0, coralZ_loadingStationFrame, new Rotation3d())
                                                           };

            for (Transform3d coralPose_loadingStationFrame : coralPoses_loadingStationFrame) {
                Pose3d coralPose_fieldFrame = loadingStationPose.plus(coralPose_loadingStationFrame);
                simulatedCoralLayout.addVisionTargets("coral", new VisionTargetSim(coralPose_fieldFrame, simulatedCoralShape));
            }
        }
    }
}
