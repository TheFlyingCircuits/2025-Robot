package frc.robot.PlayingField;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.FlyingCircuitUtils;

public class StandardFieldElement implements FieldElement {
    // A standard FieldElement with no extra bells or whistles.
    // Just a name, and some alliance-aware pose/tag info.

    protected final Pose3d redPose;
    protected final Pose3d bluePose;
    protected final int redTagID;
    protected final int blueTagID;
    protected final String name;

    protected StandardFieldElement(int redTagID, int blueTagID, String name) {
        this.redPose = FieldConstants.tagLayout.getTagPose(redTagID).get();
        this.bluePose = FieldConstants.tagLayout.getTagPose(blueTagID).get();
        this.redTagID = redTagID;
        this.blueTagID = blueTagID;
        this.name = name;
    }

    public Pose3d getPose() {
        return FlyingCircuitUtils.getAllianceDependentValue(redPose, bluePose, new Pose3d());
    }

    public int getTagID() {
        return FlyingCircuitUtils.getAllianceDependentValue(redTagID, blueTagID, -1);
    }

    public String getName() {
        return name;
    }
}
