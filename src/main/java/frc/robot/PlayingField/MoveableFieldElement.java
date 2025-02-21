package frc.robot.PlayingField;

import edu.wpi.first.math.geometry.Pose3d;

public class MoveableFieldElement implements FieldElement {
    // Useful for handeld apriltags when doing practice / demos without a full field.
    // May also be useful for game piece tracking, though that seems a little indirect.
    private String name;
    private Pose3d pose = new Pose3d();
    private int tagID = 0;

    protected MoveableFieldElement(String name) {
        this.name = name;
    }

    public void setPose(Pose3d newPose_fieldFrame) {
        pose = newPose_fieldFrame;
    }

    public void setTagID(int tagID) {
        this.tagID = tagID;
    }

    public Pose3d getPose() {
        return pose;
    }

    public String getName() {
        return name;
    }

    public int getTagID() {
        return tagID;
    }

}
