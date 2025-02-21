package frc.robot.PlayingField;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FlyingCircuitUtils;

public class ReefStalk implements FieldElement {
    // A ReefStalk has everything in a StandardFieldElement,
    // as well as the ability to get its associated
    // ReefFace and ReefBranches

    protected final Pose3d redPose;
    protected final Pose3d bluePose;
    protected final int redTagID;
    protected final int blueTagID;
    protected final String name;

    protected final ReefFace reefFace;
    protected final ReefBranch[] branches = new ReefBranch[3]; // [level2, level3, level4]


    protected ReefStalk(ReefFace reefFace, char label) {
        this.redTagID = reefFace.redTagID;
        this.blueTagID = reefFace.blueTagID;
        this.name = "STALK_"+label;
        Pose3d redTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(redTagID).get();
        Pose3d blueTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(blueTagID).get();


        // Top down view of a reef face with local coordinate systems:
        // 
        //          --------------------------------------
        //          | leftStalk → +Y     rightStalk → +Y |
        //          |     ↓                   ↓          |
        //          |    +X                  +X          |
        //          --------------------------------------
        //                       aprilTag → +Y
        //                            ↓
        //                           +X
        //
        // 
        // "Left" and "Right" are determined from a robot's perspective
        // as it goes up to score on the stalk.
        //
        // "left" stalks  have negative y coordinates in the tag's frame, and
        // "right" stalks have positive y coordinates in the tag's frame.
        double stalkY_tagFrame = FieldConstants.stalkSeparationMeters / 2.0;;
        if (this.isLeftStalk()) {
            stalkY_tagFrame *= -1;
        }

        // Because the stalk has a rather large extent, and arguably has a few different
        // places that could make sense as an origin for its local coordinate system,
        // I just picked the one that felt most appropriate / made the most sense to me.
        // In particular, I've placed stalk's XY "location" to be aligned with the
        // hole in the reef that holds the stalk, and its Z "location" to be halfway up
        // the maximum height of the stalk.
        double stalkX_tagFrame = -FieldConstants.stalkInsetMeters;
        double stalkHeightMeters = FieldConstants.branchHeightMeters[FieldConstants.branchHeightMeters.length-1];
        double stalkZ_tagFrame = (stalkHeightMeters / 2.0) - redTagPose_fieldFrame.getZ(); // subtract tag height cause we're in the tag frame at this point.

        Translation3d stalkLocation_tagFrame = new Translation3d(stalkX_tagFrame, stalkY_tagFrame, stalkZ_tagFrame);
        Transform3d stalkPose_tagFrame = new Transform3d(stalkLocation_tagFrame, new Rotation3d());

        this.redPose = redTagPose_fieldFrame.transformBy(stalkPose_tagFrame);
        this.bluePose = blueTagPose_fieldFrame.transformBy(stalkPose_tagFrame);

        // Link up this stalk to its associated reefFace and branches.
        // Because the trough is considered level 1, the actual branches
        // start at level 2 (i.e. branches = [level2, level3, level4]).
        this.reefFace = reefFace;
        for (int branchLevel = 2; branchLevel <= 4; branchLevel += 1) {
            branches[branchLevel-2] = new ReefBranch(this, branchLevel);
        }
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

    protected boolean isLeftStalk() {
        // left and right is determined from the robot's perspecive
        // as it goes up to score on the stalk.
        //
        // Left Stalks: A, C, E, G, I, K
        // Right Stalks: B, D, F, H, J, L

        // Doing arithmetic on a character means doing arithmetic
        // on it's ascii value (i.e. 'A' == 65, 'B' == 66, etc.)
        char letter = this.getLabel();
        return ((letter-'A') % 2) == 0;
    }

    /** The letter (A through L) given to this stalk in the game manual. */
    protected char getLabel() {
        // name = STALK_A, STALK_B, etc.
        return name.charAt(name.length()-1);
    }

    /** The reef face that this stalk is on. */
    public ReefFace getFace() {
        return reefFace;
    }

    /** An array of this stalk's branches in the following order: {@code[level2, level3, level4]} */
    public ReefBranch[] getBranches() {
        return branches;
    }

    /** The branch on this stalk at the given level, using the convention
     *  set in the manual that Level 1 is the trough, and the actual branches
     *  are on Levels 2 through 4. */
    public ReefBranch getBranch(int branchLevel) {
        return branches[branchLevel - 2];
    }
}
