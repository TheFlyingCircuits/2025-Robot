package frc.robot.PlayingField;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FlyingCircuitUtils;

public class ReefBranch implements FieldElement {
    // A ReefBranch has everything in a StandardFieldElement,
    // as well as the ability to get its associated
    // ReefStalk and ReefFace

    protected final Pose3d redPose;
    protected final Pose3d bluePose;
    protected final int redTagID;
    protected final int blueTagID;
    protected final String name;

    protected final ReefStalk stalk;
    protected final int branchLevel;

    public boolean scoredOn = false;


    protected ReefBranch(ReefStalk stalk, int branchLevel) {
        this.name = "BRANCH_"+stalk.getLabel()+""+branchLevel;
        this.branchLevel = branchLevel;
        this.redTagID = stalk.redTagID;
        this.blueTagID = stalk.blueTagID;
        Pose3d redTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(redTagID).get();
        Pose3d blueTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(blueTagID).get();

        Logger.recordOutput("branches/" + stalk.getFace().name + stalk.name + branchLevel, scoredOn);

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
        double branchY_tagFrame = FieldConstants.stalkSeparationMeters / 2.0;
        if (stalk.isLeftStalk()) {
            branchY_tagFrame *= -1;
        }

        // // A branch's "location" is at the branch's extremeties:
        // // its highest point off the ground, and its furthest point out from the reef.
        // double branchX_tagFrame = -FieldConstants.branchInsetMeters[branchLevel];
        // double branchZ_tagFrame = FieldConstants.branchHeightMeters[branchLevel] - redTagPose_fieldFrame.getZ(); // subtract tag height cause we're in the tag frame.

        // A branch's "location" is centered at the tip of the branch.
        //
        // vert = vertical
        // hort = horizontal
        //
        //             |\
        //           / | \
        //         /   |  \
        //       /     |   \
        //     /       |    \ <- branch location
        //   /    vert |     \
        // /           |      \
        //             |       \
        //             |        \
        //             ---------/
        //               hort /
        //                  /
        //                /
        //              /
        //            /
        //          /
        //        /
        //      /
        //    /
        //  /
        double vert = FieldConstants.branchDiameterMeters * Math.cos(FieldConstants.branchPitchRadians[branchLevel]);
        double hort = FieldConstants.branchDiameterMeters * Math.sin(FieldConstants.branchPitchRadians[branchLevel]);
        double branchX_tagFrame = -( FieldConstants.branchInsetMeters[branchLevel] + (hort / 2.0) );
        double branchZ_tagFrame = FieldConstants.branchHeightMeters[branchLevel] - (vert / 2.0) - redTagPose_fieldFrame.getZ(); // subtract tag height cause we're in the tag frame, not the floor frame.

        Translation3d branchLocation_tagFrame = new Translation3d(branchX_tagFrame, branchY_tagFrame, branchZ_tagFrame);
        Rotation3d branchOrientation_tagFrame = new Rotation3d(0, -FieldConstants.branchPitchRadians[branchLevel], 0); // the branchPitch from FieldConstants is unsigned, and needs to be negated to obey the right-hand-rule.
        Transform3d branchPose_tagFrame = new Transform3d(branchLocation_tagFrame, branchOrientation_tagFrame);

        this.redPose = redTagPose_fieldFrame.transformBy(branchPose_tagFrame);
        this.bluePose = blueTagPose_fieldFrame.transformBy(branchPose_tagFrame);

        // link up this branch to its associated stalk
        this.stalk = stalk;
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

    public int getLevel() {
        return branchLevel;
    }

    public ReefStalk getStalk() {
        return stalk;
    }

    public ReefFace getFace() {
        return stalk.getFace();
    }
}
