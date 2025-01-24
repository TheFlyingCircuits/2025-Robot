package frc.robot.Reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FlyingCircuitUtils;

public interface FieldElement {
    // Because there are many different things that could be considered an indiviudal field element of
    // interest this year (e.g. reef faces vs individual stalks vs individual branches) I opted to
    // make FieldElement an interface which is then implemented by several similar but distinct
    // enum classes, rather than just stuffing everything into one giant enum.
    // This lets us keep things a bit more organized, and also leaves open the possility for
    // unique methods for each subtype (e.g. asking a ReefFace what branches it has, or asing a Stalk which ReefFace its on.)
    // However, this comes at the cost of some mild code duplication, because interfaces can't have member variables.
    // One solution could be to use an abstract class instead of an interface, but then the subclasses couldn't be enums
    // because enums in java implicitly extend an Enum class, and java doesn't have multiple inheritance.
    
    public Pose3d getPose();

    public default Translation3d getLocation() {
        return this.getPose().getTranslation();
    }

    public default Rotation3d getOrientation() {
        return this.getPose().getRotation();
    }

    public default Pose2d getPose2d() {
        return this.getPose().toPose2d();
    }

    public default Translation2d getLocation2d() {
        return this.getPose2d().getTranslation();
    }

    public default Rotation2d getOrientation2d() {
        return this.getPose2d().getRotation();
    }

    public int getTagID();



    public enum ReefFace implements FieldElement {
        // Reef faces are named from the driver's perspective:
        //
        //                    __BACK___
        //                   /         \
        //        BACK_LEFT /           \ BACK_RIGHT
        //                 /             \
        //                 \             / 
        //       FRONT_LEFT \           / FRONT_RIGHT
        //                   \_________/ 
        //                      FRONT
        //
        //           -----Alliance Station-----
        //
        FRONT(7, 18), FRONT_LEFT(6, 19), FRONT_RIGHT(8, 17),
        BACK(10, 21), BACK_LEFT(11, 20), BACK_RIGHT(9, 22);

        private final Pose3d redPose;
        private final Pose3d bluePose;
        private final int redTagID;
        private final int blueTagID;

        // The Stalk and Branch arrays are linked up
        // in the Stalk and Branch constructors,
        // because those enums wont have their instances initialized
        // until after all the ReefFaces are constructed.

        /** The stalks on this reef face in the following order: {@code [left, right]}
         *  (where left and right are determined from the robot's perspective as it approaches the reef face). */
        public final ReefStalk[] stalks = new ReefStalk[2];

        /** The branches on this reef face in the following order: {@code [left2, left3, left4, right2, right3, right4]}
         *  (where left and right are determined from the robot's perspective as it approaches the reef face). */
        private final ReefBranch[] branches = new ReefBranch[6];
        

        private ReefFace(int redTagID, int blueTagID) {
            this.redPose = FieldConstants.tagLayout.getTagPose(redTagID).get();
            this.bluePose = FieldConstants.tagLayout.getTagPose(blueTagID).get();
            this.redTagID = redTagID;
            this.blueTagID = blueTagID;
        }

        public Pose3d getPose() {
            return FlyingCircuitUtils.getAllianceDependentValue(redPose, bluePose, new Pose3d());
        }

        public int getTagID() {
            return FlyingCircuitUtils.getAllianceDependentValue(redTagID, blueTagID, -1);
        }

        /** Left and Right are determined from the robot's perspective as it approaches the reef face */
        public ReefStalk getLeftStalk() {
            return stalks[0];
        }

        /** Left and Right are determined from the robot's perspective as it approaches the reef face */
        public ReefStalk getRightStalk() {
            return stalks[1];
        }
    }



    public enum ReefStalk implements FieldElement {
        // Reef stalks are named according to the game manual (Version 4, page 24)
        //
        //        _H__G_       
        //     I /      \ F
        //    J /        \ E
        //    K \        / D
        //     L \______/ C
        //         A  B
        //
        //  --Alliance Station--
        //
        // Note that these names were chosen just because the manual already had a convention,
        // and I imagine we won't really ever need to refer to the stalks by name.
        // Rather, I imagine these will just all be in a list which we'll use to find the closest
        // stalk to align to when we go to score.
        A(ReefFace.FRONT), B(ReefFace.FRONT),
        C(ReefFace.FRONT_RIGHT), D(ReefFace.FRONT_RIGHT),
        E(ReefFace.BACK_RIGHT), F(ReefFace.BACK_RIGHT),
        G(ReefFace.BACK), H(ReefFace.BACK),
        I(ReefFace.BACK_LEFT), J(ReefFace.BACK_LEFT),
        K(ReefFace.FRONT_LEFT), L(ReefFace.FRONT_LEFT);

        private final Pose3d redPose;
        private final Pose3d bluePose;
        private final int redTagID;
        private final int blueTagID;

        /** The reef face that this stalk is on. */
        public final ReefFace reefFace;

        /** An array of this stalk's branches in the following order: {@code[level2, level3, level4]} */
        public final ReefBranch[] branches = new ReefBranch[3];

        private ReefStalk(ReefFace reefFace) {
            this.redTagID = reefFace.redTagID;
            this.blueTagID = reefFace.blueTagID;
            Pose3d redTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(redTagID).get();
            Pose3d blueTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(blueTagID).get();

            // Stalks on the left from the robot's perspective
            // have negative y coordinates in the tag's frame.
            double stalkY_tagFrame = FieldConstants.stalkSeparationMeters / 2.0;;
            if (this.isLeftStalk()) {
                stalkY_tagFrame *= -1;
            }

            // For the sake of visualization, I've placed the stalks "location"
            // so that it's aligned with the hole in the reef that the stalk sticks out of.
            // it's location is half way up the maximum height of the stalk, so it's right in the middle.
            double stalkX_tagFrame = -FieldConstants.stalkInsetMeters;
            double stalkHeightMeters = FieldConstants.branchHeightMeters[4];
            double stalkZ_tagFrame = (stalkHeightMeters / 2.0) - redTagPose_fieldFrame.getZ();

            Translation3d stalkLocation_tagFrame = new Translation3d(stalkX_tagFrame, stalkY_tagFrame, stalkZ_tagFrame);
            Transform3d stalkPose_tagFrame = new Transform3d(stalkLocation_tagFrame, new Rotation3d());

            this.redPose = redTagPose_fieldFrame.transformBy(stalkPose_tagFrame);
            this.bluePose = blueTagPose_fieldFrame.transformBy(stalkPose_tagFrame);

            // link up this stalk to its associated reefFace
            this.reefFace = reefFace;
            if (this.isLeftStalk()) {
                reefFace.stalks[0] = this;
            }
            else {
                reefFace.stalks[1] = this;
            }
        }

        public Pose3d getPose() {
            return FlyingCircuitUtils.getAllianceDependentValue(redPose, bluePose, new Pose3d());
        }

        public int getTagID() {
            return FlyingCircuitUtils.getAllianceDependentValue(redTagID, blueTagID, -1);
        }

        public ReefFace getReefFace() {
            return reefFace;
        }

        public ReefBranch[] getBranches() {
            return branches;
        }

        private boolean isLeftStalk() {
            // left and right is determined from the robot's perspecive
            // as it goes up to score on the stalk.
            //
            // Left Stalks: A, C, E, G, I, K
            // Right Stalks: B, D, F, H, J, L

            // doing arithmetic on a character means doing arithmetic
            // on it's ascii value (i.e. 'A' == 65, 'B' == 66, etc.)
            return ((this.name().charAt(0)-'A') % 2) == 0;
        }
    }



    public enum ReefBranch implements FieldElement {
        A2(ReefStalk.A), A3(ReefStalk.A), A4(ReefStalk.A),
        B2(ReefStalk.B), B3(ReefStalk.B), B4(ReefStalk.B),
        C2(ReefStalk.C), C3(ReefStalk.C), C4(ReefStalk.C),
        D2(ReefStalk.D), D3(ReefStalk.D), D4(ReefStalk.D),
        E2(ReefStalk.E), E3(ReefStalk.E), E4(ReefStalk.E),
        F2(ReefStalk.F), F3(ReefStalk.F), F4(ReefStalk.F),
        G2(ReefStalk.G), G3(ReefStalk.G), G4(ReefStalk.G),
        H2(ReefStalk.H), H3(ReefStalk.H), H4(ReefStalk.H),
        I2(ReefStalk.I), I3(ReefStalk.I), I4(ReefStalk.I),
        J2(ReefStalk.J), J3(ReefStalk.J), J4(ReefStalk.J),
        K2(ReefStalk.K), K3(ReefStalk.K), K4(ReefStalk.K),
        L2(ReefStalk.L), L3(ReefStalk.L), L4(ReefStalk.L);


        private final Pose3d redPose;
        private final Pose3d bluePose;
        private final int redTagID;
        private final int blueTagID;

        public final ReefFace reefFace;
        public final ReefStalk stalk;
        public final int branchLevel;

        private ReefBranch(ReefStalk stalk) {
            this.redTagID = stalk.redTagID;
            this.blueTagID = stalk.blueTagID;
            Pose3d redTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(redTagID).get();
            Pose3d blueTagPose_fieldFrame = FieldConstants.tagLayout.getTagPose(blueTagID).get();

            // Stalks on the left from the robot's perspective
            // have negative y coordinates in the tag's frame.
            double branchY_tagFrame = FieldConstants.stalkSeparationMeters / 2.0;
            if (stalk.isLeftStalk()) {
                branchY_tagFrame *= -1;
            }

            // A branch's "locaiton" is at the branch's extremeties:
            // its highest point off the ground, and its furthest point out from the reef.
            this.branchLevel = Integer.parseInt(this.name().charAt(1)+"");
            double branchX_tagFrame = -FieldConstants.branchInsetMeters[branchLevel];
            double branchZ_tagFrame = FieldConstants.branchHeightMeters[branchLevel] - redTagPose_fieldFrame.getZ();

            Translation3d branchLocation_tagFrame = new Translation3d(branchX_tagFrame, branchY_tagFrame, branchZ_tagFrame);
            Rotation3d branchOrientation_tagFrame = new Rotation3d(0, -FieldConstants.branchPitchRadians[branchLevel], 0);
            Transform3d branchPose_tagFrame = new Transform3d(branchLocation_tagFrame, branchOrientation_tagFrame);

            this.redPose = redTagPose_fieldFrame.transformBy(branchPose_tagFrame);
            this.bluePose = blueTagPose_fieldFrame.transformBy(branchPose_tagFrame);

            // link up this branch to its stalk and reef face
            this.stalk = stalk;
            this.reefFace = stalk.reefFace;
            stalk.branches[branchLevel-2] = this;
            if (stalk.isLeftStalk()) {
                // branches[] = {L2, L3, L4, R2, R3, R4}
                //   index:       0,  1,  2
                reefFace.branches[branchLevel-2] = this;
            }
            else {
                // branches[] = {L2, L3, L4, R2, R3, R4}
                //   index:                   3,  4,  5
                reefFace.branches[branchLevel+1] = this;
            }
        }

        public Pose3d getPose() {
            return FlyingCircuitUtils.getAllianceDependentValue(redPose, bluePose, new Pose3d());
        }

        public int getTagID() {
            return FlyingCircuitUtils.getAllianceDependentValue(redTagID, blueTagID, -1);
        }
    }



    public enum NonReefElement implements FieldElement {
        // Loading stations are named from driver perspective
        LEFT_LOADING_STATION(1, 13), RIGHT_LOADING_STATION(2, 12),
        PROCESSOR(16, 3), BARGE(5, 14), ENEMY_BARGE(4, 15);

        private Pose3d redPose;
        private Pose3d bluePose;
        private int redTagID;
        private int blueTagID;

        private NonReefElement(int redTagID, int blueTagID) {
            this.redPose = FieldConstants.tagLayout.getTagPose(redTagID).get();
            this.bluePose = FieldConstants.tagLayout.getTagPose(blueTagID).get();
            this.redTagID = redTagID;
            this.blueTagID = blueTagID;
        }

        public Pose3d getPose() {
            return FlyingCircuitUtils.getAllianceDependentValue(redPose, bluePose, new Pose3d());
        }

        public int getTagID() {
            return FlyingCircuitUtils.getAllianceDependentValue(redTagID, blueTagID, -1);
        }
    }
}
