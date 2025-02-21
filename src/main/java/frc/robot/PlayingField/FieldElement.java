package frc.robot.PlayingField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public interface FieldElement {
    // Because there are many different things that could be considered an indiviudal field element of
    // interest this year (e.g. reef faces vs individual stalks vs individual branches) I opted to
    // make FieldElement an interface which is then implemented by several similar but distinct classes,
    // rather than having a single FieldElement type like last year.
    // This increases our type safety in functions where we expect to be working with a specific type of field element,
    // and also leaves open the possility for unique methods for each variant/subtype
    // (e.g. asking a ReefFace what branches it has, or asking a Stalk which ReefFace its on).
    //
    // Design Note: I originally wanted to make FieldElement an enum that was extended by other enum subtypes, but such a design isn't possible
    //              in java because each enum implicitly extends an Enum class, and java doesn't support multiple inheritance
    //              (e.g. if ReefFace and FieldElement were enums, then ReefFace couldn't extend FieldElement because ReefFace already extends Enum,
    //                    and java's lack of multiple inheritance means that each class can't extend more than one other class).
    //
    //              This restriction led me to make FieldElement an interface, which was then implemented by all the subtypes (which were still enums themselves).
    //              However, this made it difficult to properly set up the links between faces, stalks, and branches because each set of
    //              enum instances had to be fully constructed before they could be refrenced elsewhere, which led to some unexpected
    //              null pointer issues if you weren't careful with the initialization order:
    //              https://stackoverflow.com/questions/1506410/java-enums-two-enum-types-each-containing-references-to-each-other
    //              https://stackoverflow.com/questions/14850842/overriding-abstract-method-or-using-one-single-method-in-enums
    //              
    //              The interface+enums approach also conflicted a bit with my desire to have FieldElement still
    //              emulate the feel of an enum. In particular, I preferred the simple flat structure of:
    //
    //              FieldElement.PROCESSOR                                     FieldElement.NonReefElement.PROCESSOR
    //              FieldElement.FRONT_LEFT_REEF_FACE                          FieldElement.ReefFace.FRONT_LEFT
    //              FieldElement.STALK_A                     rather than       FieldElement.ReefStalk.A
    //              FieldElement.BRANCH_B4                                     FieldElement.ReefBranch.B4
    //              FieldElement.HANDHELD_TAG                                  FieldElement.MoveableFieldElement.HANDHELD_TAG
    //
    //              While I could've achieved the flat structure by having a reference to each enum instance
    //              directly in FieldElement, that would give two different ways of accesing each field element
    //              (i.e. you could use the left or right approach for any given row above), and I much preferred
    //              the simplicity of only having one way to do things (zen of python and whatnot).
    //
    //              This led to the current structure of a FieldElement interface that holds all the instances
    //              of several implementing classes. While I'm still not super pleased with this approach because
    //              of how spread out the implementation is, and how much enum functionality I have to re-implement
    //              myself (like variable names and a list of all values) it at least offers external users the API
    //              I was going for. Because of that, and for the sake of time, I'm going to leave it like this for now.
    //              I feel like the real solution to this problem is to just use language features that
    //              aren't available in java, like Rust style enums or Go/Typescript style structural typing.
    //              


    // ******** Start Interface Definition ********
    
    public Pose3d getPose();

    public int getTagID();

    public String getName();

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

    // ******** End Interface Definition ********


    // Because the following variables are all members of an interface,
    // they're implicitly public, static, and final.


    // Loading stations are named from the driver's perspective
    //
    //           |                              |
    //           |             Field            |
    //           \                              /
    //       LEFT \                            / RIGHT
    //             \-----Alliance Station-----/
    //
    FieldElement LEFT_LOADING_STATION  = new StandardFieldElement(1, 13, "LEFT_LOADING_STATION");
    FieldElement RIGHT_LOADING_STATION = new StandardFieldElement(2, 12, "RIGHT_LOADING_STATION");
    FieldElement PROCESSOR             = new StandardFieldElement(16, 3, "PROCESSOR");
    FieldElement BARGE                 = new StandardFieldElement(5, 14, "BARGE");
    FieldElement ENEMY_BARGE           = new StandardFieldElement(4, 15, "ENEMY_BARGE");

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
    ReefFace FRONT_REEF_FACE       = new ReefFace(7, 18, "FRONT_REEF_FACE", 'A', 'B');
    ReefFace FRONT_RIGHT_REEF_FACE = new ReefFace(8, 17, "FRONT_RIGHT_REEF_FACE", 'C', 'D');
    ReefFace BACK_RIGHT_REEF_FACE  = new ReefFace(9, 22, "BACK_RIGHT_REEF_FACE", 'E', 'F');
    ReefFace BACK_REEF_FACE        = new ReefFace(10, 21, "BACK_REEF_FACE", 'G', 'H');
    ReefFace BACK_LEFT_REEF_FACE   = new ReefFace(11, 20, "BACK_LEFT_REEF_FACE", 'I', 'J');
    ReefFace FRONT_LEFT_REEF_FACE  = new ReefFace(6, 19, "FRONT_LEFT_REEF_FACE", 'K', 'L');
    ReefFace[] ALL_REEF_FACES = {FRONT_REEF_FACE, FRONT_RIGHT_REEF_FACE, BACK_RIGHT_REEF_FACE,
                                 BACK_REEF_FACE, BACK_LEFT_REEF_FACE, FRONT_LEFT_REEF_FACE};


    // Reef stalks are named according to the game manual
    //
    //                      _H__G_
    //                   I /      \ F
    //                  J /        \ E
    //                  K \        / D
    //                   L \______/ C
    //                       A  B
    //
    //            -----Alliance Station-----
    //
    // These names were chosen because the manual already had a convention,
    // not because they're terribly intuitive / easy to remember.
    // That's ok though, because it should be rare that we have to refer
    // to a specific stalk by name. Rather, I imagine we'll mostly just
    // access these varibles through their associated reef faces, or via
    // a getClosestStalk() function that's used when we go to score.
    //
    //
    // When calling getLeftStalk() or getRightStalk() on a ReefFace,
    // "Left" and "Right" are determined from the perspective where you're
    // viewing the reef face head on from outside the reef
    // (i.e. when you look straight ahead, you see the face's apriltag).
    //
    //                      _R__L_
    //                   L /      \ R
    //                  R /        \ L
    //                  L \        / R
    //                   R \______/ L
    //                       L  R
    //
    //            -----Alliance Station-----
    //
    ReefStalk STALK_A = FRONT_REEF_FACE.getLeftStalk();
    ReefStalk STALK_B = FRONT_REEF_FACE.getRightStalk();
    ReefStalk STALK_C = FRONT_RIGHT_REEF_FACE.getLeftStalk();
    ReefStalk STALK_D = FRONT_RIGHT_REEF_FACE.getRightStalk();
    ReefStalk STALK_E = BACK_RIGHT_REEF_FACE.getLeftStalk();
    ReefStalk STALK_F = BACK_RIGHT_REEF_FACE.getRightStalk();
    ReefStalk STALK_G = BACK_REEF_FACE.getLeftStalk();
    ReefStalk STALK_H = BACK_REEF_FACE.getRightStalk();
    ReefStalk STALK_I = BACK_LEFT_REEF_FACE.getLeftStalk();
    ReefStalk STALK_J = BACK_LEFT_REEF_FACE.getRightStalk();
    ReefStalk STALK_K = FRONT_LEFT_REEF_FACE.getLeftStalk();
    ReefStalk STALK_L = FRONT_LEFT_REEF_FACE.getRightStalk();
    ReefStalk[] ALL_STALKS = {STALK_A, STALK_B, STALK_C, STALK_D, STALK_E, STALK_F,
                              STALK_G, STALK_H, STALK_I, STALK_J, STALK_K, STALK_L};



    // Branches are named according to their stalk and level.
    // The game manual uses "level 1" to refer to the trough,
    // making the actual branches reside at levels 2, 3, and 4.
    ReefBranch BRANCH_A2 = STALK_A.getBranch(2), BRANCH_A3 = STALK_A.getBranch(3), BRANCH_A4 = STALK_A.getBranch(4);
    ReefBranch BRANCH_B2 = STALK_B.getBranch(2), BRANCH_B3 = STALK_B.getBranch(3), BRANCH_B4 = STALK_B.getBranch(4);
    ReefBranch BRANCH_C2 = STALK_C.getBranch(2), BRANCH_C3 = STALK_C.getBranch(3), BRANCH_C4 = STALK_C.getBranch(4);
    ReefBranch BRANCH_D2 = STALK_D.getBranch(2), BRANCH_D3 = STALK_D.getBranch(3), BRANCH_D4 = STALK_D.getBranch(4);
    ReefBranch BRANCH_E2 = STALK_E.getBranch(2), BRANCH_E3 = STALK_E.getBranch(3), BRANCH_E4 = STALK_E.getBranch(4);
    ReefBranch BRANCH_F2 = STALK_F.getBranch(2), BRANCH_F3 = STALK_F.getBranch(3), BRANCH_F4 = STALK_F.getBranch(4);
    ReefBranch BRANCH_G2 = STALK_G.getBranch(2), BRANCH_G3 = STALK_G.getBranch(3), BRANCH_G4 = STALK_G.getBranch(4);
    ReefBranch BRANCH_H2 = STALK_H.getBranch(2), BRANCH_H3 = STALK_H.getBranch(3), BRANCH_H4 = STALK_H.getBranch(4);
    ReefBranch BRANCH_I2 = STALK_I.getBranch(2), BRANCH_I3 = STALK_I.getBranch(3), BRANCH_I4 = STALK_I.getBranch(4);
    ReefBranch BRANCH_J2 = STALK_J.getBranch(2), BRANCH_J3 = STALK_J.getBranch(3), BRANCH_J4 = STALK_J.getBranch(4);
    ReefBranch BRANCH_K2 = STALK_K.getBranch(2), BRANCH_K3 = STALK_K.getBranch(3), BRANCH_K4 = STALK_K.getBranch(4);
    ReefBranch BRANCH_L2 = STALK_L.getBranch(2), BRANCH_L3 = STALK_L.getBranch(3), BRANCH_L4 = STALK_L.getBranch(4);
    ReefBranch[] ALL_BRANCHES = {BRANCH_A2, BRANCH_A3, BRANCH_A4, BRANCH_B2, BRANCH_B3, BRANCH_B4,
                                 BRANCH_C2, BRANCH_C3, BRANCH_C4, BRANCH_D2, BRANCH_D3, BRANCH_D4,
                                 BRANCH_E2, BRANCH_E3, BRANCH_E4, BRANCH_F2, BRANCH_F3, BRANCH_F4,
                                 BRANCH_G2, BRANCH_G3, BRANCH_G4, BRANCH_H2, BRANCH_H3, BRANCH_H4,
                                 BRANCH_I2, BRANCH_I3, BRANCH_I4, BRANCH_J2, BRANCH_J3, BRANCH_J4,
                                 BRANCH_K2, BRANCH_K3, BRANCH_K4, BRANCH_L2, BRANCH_L3, BRANCH_L4};



    /** A special field element that's used for demos or practice without a full field */
    MoveableFieldElement HANDHELD_TAG = new MoveableFieldElement("HANDHELD_TAG");
}
