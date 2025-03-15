package frc.robot.PlayingField;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.FlyingCircuitUtils;

public class ReefFace implements FieldElement {
    // A ReefFace has everything in a StandardFieldElement,
    // as well as the ability to get its associated
    // ReefStalks and ReefBranches

    protected final Pose3d redPose;
    protected final Pose3d bluePose;
    protected final int redTagID;
    protected final int blueTagID;
    protected final String name;

    protected final ReefStalk leftStalk;
    protected final ReefStalk rightStalk;

    protected final Boolean isAlgaeInL3;
    
    protected ReefFace(int redTagID, int blueTagID, String name, char leftStalkLabel, char rightStalkLabel, Boolean isAlgaeInL3) {
        this.redPose = FieldConstants.tagLayout.getTagPose(redTagID).get();
        this.bluePose = FieldConstants.tagLayout.getTagPose(blueTagID).get();
        this.redTagID = redTagID;
        this.blueTagID = blueTagID;
        this.name = name;
        this.isAlgaeInL3=isAlgaeInL3;

        leftStalk = new ReefStalk(this, leftStalkLabel);
        rightStalk = new ReefStalk(this, rightStalkLabel);
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

    /** The stalks on this reef face in the following order: {@code [left, right]} */
    public ReefStalk[] getStalks() {
        return new ReefStalk[] {leftStalk, rightStalk};
    }

    public ReefStalk getLeftStalk() {
        return leftStalk;
    }

    public ReefStalk getRightStalk() {
        return rightStalk;
    }

    /** The branches on this reef face in the following order:
     *  {@code [left2, left3, left4, right2, right3, right4]} */
    public ReefBranch[] getBranches() {
        return new ReefBranch[] {leftStalk.getBranch(2), leftStalk.getBranch(3), leftStalk.getBranch(4),
                                 rightStalk.getBranch(2), rightStalk.getBranch(3), rightStalk.getBranch(4)};
    }

    /** The branches on this reef face at the given level, using the convention
     *  set in the manual that Level 1 is the trough, and the actual branches
     *  are at Levels 2 through 4. */
    public ReefBranch[] getBranches(int branchLevel) {
        return new ReefBranch[] {leftStalk.getBranch(branchLevel), rightStalk.getBranch(branchLevel)};
    }

    public Boolean isHighAlgae() {
        return isAlgaeInL3;
    }

    // public static List<ReefFace> getAll() {
    //     return allReefFaces;
    // }
    //
    // I originally wanted to ease the burden of reimplementing the equivalent of Enum.values()
    // by having
    //
    //     public static final List<ReefFace> allReefFaces = new ArrayList<>();
    //
    // at the top of the class, and then adding instances to it as they were created in the constructor
    // like this:
    //
    //     allReefFaces.add(this);
    //
    // However, this led to null pointer exceptions when calling ReefFace.getAll() before accessing
    // any individual items from FieldElement.java!
    //     1) Calling ReefFace.getAll() starts the static initalization of ReefFace.java
    //     2) Java sees that ReefFace.java implements an interface with default methods (FieldElement.java)
    //        so it pauses initalization of ReefFace.java until FieldElement.java can run its own static init blocks
    //        (akin to how a superclass's constructor must be called before a subclasses constructor).
    //     3) FieldElement.java starts to construct one of its static members, which happens to be a ReefFace.
    //     4) Java sees that the static initialzation of ReefFace is still "in progress" at this point.
    //        To prevent endless recursive initialization (ReefFace needs FieldElement, which needs ReefFace, which needs FieldElement...),
    //        Java just pushes forward with the ReefFace constructor hoping that it just doesn't require access to any
    //        of ReefFace's static members!
    //     5) All is well until we attempt to add an element to the "allReefFaces" list. 
    //        Because the initialization of ReefFace is still on hold at this point, none of its
    //        static members are ready to be used, including the allReefFaces list.
    //        The result is a null pointer exception when executing "allReefFaces.add(this)",
    //        because "allReefFaces" is null.
    //
    //        See this link for another description of this process if what I said isn't making sense:
    //        https://stackoverflow.com/questions/31508001/creating-an-object-of-a-class-in-its-own-static-initializer
    //
    // While there are a few ways this can be fixed
    //
    //     1) Don't make the list final so we can put
    //
    //            if (allReefFaces == null) {
    //                allReefFaces = new ArrayList<>();
    //            }
    //         
    //        in the ReefFace constructor.
    //
    //     2) Don't let FieldElement have any default methods, and just
    //        copy and paste the implementation across the different subtypes.
    //        (though if ReefFace.getAll() was called before FieldElement init,
    //         it would just return an empty list, so this approach would also
    //         require other changes...)
    //
    //     3) Hold all FieldElement instances in FieldConstants instead
    //        (i.e. FieldElement target = FieldConstants.PROCESSOR),
    //        Which should remove the circular dependency between FieldElement.java and ReefFace.java
    //        (this one is probably my favorite, but I'm still not sold on it...).
    //
    // I ultimately decided to just hardcode the lists in the FieldElement class.
    // While I'm obviously not super happy with this solution, I think it should
    // ultimately be good enough. Almost by definition, we never expect the lists
    // of field elements to change, and this also fits well with my desire to have
    // FieldElement feel as enum-like as possible. Now the only time you should have
    // to type the name of a subtype is when you're declaring a variable of that specific
    // subtype. Accessing all instances (or lists of instances) will go through FieldElement.
}
