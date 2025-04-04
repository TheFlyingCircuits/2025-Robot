package frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmPosition {

    public double shoulderAngleDegrees;
    public double wristAngleDegrees;
    public double extensionMeters;

    //2 coral distance
    public static final ArmPosition frontL1 = new ArmPosition(14, WristConstants.maxAngleDegrees, ArmConstants.minExtensionMeters);

    //1 coral distance
    public static final ArmPosition frontL2 = new ArmPosition(39.37, 109.20, 0.71);

    //1 coral distance
    public static final ArmPosition frontL3 = new ArmPosition(58.0, 91.91, 0.92);

    //1 coral distance
    public static final ArmPosition frontL4 = new ArmPosition(72.59, 33.28, 1.6);


    //2 coral distance
    public static final ArmPosition frontL2_old = new ArmPosition(35.3, 98, 0.845);

    //2 coral distance
    public static final ArmPosition frontL3_old = new ArmPosition(52.2, 80, 1.08);

    //1 coral distance
    public static final ArmPosition frontL4_old = new ArmPosition(71, 35, 1.6);

    
    //1 coral distance
    public static final ArmPosition backL2 = new ArmPosition(95.45, 136.58, ArmConstants.minExtensionMeters);

    //1 coral distance
    public static final ArmPosition backL3 = new ArmPosition(96.85, 115.86, 0.82);
    
    //1 coral distance
    public static final ArmPosition backL4 = new ArmPosition(92.81, 134.95, ArmConstants.maxExtensionMeters);

    /**
        * Creates a new ArmPosition object, representing the position of the arm at a given point.
        * 
        * @param shoulderAngleDegrees - Angle of the shoulder pivot of the arm. An angle of zero represents the arm pointing straight forward,
        *  with an upwards rotation begin positive.
        * @param wristAngleDegrees - Angle of the wrist pivot of the arm. An angle of zero represents the end effector being parallel with the ground
        *  (intake position), with an upwards rotation being positive.
        * @param extensionMeters - Total length of the arm, from the point on the arm aligning with the shoulder pivot center, to the end effector pivot center.
        */
    public ArmPosition(double shoulderAngleDegrees, double wristAngleDegrees, double extensionMeters) {
        this.shoulderAngleDegrees=shoulderAngleDegrees;
        this.wristAngleDegrees=wristAngleDegrees;
        this.extensionMeters=extensionMeters;    
    }

    public ArmPosition() {
        this(ArmConstants.armMinAngleDegrees, WristConstants.maxAngleDegrees, ArmConstants.minExtensionMeters);
    }

    public static ArmPosition getPreset(int level, boolean facingReef) {
        if (facingReef) {
            if (level == 1) { return frontL1; }
            if (level == 2) { return frontL2; }
            if (level == 3) { return frontL3; }
            if (level == 4) { return frontL4; }
        }
        else {
            if (level == 2) { return backL2; }
            if (level == 3) { return backL3; }
            if (level == 4) { return backL4; }
        }

        // should never get here!
        return new ArmPosition();
    }

    public ArmPosition constrainedToLimits() {
        ArmPosition output = new ArmPosition();

        output.shoulderAngleDegrees = this.shoulderAngleDegrees;
        while (output.shoulderAngleDegrees > ArmConstants.armMaxAngleDegrees) {
            output.shoulderAngleDegrees -= 360.0;
        }
        while (output.shoulderAngleDegrees < ArmConstants.armMinAngleDegrees) {
            output.shoulderAngleDegrees += 360.0;
        }

        output.extensionMeters = this.extensionMeters;
        if (output.extensionMeters > ArmConstants.maxExtensionMeters) {
            output.extensionMeters = ArmConstants.maxExtensionMeters;
        }
        if (output.extensionMeters < ArmConstants.minExtensionMeters) {
            output.extensionMeters = ArmConstants.minExtensionMeters;
        }

        output.wristAngleDegrees = this.wristAngleDegrees;
        while (output.wristAngleDegrees > WristConstants.maxAngleDegrees) {
            output.wristAngleDegrees -= 360.0;
        }
        while (output.wristAngleDegrees < WristConstants.minAngleDegrees) {
            output.wristAngleDegrees += 360.0;
        }

        return output;
    }


    /**
        * Returns a pose2d object representing the position of the end effector. The translation component represents
        * the position of the end effector pivot relative to the robot center, while the angle component represents the angle of the wrist.
        */
    public Pose2d getEndEffectorPose() {

        Translation2d armTranslationMeters = new Translation2d(extensionMeters, -ArmConstants.shoulderBracketLengthMeters);

        return new Pose2d(
            armTranslationMeters.rotateBy(Rotation2d.fromDegrees(shoulderAngleDegrees)).plus(ArmConstants.shoulderPivotPositionMeters),
            Rotation2d.fromDegrees(wristAngleDegrees)
        );
    }
    

    /**
     * Creates an arm position object given a desired pose of the end effector relative to the center of the robot.
     * The translation component represent the position of the end effector pivot point relative to the robot center,
     * while the angle component represents the angle of the wrist.
     * @param endEffectorPose
     * @return
     */
    public static ArmPosition generateArmPosition(Pose2d endEffectorPose) {

        //make position relative to pivot
        endEffectorPose = endEffectorPose.transformBy(new Transform2d(ArmConstants.shoulderPivotPositionMeters, new Rotation2d()));

        //angle of the straight line pointing to the desired point
        double straightAngleRadians = Math.atan2(endEffectorPose.getY(), endEffectorPose.getX());

        //length of extension, calculate because hypotenuse formed by two legs of arm is equivalent to hypotenuse from pivot to point
        double extensionLengthMeters = Math.sqrt(Math.pow(endEffectorPose.getX(), 2)
            + Math.pow(endEffectorPose.getY(), 2)
            - Math.pow(ArmConstants.shoulderBracketLengthMeters, 2));

        //angle that just the triangle of the arm forms
        double angleOffsetRadians = Math.atan2(extensionLengthMeters, ArmConstants.shoulderBracketLengthMeters);

        //apply offset
        double shoulderAngleDegrees = Math.toDegrees(straightAngleRadians) - Math.toDegrees(angleOffsetRadians) + 90;


        return new ArmPosition(shoulderAngleDegrees, endEffectorPose.getRotation().getDegrees(), extensionLengthMeters);
    }
}
