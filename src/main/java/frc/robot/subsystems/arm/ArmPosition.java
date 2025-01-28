package frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmPosition {

    public double shoulderAngleDegrees;
    public double wristAngleDegrees;
    public double extensionMeters;

    /**
        * Creates a new ArmPosition object, representing the position of the arm at a given point.
        * 
        * @param shoulderAngleDegrees - Angle of the shoulder pivot of the arm. An angle of zero represents the arm pointing straight forward,
        *  with an upwards rotation begin positive.
        * @param wristAngleDegrees - Angle of the wrist pivot of the arm. An angle of zero represents the end effector being parallel with the ground,
        *  with an upwards rotation being positive.
        * @param extensionMeters - Total length of the arm, from the point on the arm aligning with the shoulder pivot center, to the end effector pivot center.
        */
    public ArmPosition(double shoulderAngleDegrees, double wristAngleDegrees, double extensionMeters) {
        this.shoulderAngleDegrees=shoulderAngleDegrees;
        this.wristAngleDegrees=wristAngleDegrees;
        this.extensionMeters=extensionMeters;    
    }


    /**
        * Returns a pose2d object representing the position of the end effector. The translation component represents
        * the position of the end effector pulley relative to the robot center, while the angle component represents the angle of the wrist.
        */
    public Pose2d getEndEffectorPose() {

        Translation2d armTranslationMeters = new Translation2d(extensionMeters, -ArmConstants.shoulderBracketLengthMeters);

        return new Pose2d(
            armTranslationMeters.rotateBy(Rotation2d.fromDegrees(shoulderAngleDegrees)).plus(ArmConstants.shoulderPivotPositionMeters),
            Rotation2d.fromDegrees(wristAngleDegrees) 
        );
    }
    

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
