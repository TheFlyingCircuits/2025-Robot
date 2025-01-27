package main.java.frc.robot.subsystems.arm;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class ArmUtils {
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
         * the position of the end effector pulley, while the angle component represents the angle of the wrist.
         */
        public Pose2d getEndEffectorPose() {
            return new Pose2d(
                ArmConstants.shoulderToWristMeters.rotateBy(Rotation2d.fromDegrees(shoulderAngleDegrees)),
                Math.toRadians(wristAngleDegrees)            
            );
        }
    }

    public static double generateArmPosition(Pose2d endEffectorPose) {
        //TODO: fill this in
    }
}
