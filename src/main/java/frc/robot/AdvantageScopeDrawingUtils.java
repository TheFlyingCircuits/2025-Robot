package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.arm.ArmPosition;

public class AdvantageScopeDrawingUtils {

    /**<pre>&nbsp;
     *       0------1
     *      /|     /|
     *     3-+----2 |
     *     | |    | |
     *   Z | 4----+-5
     *     |/     |/ X
     *     7------6   
     *        Y
     *</pre>
     */
    public static List<Translation3d> getBoxCorners(double lengthX, double lengthY, double lengthZ) {
        List<Translation3d> corners = new ArrayList<>(8);
        // top half
        corners.add(new Translation3d(lengthX, lengthY, lengthZ));
        corners.add(new Translation3d(lengthX,    0,    lengthZ));
        corners.add(new Translation3d(   0,       0,    lengthZ));
        corners.add(new Translation3d(   0,    lengthY, lengthZ));

        // bottom half
        corners.add(new Translation3d(lengthX, lengthY,    0   ));
        corners.add(new Translation3d(lengthX,    0,       0   ));
        corners.add(new Translation3d(   0,       0,       0   ));
        corners.add(new Translation3d(   0,    lengthY,    0   ));

        return corners;
    }
    // maybe better to take as input: frontX,backX,leftY,rightY,bottomZ,topZ, so box is already in the right place?

    public static List<Translation3d> getPencilTrajectoryFromBoxCorners(List<Translation3d> boxCorners) {
        List<Translation3d> trajectory = new ArrayList<>();

        //      0------1
        //     /|     /|
        //    3-+----2 |
        //    | |    | |
        //    | 4----+-5
        //    |/     |/ 
        //    7------6   

        // top half
        trajectory.add(boxCorners.get(0));
        trajectory.add(boxCorners.get(1));
        trajectory.add(boxCorners.get(2));
        trajectory.add(boxCorners.get(3));
        trajectory.add(boxCorners.get(0)); // close the loop

        // bottom half, returning back up at each corner for the vertical edges
        trajectory.add(boxCorners.get(4));

        trajectory.add(boxCorners.get(5)); // 5 <-> 1
        trajectory.add(boxCorners.get(1));
        trajectory.add(boxCorners.get(5));

        trajectory.add(boxCorners.get(6)); // 6 <-> 2
        trajectory.add(boxCorners.get(2));
        trajectory.add(boxCorners.get(6));

        trajectory.add(boxCorners.get(7)); // 7 <-> 3
        trajectory.add(boxCorners.get(3));
        trajectory.add(boxCorners.get(7));

        trajectory.add(boxCorners.get(4));
        trajectory.add(boxCorners.get(0)); // back home to finish the loop

        return trajectory;
    }


    private static List<Translation3d> getBumperBox_robotFrame() {
        double lengthX = DrivetrainConstants.bumperWidthMeters;
        double lengthY = DrivetrainConstants.bumperWidthMeters;
        double lengthZ = Units.inchesToMeters(5);
        List<Translation3d> boxCorners = getBoxCorners(lengthX, lengthY, lengthZ);
        List<Translation3d> output = new ArrayList<>();

        // center in x & y, bottom surface of bumpers is 0.5 inches off the ground
        Translation3d offset = new Translation3d(-lengthX/2.0, -lengthY/2.0, Units.inchesToMeters(0.5));
        for (Translation3d corner : boxCorners) {
            output.add(corner.plus(offset));
        }
        return output;
    }

    public static void drawBumpers(String nameInLog, Pose2d robotPose) {
        List<Translation3d> bumperVertices_robotFrame = getBumperBox_robotFrame();

        List<Translation3d> bumperVertices_fieldFrame = new ArrayList<>();
        Rotation3d robotOrientation_fieldFrame = new Rotation3d(robotPose.getRotation());
        Translation3d robotLocation_fieldFrame = new Translation3d(robotPose.getTranslation());

        for (Translation3d vertex : bumperVertices_robotFrame) {
            Translation3d vertexFieldFrame = vertex.rotateBy(robotOrientation_fieldFrame).plus(robotLocation_fieldFrame);
            bumperVertices_fieldFrame.add(vertexFieldFrame);
        }

        List<Translation3d> drawMe = getPencilTrajectoryFromBoxCorners(bumperVertices_fieldFrame);
        Logger.recordOutput(nameInLog, drawMe.toArray(new Translation3d[0]));
    }



    private static List<Translation3d> getBicepBox_shoulderFrame() {
        // TODO: ASCII art
        double boxLengthX = 2 * Math.abs(ArmConstants.retractionHardStop_elbowFrame.getX());
        double boxLengthY = ArmConstants.stationaryStageWidthMeters + Units.inchesToMeters(0.5);
        double boxLengthZ = Math.abs(ArmConstants.elbowLocation_shoulderFrame.getZ()) + (ArmConstants.stationaryStageHeightMeters / 2.0);

        List<Translation3d> box = getBoxCorners(boxLengthX, boxLengthY, boxLengthZ);
        // center in x and y, shift down by entire lengthZ so that top surface of box contains the shoulder
        Translation3d adjustment = new Translation3d(-boxLengthX/2.0, -boxLengthY/2.0, -boxLengthZ);
        for (int i = 0; i < box.size(); i += 1) {
            Translation3d unadjusted = box.get(i);
            box.set(i, unadjusted.plus(adjustment));
        }

        return box;
    }

    private static List<Translation3d> getExtensionBox_elbowFrame(double extensionLengthMeters) {
        double boxLengthX = extensionLengthMeters;
        double boxLengthY = ArmConstants.stationaryStageWidthMeters;
        double boxLengthZ = ArmConstants.stationaryStageHeightMeters;

        List<Translation3d> box = getBoxCorners(boxLengthX, boxLengthY, boxLengthZ);
        // center y and z, shift in x because extension is measured from hard stop, not from elbow origin
        // (x coordinate of retractionHardStop_elbowFrame is already negative)
        Translation3d adjustment = new Translation3d(ArmConstants.retractionHardStop_elbowFrame.getX(), -boxLengthY/2.0, -boxLengthZ/2.0);
        for (int i = 0; i < box.size(); i += 1) {
            Translation3d unadjusted = box.get(i);
            box.set(i, unadjusted.plus(adjustment));
        }

        return box;
    }

    private static List<Translation3d> getWristVertices_wristFrame() {
        //
        //  0 \
        //  |    \
        //  |       \
        //  2__________1
        //

        
        List<Translation3d> vertices = new ArrayList<>();

        // right side
        Translation3d rightOffset = new Translation3d(0, -ArmConstants.wristOuterWidthMeters/2.0, 0);
        vertices.add(new Translation3d(0, 0, 0).plus(rightOffset));
        vertices.add(ArmConstants.orangeWheels_wristFrame.plus(rightOffset));
        vertices.add(new Translation3d(0, 0, ArmConstants.orangeWheels_wristFrame.getZ()).plus(rightOffset));

        // left side
        Translation3d leftOffset = rightOffset.times(-1);
        vertices.add(new Translation3d(0, 0, 0).plus(leftOffset));
        vertices.add(ArmConstants.orangeWheels_wristFrame.plus(leftOffset));
        vertices.add(new Translation3d(0, 0, ArmConstants.orangeWheels_wristFrame.getZ()).plus(leftOffset));

        return vertices;
    }

    public static List<Translation3d> getPencilTrajectoryFromWristVertices(List<Translation3d> wristVertices) {
        //
        //  0 \
        //  |    \
        //  |       \
        //  2__________1
        //
        //
        //  3 \
        //  |    \
        //  |       \
        //  5__________4
        //

        List<Translation3d> output = new ArrayList<>();

        // right half
        output.add(wristVertices.get(0));
        output.add(wristVertices.get(1));
        output.add(wristVertices.get(2));
        output.add(wristVertices.get(0));

        // punch down to left, returning to right for connections along the way
        output.add(wristVertices.get(3));
        output.add(wristVertices.get(4));
        output.add(wristVertices.get(1));
        output.add(wristVertices.get(4));

        output.add(wristVertices.get(5));
        output.add(wristVertices.get(2));
        output.add(wristVertices.get(5));

        output.add(wristVertices.get(3));
        output.add(wristVertices.get(0)); // finish back home

        return output;
    }

    public static void logArmWireframe(String name, ArmPosition armPosition, Pose2d robotPose, boolean hasLeftCoral, boolean hasRightCoral) {
        // get poses of components
        double shoulderRads = Units.degreesToRadians(armPosition.shoulderAngleDegrees);
        double extensionMeters = armPosition.extensionMeters;
        double wristRads = Units.degreesToRadians(armPosition.wristAngleDegrees);


        // get robot->shoulder
        Rotation3d shoulderOrientation_robotFrame = new Rotation3d(0, -shoulderRads, 0); // wpilib convention, positive pitch is down.
        Transform3d shoulderPose_robotFrame = new Transform3d(ArmConstants.shoulderLocation_robotFrame, shoulderOrientation_robotFrame);

        // get robot->elbow
        Transform3d elbowPose_shoulderFrame = new Transform3d(ArmConstants.elbowLocation_shoulderFrame, Rotation3d.kZero);
        Transform3d elbowPose_robotFrame = shoulderPose_robotFrame.plus(elbowPose_shoulderFrame);

        // get robot->wrist
        double minExtension = ArmConstants.tipOfStationaryStage_elbowFrame.minus(ArmConstants.retractionHardStop_elbowFrame).getX();
        double deltaExtentsion = extensionMeters - minExtension;
        Translation3d wristLocation_elbowFrame = ArmConstants.wristLocation_elbowFrame_retracted.plus(new Translation3d(deltaExtentsion, 0, 0));
        Rotation3d wristOrientation_elbowFrame = new Rotation3d(0, -wristRads, 0); // wpilib convention, positive pitch is down.
        Transform3d wristPose_elbowFrame = new Transform3d(wristLocation_elbowFrame, wristOrientation_elbowFrame);
        Transform3d wristPose_robotFrame = elbowPose_robotFrame.plus(wristPose_elbowFrame);

        // components to field frame
        Transform3d robotPose_fieldFrame = new Transform3d(new Translation3d(robotPose.getTranslation()), new Rotation3d(robotPose.getRotation()));
        Transform3d shoulderPose_fieldFrame = robotPose_fieldFrame.plus(shoulderPose_robotFrame);
        Transform3d elbowPose_fieldFrame = robotPose_fieldFrame.plus(elbowPose_robotFrame);
        Transform3d wristPose_fieldFrame = robotPose_fieldFrame.plus(wristPose_robotFrame);

        // vertices for wireframe
        List<Translation3d> bicepBox_shoulderFrame = getBicepBox_shoulderFrame();
        List<Translation3d> extensionBox_elbowFrame = getExtensionBox_elbowFrame(extensionMeters);
        List<Translation3d> wristPrism_wristFrame = getWristVertices_wristFrame();

        List<Translation3d> bicepBox_fieldFrame = new ArrayList<>();
        for (Translation3d bicepVertex_shoulderFrame : bicepBox_shoulderFrame) {
            Translation3d bicepVertex_fieldFrame = bicepVertex_shoulderFrame.rotateBy(shoulderPose_fieldFrame.getRotation()).plus(shoulderPose_fieldFrame.getTranslation());
            bicepBox_fieldFrame.add(bicepVertex_fieldFrame);
        }

        List<Translation3d> extensionBox_fieldFrame = new ArrayList<>();
        for (Translation3d extensionVertex_elbowFrame : extensionBox_elbowFrame) {
            Translation3d extensionVertex_fieldFrame = extensionVertex_elbowFrame.rotateBy(elbowPose_fieldFrame.getRotation()).plus(elbowPose_fieldFrame.getTranslation());
            extensionBox_fieldFrame.add(extensionVertex_fieldFrame);
        }

        List<Translation3d> wristPrism_fieldFrame = new ArrayList<>();
        for (Translation3d wristVertex_wristFrame : wristPrism_wristFrame) {
            Translation3d wristVertex_fieldFrame = wristVertex_wristFrame.rotateBy(wristPose_fieldFrame.getRotation()).plus(wristPose_fieldFrame.getTranslation());
            wristPrism_fieldFrame.add(wristVertex_fieldFrame);
        }

        List<Translation3d> drawMeBicep = getPencilTrajectoryFromBoxCorners(bicepBox_fieldFrame);
        List<Translation3d> drawMeExtension = getPencilTrajectoryFromBoxCorners(extensionBox_fieldFrame);
        List<Translation3d> drawMeWrist = getPencilTrajectoryFromWristVertices(wristPrism_fieldFrame);

        Logger.recordOutput(name+"/bicep", drawMeBicep.toArray(new Translation3d[0]));
        Logger.recordOutput(name+"/extension", drawMeExtension.toArray(new Translation3d[0]));
        Logger.recordOutput(name+"/wrist", drawMeWrist.toArray(new Translation3d[0]));

        List<Pose3d> coralPoses_fieldFrame = new ArrayList<>();
        if (hasLeftCoral) {
            Transform3d leftCoralPose_wristFrame = new Transform3d(ArmConstants.intakeToLeftCoral, new Rotation3d());
            Transform3d leftCoralPose_fieldFrame = wristPose_fieldFrame.plus(leftCoralPose_wristFrame);
            coralPoses_fieldFrame.add(new Pose3d(leftCoralPose_fieldFrame.getTranslation(), leftCoralPose_fieldFrame.getRotation()));
        }
        if (hasRightCoral) {
            Transform3d rightCoralPose_wristFrame = new Transform3d(ArmConstants.intakeToRightCoral, new Rotation3d());
            Transform3d rightCoralPose_fieldFrame = wristPose_fieldFrame.plus(rightCoralPose_wristFrame);
            coralPoses_fieldFrame.add(new Pose3d(rightCoralPose_fieldFrame.getTranslation(), rightCoralPose_fieldFrame.getRotation()));
        }
        Logger.recordOutput(name+"/coral", coralPoses_fieldFrame.toArray(new Pose3d[0]));
    }

    public static void drawCircle(String name, Translation2d center, double radius) {
        // https://stackoverflow.com/questions/11774038/how-to-render-a-circle-with-as-few-vertices-as-possible
        double acceptableVisualDeviationMeters = 0.01;
        int numVertices = (int)Math.ceil((2*Math.PI) / Math.acos((2 * Math.pow(1 - (acceptableVisualDeviationMeters/radius), 2)) - 1));
        if (numVertices < 8) {
            numVertices = 8;
        }
        double radsBetweenVertices = (2.0 * Math.PI) / numVertices;

        List<Translation2d> points = new ArrayList<>();
        for (int i = 0; i < numVertices; i += 1) {
            double rads = radsBetweenVertices * i;
            Translation2d radiusVector = new Translation2d(radius * Math.cos(rads), radius * Math.sin(rads));
            points.add(center.plus(radiusVector));
        }

        // close the loop
        points.add(points.get(0));

        Logger.recordOutput(name, points.toArray(new Translation2d[0]));
    }
    
}
