package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public interface VisionIO {

    public final class VisionMeasurement {

        public String cameraName;

        /**
         * Pose of the robot in meters and degrees, relative to the field. The origin is always centered on the right edge of the blue side, 
         * so that all coordinates on the field are positive.
         * <p>
         * When each axis is pointed at you, their corresponding angles are counterclockwise positive.
         * For example, when looking down at the z-axis, the yaw is counterclockwise positive.
         */
        public Pose2d robotFieldPose;

        /**
         * FPGA timestamp of when the robotFieldPose has been updated
         */
        public double timestampSeconds;

        /**
         * The average distance from the camera to each tag that it sees, in meters.
         */
        public double averageTagDistanceMeters;

        /**
         * Standard deviations of this vision measurement, in meters and radians.
         * Represents (X, Y, and rotation).
         */
        public Matrix<N3, N1> stdDevs;

        public int[] tagsUsed;

        /**
         * Creates a new VisionMeasurement object. See the definition of this class for further documentation.
         */
        public VisionMeasurement(Pose2d robotFieldPose, double timestampSeconds, double nearestTagDistanceMeters, Matrix<N3, N1> stdDevs) {
            this.robotFieldPose=robotFieldPose;
            this.timestampSeconds=timestampSeconds;
            this.averageTagDistanceMeters=nearestTagDistanceMeters;
            this.stdDevs=stdDevs;
            this.tagsUsed = null;
        }

        /**
         * Creates a new VisionMeasurement object. See the definition of this class for further documentation.
         */
        public VisionMeasurement() {};


    }

    public class VisionIOInputs {

        /**
         * List of all vision measurements from the last frame. 
         * This array is sorted by the standard deviation of the measurement, from biggest to smallest.
         * (I think? this is the order that you should apply measurements to the pose estimator)
         * If no measurements are present, the array will be empty.
         */
        public List<VisionMeasurement> visionMeasurements = new ArrayList<VisionMeasurement>();

        /**
         * The 3D coordinates of every note detected by the intake camera in the robot coordinate frame.
         * <p>
         * Positive X is forward
         * <p>
         * Positive Y is left
         * <p>
         * Positive Z is up
         */
        public List<Translation3d> detectedCoralsRobotFrame = new ArrayList<Translation3d>();

    }

    //AdvantageKit's AutoLog doesn't support logging array lists or custom objects,
    //so we wrote our own logging methods. There is a better way of doing this using
    //WPILib structs, but I'm not sure exactly how to go about doing it.
    public class VisionIOInputsLogged extends VisionIOInputs implements LoggableInputs {
        
        public void toLog(LogTable table) {

            for (VisionMeasurement meas : visionMeasurements) {
                
                String rootString = meas.cameraName+"VisionMeasurement";

                table.put(rootString+"/RobotFieldPose", meas.robotFieldPose);
                table.put(rootString+"/TimestampSeconds", meas.timestampSeconds);
                table.put(rootString+"/NearestTagDistanceMeters", meas.averageTagDistanceMeters);
                table.put(rootString+"/StdDevX", meas.stdDevs.get(0, 0));
                table.put(rootString+"/StdDevY", meas.stdDevs.get(1, 0));
                table.put(rootString+"/StdDevRot", meas.stdDevs.get(2, 0));
                table.put(rootString+"/CameraName", meas.cameraName);
                table.put(rootString+"/tagsUsed", meas.tagsUsed);

            }

            for (int i = 0; i < detectedCoralsRobotFrame.size(); i++) {

                Translation3d coral = detectedCoralsRobotFrame.get(i);

                table.put("DetectedCoralRobotFrame" + Integer.toString(i), coral);
            }
        }

        public void fromLog(LogTable table) {




            for (int i = 0;;i++) {
                String rootString = VisionConstants.tagCameraNames[i]+"VisionMeasurement";

                //hacky way to check if this vision measurement doesn't exist
                if (table.get(rootString+"/RobotFieldPose", 0) == 0)
                    break;
                
                VisionMeasurement meas = new VisionMeasurement();

                meas.robotFieldPose = table.get(rootString+"/RobotFieldPose", meas.robotFieldPose);
                meas.timestampSeconds = table.get(rootString+"/TimestampSeconds", meas.timestampSeconds);
                meas.averageTagDistanceMeters = table.get(rootString+"/NearestTagDistanceMeters", meas.averageTagDistanceMeters);
                double stdDevX = table.get(rootString+"/StdDevX", meas.stdDevs.get(0, 0));
                double stdDevY = table.get(rootString+"/StdDevY", meas.stdDevs.get(1, 0));
                double stdDevRot = table.get(rootString+"/StdDevRot", meas.stdDevs.get(2, 0));
                meas.stdDevs = VecBuilder.fill(stdDevX, stdDevY, stdDevRot);
                meas.cameraName = table.get(rootString+"/CameraName", "");
                meas.tagsUsed = table.get(rootString+"/tagsUsed", new int[0]);

                visionMeasurements.add(meas);
            }

            for (int i = 0;; i++) {

                String entryName = "DetectedCoralRobotFrame" + Integer.toString(i);

                if (table.get(entryName, 0) == 0) {
                    break;
                }

                Translation3d coral = table.get("DetectedCoralRobotFrame" + Integer.toString(i), new Translation3d());
                detectedCoralsRobotFrame.add(coral);

            }
            
        }
    }

    public default void updateInputs(VisionIOInputs inputs) {};

}
