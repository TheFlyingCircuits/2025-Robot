package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.PlayingField.FieldConstants;

public class PoseObservation implements LoggableInputs {

    public Pose3d robotPose;
    public double timestampSeconds;
    public int[] tagsUsed;
    public double avgTagToCamMeters;
    public double ambiguity;

    public Translation3d[] sightlines = new Translation3d[0];

    public PoseObservation(Pose3d robotPose, double timestampSeconds, int[] tagsUsed, double avgTagToCamMeters, double ambiguity, Pose3d camPose) {
        this.robotPose = robotPose;
        this.timestampSeconds = timestampSeconds;
        this.tagsUsed = tagsUsed;
        this.avgTagToCamMeters = avgTagToCamMeters;
        this.ambiguity = ambiguity;

        this.sightlines = new Translation3d[2*tagsUsed.length];
        for (int i = 0; i < tagsUsed.length; i += 1) {
            sightlines[2*i] = camPose.getTranslation();
            sightlines[(2*i)+1] = FieldConstants.tagPose(tagsUsed[i]).getTranslation();
        }
    }

    public PoseObservation() {
        this.robotPose = null;
        this.timestampSeconds = -1;
        this.tagsUsed = new int[0];
        this.avgTagToCamMeters = -1;
        this.ambiguity = 0;
    }


    public boolean usesTag(int tagToLookFor) {
        for (int tagID : tagsUsed) {
            if (tagID == tagToLookFor) {
                return true;
            }
        }
        return false;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        double slopeStdDevMeters_PerMeterX = 0.008;
        double slopeStdDevMeters_PerMeterY = 0.008;

        if (tagsUsed.length > 1) {
            slopeStdDevMeters_PerMeterX = 0.004;
            slopeStdDevMeters_PerMeterY = 0.009;
        }

        return VecBuilder.fill(
            slopeStdDevMeters_PerMeterX*avgTagToCamMeters,
            slopeStdDevMeters_PerMeterY*avgTagToCamMeters,
            99999
        );
    }

    @Override
    public void toLog(LogTable table) {
        if (robotPose == null) {
            table.put("robotPose", new Pose3d[0]);
        }
        else {
            table.put("robotPose", new Pose3d[] {robotPose});
        }
        // table.put("robotPose", robotPose);
        table.put("timestamp", timestampSeconds);
        table.put("tagsUsed", tagsUsed);
        table.put("avgTagToCamMeters", avgTagToCamMeters);
        table.put("ambiguity", ambiguity);
        table.put("sightlines", sightlines);
    }

    @Override
    public void fromLog(LogTable table) {
        Pose3d[] optionalRobotPose = table.get("robotPose", new Pose3d[0]);
        if (optionalRobotPose.length > 0) {
            robotPose = optionalRobotPose[0];
        }
        else {
            robotPose = null;
        }
        // robotPose = table.get("robotPose", robotPose);
        timestampSeconds = table.get("timestamp", timestampSeconds);
        tagsUsed = table.get("tagsUsed", tagsUsed);
        avgTagToCamMeters = table.get("stdDevs", avgTagToCamMeters);
        ambiguity = table.get("ambiguity", ambiguity);
        sightlines = table.get("sightlines", sightlines);
    }
}
