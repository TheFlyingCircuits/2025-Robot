package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;

public record SingleTagPoseObservation (String camName, Pose3d robotPose, double timestampSeconds, int tagUsed, double tagToCamMeters, double ambiguity) {
    public Matrix<N3, N1> getStandardDeviations() {
        double slopeStdDevMeters_PerMeter = 0.003;

        return VecBuilder.fill(
            slopeStdDevMeters_PerMeter*tagToCamMeters,
            slopeStdDevMeters_PerMeter*tagToCamMeters,
            99999
        );
    }

    public boolean usesReefTag() {
        for (ReefFace reefFace : FieldElement.ALL_REEF_FACES) {
            if (this.tagUsed() == reefFace.getTagID()) {
                return true;
            }
        }

        return false;
    }

    public Pose3d getTagPose() {
        return FieldConstants.tagPose(tagUsed);
    }
}
