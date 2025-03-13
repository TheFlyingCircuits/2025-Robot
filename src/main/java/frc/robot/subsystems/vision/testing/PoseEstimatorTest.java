package frc.robot.subsystems.vision.testing;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.PlayingField.FieldConstants;

public class PoseEstimatorTest {

    SwerveDrivePoseEstimator fusedPoseEstimator;
    SwerveDrivePoseEstimator spamPoseEstimator;

    SwerveModulePosition frontLeft = new SwerveModulePosition();
    SwerveModulePosition frontRight = new SwerveModulePosition();
    SwerveModulePosition backLeft = new SwerveModulePosition();
    SwerveModulePosition backRight = new SwerveModulePosition();
    SwerveModulePosition[] positions = {frontLeft, frontRight, backLeft, backRight};

    boolean fusedNeedsVision = true;
    double timestamp = -1;

    SwerveModulePosition endPosition = new SwerveModulePosition(FieldConstants.maxX, new Rotation2d());

    public PoseEstimatorTest() {
        fusedPoseEstimator = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.swerveKinematics, new Rotation2d(), positions, new Pose2d());
        spamPoseEstimator = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.swerveKinematics, new Rotation2d(), positions, new Pose2d());
    }

    private void changePositions(Translation2d vector) {
        for (int i = 0; i < 4; i += 1) {
            positions[i] = new SwerveModulePosition(positions[i].distanceMeters + vector.getNorm(), vector.getAngle());
        }
    }

    private void resetPositions() {
        for (int i = 0; i < 4; i += 1) {
            positions[i] = new SwerveModulePosition();
        }
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            resetPositions();
            fusedPoseEstimator.resetPosition(new Rotation2d(), positions, new Pose2d());
            spamPoseEstimator.resetPosition(new Rotation2d(), positions, new Pose2d());
            fusedNeedsVision = true;
            log();
            return;
        }
        changePositions(new Translation2d(Units.inchesToMeters(32.0/50.0), 0));

        fusedPoseEstimator.update(new Rotation2d(), positions);
        spamPoseEstimator.update(new Rotation2d(), positions);

        if (fusedPoseEstimator.getEstimatedPosition().getX() > FieldConstants.maxX/4.0) {
            if (fusedNeedsVision) {
                timestamp = Timer.getFPGATimestamp();
                fusedPoseEstimator.addVisionMeasurement(new Pose2d(FieldConstants.maxX, 0, new Rotation2d()), timestamp);
                fusedNeedsVision = false;
            }
            spamPoseEstimator.addVisionMeasurement(new Pose2d(FieldConstants.maxX, 0, new Rotation2d()), timestamp);
        }

        log();
    }

    public void log() {
        Logger.recordOutput("fusedTest", fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("spamTest", spamPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("trigger", FieldConstants.maxX / 4.0);

        fusedPoseEstimator.sampleAt(timestamp);
    }
    
}
