package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.wrist.Wrist;

public class ScoreOnReef extends Command {

    private Drivetrain drivetrain;
    private Arm arm;
    private Wrist wrist;
    private Supplier<ChassisSpeeds> translationController;
    private Supplier<ReefBranch> reefBranch;
    private Leds leds;
    private Supplier<String> sideCoralIsIn;
    private Supplier<Boolean> isFacingForward;

    /**
     *  @param translationController - ChassisSpeeds supplier for driver input while scoring.
     */
    public ScoreOnReef(Drivetrain drivetrain, Arm arm, Wrist wrist, Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Leds leds, Supplier<String> sideCoralIsIn, Supplier<Boolean> isFacingReef) {
    
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.wrist = wrist;
        this.translationController = translationController;
        this.reefBranch=reefBranch;
        this.leds = leds;
        this.sideCoralIsIn=sideCoralIsIn;
        this.isFacingForward=isFacingReef;
        addRequirements(drivetrain, arm.shoulder, arm.extension, wrist);
    }

    // bumper 37 inches
    private Pose2d adjustedReefScoringPose(Pose2d stalkPose, String sideCoralIsIn, boolean isFacingForward) {
        double adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(25);
        double adjustedY;
        Rotation2d rotation = Rotation2d.fromDegrees(180);

        //commenting out pivot side score for now because haven't tested it
        // if (isFacingForward) {
        //     rotation = Rotation2d.fromDegrees(180);
        // } else {
        //     rotation = new Rotation2d();
        // }

        if (((sideCoralIsIn == "left") & isFacingForward) || ((sideCoralIsIn == "right") & !isFacingForward)) {
            adjustedY = Units.inchesToMeters(3.25);
        } else {
            adjustedY = -Units.inchesToMeters(3.25);
        }


        //TODO: make this potentially a pivot-side score
        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotation);
        return stalkPose.plus(targetPoseToRobotRelativeToStalk);
    }

    /**
     * Generates an ArmPosition object representing the position of the arm to score on the desired branch.
     * This is calculated based on the position of the robot relative to the branch.
     */
    private ArmPosition calculateArmScoringPosition() {
        Translation2d robotTranslation = drivetrain.getPoseMeters().getTranslation();
        Translation3d branchTranslation = reefBranch.get().getLocation();

        Translation2d horizontalTranslation = robotTranslation.minus(branchTranslation.toTranslation2d());
        double horizontalExtensionMeters = horizontalTranslation.getNorm();

        double verticalExtensionMeters = branchTranslation.getZ();

        horizontalExtensionMeters -= 0.1; //TODO: tweak these
        verticalExtensionMeters += 0.2;

        double targetWristAngleDegrees = WristConstants.maxAngleDegrees-5;
        
        switch (reefBranch.get().getLevel()) {
            case 1:
                targetWristAngleDegrees = 40;
                break;
            case 2:
                return new ArmPosition(38.2, 101, 0.73);
            case 3:
                return new ArmPosition(57.8, 79, 0.97);
            case 4:
                return new ArmPosition(71, 42, 1.57);
        }

        return ArmPosition.generateArmPosition(
            new Pose2d(
                horizontalExtensionMeters,
                verticalExtensionMeters,
                Rotation2d.fromDegrees(targetWristAngleDegrees)
            )
        );

    }

    @Override
    public void execute() {
        Pose2d targetPose = adjustedReefScoringPose(reefBranch.get().getStalk().getPose2d(), sideCoralIsIn.get(), isFacingForward.get());

        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        drivetrain.pidToPose(targetPose);

        ArmPosition desiredArmPosition = calculateArmScoringPosition();

        Logger.recordOutput("scoreOnReef/armDesiredDegrees", desiredArmPosition.shoulderAngleDegrees);
        Logger.recordOutput("scoreOnReef/wristDesiredDegrees", desiredArmPosition.wristAngleDegrees);
        Logger.recordOutput("scoreOnReef/extensionDesiredMeters", desiredArmPosition.extensionMeters);

        arm.setShoulderTargetAngle(desiredArmPosition.shoulderAngleDegrees);
        
        if (Math.abs(arm.getShoulderAngleDegrees() - arm.getTargetShoulderAngleDegrees()) < 20) {
            arm.setExtensionTargetLength(desiredArmPosition.extensionMeters);
        }
        else {
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
        }

        wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees);

        leds.progressBar(arm.getExtensionMeters() / desiredArmPosition.extensionMeters);
    }
}
