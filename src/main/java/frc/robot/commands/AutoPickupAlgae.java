package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.ReefFace;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

public class AutoPickupAlgae extends Command {

    private Drivetrain drivetrain;
    private Arm arm;
    private Wrist wrist;
    private ArmPosition desiredArmPosition;
    private Supplier<ReefFace> closestFace;
    Supplier<ChassisSpeeds> translationController;
    private PlacerGrabber placerGrabber;

    private double retractionAdjustedX;
    private double retractedAndWaitingForOthersAdjustedX;
    private double intakeAlgaeAdjustedX;
    boolean isHighAlgae;


    public AutoPickupAlgae(Drivetrain drivetrain, Arm arm, Wrist wrist,
        Supplier<ReefFace> reefFace, Supplier<ChassisSpeeds> translationController, PlacerGrabber placerGrabber) {
    
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.wrist = wrist;
        this.closestFace = reefFace;
        isHighAlgae=closestFace.get().isHighAlgae();
        this.translationController=translationController;
        this.placerGrabber = placerGrabber;
        addRequirements(drivetrain, arm.shoulder, arm.extension, wrist, placerGrabber);

        super.setName("AutoPickupAlgae");
    }

    public boolean readyToPickUp() {
        boolean shoulderReady = Math.abs(desiredArmPosition.shoulderAngleDegrees - arm.getShoulderAngleDegrees()) < 1.5;
        boolean extensionReady = Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02;
        boolean wristReady = Math.abs(desiredArmPosition.wristAngleDegrees - wrist.getWristAngleDegrees()) < 4;

        return shoulderReady && extensionReady && wristReady;
    }


    private Pose2d adjustedReefScoringPose(Pose2d reefFacePose, double extraAdjustedX) {
        double adjustedX = 0;   // puts center of robot at the outer edge of the reef
        adjustedX += DrivetrainConstants.bumperWidthMeters / 2.0;  // move back a half bumper length so the bumper is touching the edge of the reef
        adjustedX += FieldConstants.coralOuterDiameterMeters + extraAdjustedX;    //was0.3  // move back one coral distance so we can still score if coral is in the way

        
        Rotation2d rotationAdjustment = Rotation2d.k180deg;

        double adjustedY = 0;



        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotationAdjustment);
        Pose2d scoringPose = reefFacePose.plus(targetPoseToRobotRelativeToStalk);
        Logger.recordOutput("autoPickup/targetDrivePose", scoringPose);
        return scoringPose;
    }


    
    @Override
    public void initialize() {
        
        retractionAdjustedX = 0.425;
        retractedAndWaitingForOthersAdjustedX = 0.275;
        intakeAlgaeAdjustedX = -0.05;

    }

    @Override
    public void execute() {
        double extensionTargetLength = isHighAlgae ? 0.92 : 0.71;
        double shoulderTargetAngle;
        double wristTargetAngle;
        double adjustedX;

        arm.setExtensionTargetLength(extensionTargetLength);
        boolean extensionNearTarget = Math.abs(arm.getExtensionMeters() - extensionTargetLength) < 0.1;
        Logger.recordOutput("autoPickup/extensionTargetError",Math.abs(arm.getExtensionMeters() - extensionTargetLength));
        Logger.recordOutput("autoPickup/extensionNearTarget",extensionNearTarget);
        Logger.recordOutput("autoPickup/extensionTargetLength",extensionTargetLength);
        Logger.recordOutput("autoPickup/arm.getExtensionMeters()",arm.getExtensionMeters());
        if (extensionNearTarget) {
            shoulderTargetAngle = isHighAlgae ? 65.3: 47;
        } else {
            shoulderTargetAngle = ArmPosition.frontL4.shoulderAngleDegrees;
        }

        if (extensionNearTarget) {
            wristTargetAngle = isHighAlgae ? -52 : -46;
        } else {
            wristTargetAngle = 90;
        }

        arm.setShoulderTargetAngle(shoulderTargetAngle);
        wrist.setTargetPositionDegrees(wristTargetAngle);
        placerGrabber.setFrontRollerVolts(10);

        boolean shoulderNearTarget = Math.abs(arm.getShoulderAngleDegrees() - shoulderTargetAngle) < 5;
        boolean wristNearTarget = Math.abs(wrist.getWristAngleDegrees() - wristTargetAngle) < 5;

        if (!extensionNearTarget) {
            adjustedX = retractionAdjustedX;
        } else if (extensionNearTarget && (!wristNearTarget||!shoulderNearTarget)) {
            adjustedX = retractedAndWaitingForOthersAdjustedX;
        } else if (placerGrabber.getFrontRollerAvgAmps() > 13.5){
            adjustedX = 1.0;
        } else {
            adjustedX=intakeAlgaeAdjustedX;
        }

        Pose2d targetPose = adjustedReefScoringPose(closestFace.get().getPose2d(), adjustedX);

        ChassisSpeeds driverControl = translationController.get();
        if (Math.hypot(driverControl.vxMetersPerSecond, driverControl.vyMetersPerSecond) < 1) {
             drivetrain.pidToPose(targetPose, 1);
        }
        else {
            drivetrain.fieldOrientedDrive(driverControl.div(3), true);
        }
            
    }
}
