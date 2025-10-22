package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

public class AimAtReef extends Command {

    private Drivetrain drivetrain;
    private Arm arm;
    private Wrist wrist;
    private Leds leds;
    private Supplier<ChassisSpeeds> translationController;
    private Supplier<ReefBranch> reefBranch;
    private Supplier<Boolean> isFacingReef;
    private Supplier<Direction> coralSideSupplier;
    private Direction coralSide;
    private ArmPosition desiredArmPosition;
    private boolean extensionTargetSet = false;
    private boolean shoulderNearTarget = false;
    private boolean extensionNearTarget = false;
    private boolean wristNearTarget = false;


    /**
     *  @param translationController - ChassisSpeeds supplier for driver input while scoring. TO BE IMPLEMENTED. CURRENTLY DOES NOTHING.
     */
    public AimAtReef(Drivetrain drivetrain, Arm arm, Wrist wrist, Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Leds leds, Supplier<Direction> sideCoralIsIn, Supplier<Boolean> isFacingReef) {
    
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.wrist = wrist;
        this.leds = leds;
        this.translationController = translationController;
        this.reefBranch=reefBranch;
        this.coralSideSupplier=sideCoralIsIn;
        this.isFacingReef = isFacingReef;
        addRequirements(drivetrain, arm.shoulder, arm.extension, wrist);

        super.setName("AimAtReef");
    }

    public boolean readyToScore() {
        boolean shoulderReady = Math.abs(desiredArmPosition.shoulderAngleDegrees - arm.getShoulderAngleDegrees()) < 1.5;
        boolean extensionReady = Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02;
        boolean wristReady = Math.abs(desiredArmPosition.wristAngleDegrees - wrist.getWristAngleDegrees()) < 4;
        boolean driveAngleGood = drivetrain.isAngleAligned();
        boolean driveTranslationGood = drivetrain.translationControllerAtSetpoint();

        Logger.recordOutput("aimAtReef/shoulderReady", shoulderReady);
        Logger.recordOutput("aimAtReef/extensionReady", extensionReady);
        Logger.recordOutput("aimAtReef/wristReady", wristReady);
        Logger.recordOutput("aimAtReef/driveAngleGood", driveAngleGood);
        Logger.recordOutput("aimAtReef/driveTranslationGood", driveTranslationGood);

        return shoulderReady && extensionReady && wristReady && driveAngleGood && driveTranslationGood;
    }

    public boolean shouldStartEarlyExitTimer() {
        return this.extensionTargetSet && (Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02);
    }


    private Pose2d adjustedReefScoringPose(Pose2d stalkPose, Direction sideCoralIsIn, boolean isFacingForward, int branchLevel) {
        double adjustedX = FieldConstants.stalkInsetMeters;        // puts center of robot at the outer edge of the reef
        adjustedX += DrivetrainConstants.bumperWidthMeters / 2.0;  // move back a half bumper length so the bumper is touching the edge of the reef
        adjustedX += FieldConstants.coralOuterDiameterMeters;      // move back one coral distance so we can still score if coral is in the way

        
        Rotation2d rotationAdjustment;
        if (isFacingForward) {
            rotationAdjustment = Rotation2d.k180deg;
        } else {
            rotationAdjustment = Rotation2d.kZero;
        }


        double adjustedY;
        if (sideCoralIsIn == Direction.left) {
            adjustedY = PlacerGrabber.innerWidthMeters/2.0;
        } else {
            adjustedY = -PlacerGrabber.innerWidthMeters/2.0;
        }

        if (!isFacingForward) {
            //adjust for different when coming out other side
            adjustedY += Math.signum(adjustedY) * Units.inchesToMeters(1);
            adjustedY *= -1;
        }

        boolean armNotAligned = (!shoulderNearTarget || !extensionNearTarget || !wristNearTarget);
        if (isFacingForward && branchLevel == 4 && armNotAligned) {
            adjustedX += 0.2;
        }



        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotationAdjustment);
        Pose2d scoringPose = stalkPose.plus(targetPoseToRobotRelativeToStalk);
        Logger.recordOutput("aimAtReef/targetDrivePose", scoringPose);
        return scoringPose;
    }

    /**
     * Generates an ArmPosition object representing the position of the arm to score on the desired branch.
     * This is calculated based on the position of the robot relative to the branch.
     */
    private ArmPosition calculateArmScoringPosition() {

        //COMMENTED OUT FOR THE MOMENT AS WE ARE USING SETPOINTS

        // Translation2d robotTranslation = drivetrain.getPoseMeters().getTranslation();
        // Translation3d branchTranslation = reefBranch.get().getLocation();

        // Translation2d horizontalTranslation = robotTranslation.minus(branchTranslation.toTranslation2d());
        // double horizontalExtensionMeters = horizontalTranslation.getNorm();

        // double verticalExtensionMeters = branchTranslation.getZ();

        // horizontalExtensionMeters -= 0.1;
        // verticalExtensionMeters += 0.2;

        // double targetWristAngleDegrees = WristConstants.maxAngleDegrees-5;

        return ArmPosition.getPreset(reefBranch.get().getLevel(), isFacingReef.get());
    }
    
    @Override
    public void initialize() {
        this.extensionTargetSet = false;
        coralSide = coralSideSupplier.get();
        this.desiredArmPosition = this.calculateArmScoringPosition();
        Logger.recordOutput("aimAtReef/running", true);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("aimAtReef/running", false);
    }

    @Override
    public void execute() {



        desiredArmPosition = calculateArmScoringPosition();
        Pose2d targetPose = adjustedReefScoringPose(reefBranch.get().getStalk().getPose2d(), coralSide, isFacingReef.get(), reefBranch.get().getLevel());

        Logger.recordOutput("aimAtReef/armDesiredDegrees", desiredArmPosition.shoulderAngleDegrees);
        Logger.recordOutput("aimAtReef/wristDesiredDegrees", desiredArmPosition.wristAngleDegrees);
        Logger.recordOutput("aimAtReef/extensionDesiredMeters", desiredArmPosition.extensionMeters);



        /**** ARM ALIGNMENT ****/


        boolean closeToReef = targetPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 1.0;
        boolean movingSlow = drivetrain.getSpeedMetersPerSecond() < 2.0;

        Translation2d frontFace = FieldElement.FRONT_REEF_FACE.getLocation2d();
        Translation2d backFace = FieldElement.BACK_REEF_FACE.getLocation2d();
        Translation2d centerOfReef = frontFace.plus(backFace).div(2.0);
        Translation2d robotToReef = centerOfReef.minus(drivetrain.getPoseMeters().getTranslation());
        robotToReef = robotToReef.div(robotToReef.getNorm());
        Rotation2d robotOrientation = drivetrain.getPoseMeters().getRotation();
        double dotProduct = (robotOrientation.getCos() * robotToReef.getX()) + (robotOrientation.getSin() * robotToReef.getY());

        boolean driveAngleClose = Math.abs(dotProduct) > 0.75;

        shoulderNearTarget = Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 30;
        extensionNearTarget = Math.abs(arm.getExtensionMeters() - desiredArmPosition.extensionMeters) < 0.3;
        wristNearTarget = Math.abs(wrist.getWristAngleDegrees() - desiredArmPosition.wristAngleDegrees) < 20;

        // immediately start moving shoulder
        arm.setShoulderTargetAngle(desiredArmPosition.shoulderAngleDegrees);

        if (movingSlow && shoulderNearTarget && driveAngleClose) {
            // Only start moving extension & wrist when shoulder is near the setpoint
    
            arm.setExtensionTargetLength(desiredArmPosition.extensionMeters);
            this.extensionTargetSet = true;

            boolean wristReadyToMove = Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 20;
                // && arm.getExtensionMeters() > ArmConstants.minExtensionMeters + 0.2;
            if (wristReadyToMove) {
                double maxWristVolts = 6;
                if (reefBranch.get().getLevel() == 4) maxWristVolts = 8;
                wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees, maxWristVolts);
            }
        }
        else {
            // Stow extension and wrist when the shoulder isn't ready yet
            this.extensionTargetSet = false;
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
            wrist.setTargetPositionDegrees(WristConstants.homeAngleDegrees);
        }

        if (closeToReef && DriverStation.isAutonomous()) {
            drivetrain.fullyTrustVisionNextPoseUpdate();
        }



        /**** DRIVETRAIN ALIGNMENT ****/

        ChassisSpeeds driverControl = translationController.get();
        if (Math.hypot(driverControl.vxMetersPerSecond, driverControl.vyMetersPerSecond) < 1) {
            double maxSpeed = 1;
            if (DriverStation.isAutonomous()) {
                maxSpeed = 1.0;
            }
            
            // drivetrain.beeLineToPose(targetPose);
            closeToReef = targetPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 1.25;
            // if (!closeToReef) {
            //     // drivetrain.profileToPose(targetPose);
            //     drivetrain.pidToPose(targetPose, 2.0);
            // } else {
            //     drivetrain.pidToPose(targetPose, maxSpeed);
            // }
            drivetrain.pidToPose(targetPose, 3.25);

            
            // drivetrain.fieldOrientedDrive(driverControl.div(3), true);
            // drivetrain.fieldOrientedDriveOnALine(driverControl.div(3.0), targetPose);
        }
        else {
            drivetrain.fieldOrientedDrive(driverControl.div(3), true);
        }



        // leds.progressBar(arm.getExtensionMeters() / desiredArmPosition.extensionMeters);
    }
}
