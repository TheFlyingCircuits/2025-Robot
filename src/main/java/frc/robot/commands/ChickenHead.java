package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AdvantageScopeDrawingUtils;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

public class ChickenHead extends Command {

    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private Arm arm;
    private Wrist wrist;
    private PlacerGrabber placerGrabber;
    private Supplier<ReefBranch> targetBranchSupplier;
    private ReefBranch targetBranch;
    private boolean readyToScore = false;

    public ChickenHead(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, Arm arm, Wrist wrist, PlacerGrabber intakeWheels, Supplier<ReefBranch> targetBranchSupplier) {
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.arm = arm;
        this.wrist = wrist;
        this.placerGrabber = intakeWheels;
        this.targetBranchSupplier = targetBranchSupplier;

        // intakeWheels aren't added to requirements because we never write to them,
        // we only read from them.
        super.addRequirements(drivetrain, arm.shoulder, arm.extension, wrist);
        super.setName("ChickenHeadCommand");
    }

    @Override
    public void initialize() {
        targetBranch = targetBranchSupplier.get();
        this.readyToScore = false;
    }

    @Override
    public void execute() {
        // this.setDriveToOneCoralDistancePose(); <- used for generating setpoints
        this.aimDriveAtReef();

        double maxDistanceFromCenter = DrivetrainConstants.frameWidthMeters/2.0;
        maxDistanceFromCenter += Units.inchesToMeters(12 + 6);
        if (drivetrain.getPoseMeters().getTranslation().getDistance(targetBranch.getLocation2d()) > maxDistanceFromCenter) {
            return;
        }

        ArmPosition desiredArmState = this.getDesiredArmState().constrainedToLimits();

        // immediately start moving shoulder
        arm.setShoulderTargetAngle(desiredArmState.shoulderAngleDegrees);

        boolean closeToReef = targetBranch.getFace().getPose2d().minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 1;
        boolean movingSlow = drivetrain.getSpeedMetersPerSecond() < 2;
        boolean shoulderNearTarget = Math.abs(arm.getShoulderAngleDegrees() - desiredArmState.shoulderAngleDegrees) < 10;
        if (closeToReef && movingSlow && shoulderNearTarget) {
            // Only start moving extension & wrist when shoulder is near the setpoint
            arm.setExtensionTargetLength(desiredArmState.extensionMeters);
                
            double maxWristVolts = 9;
            if (targetBranch.getLevel() == 4) maxWristVolts = 9;
            wrist.setTargetPositionDegrees(desiredArmState.wristAngleDegrees, maxWristVolts);
        }
        else {
            // Stow extension and wrist when the shoulder isn't ready yet
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
            wrist.setTargetPositionDegrees(WristConstants.maxAngleDegrees - 5);
        }

        this.readyToScore = this.readyToScore(desiredArmState);

        // this.arm.setShoulderTargetAngle(desiredArmState.shoulderAngleDegrees);
        // this.arm.setExtensionTargetLength(desiredArmState.extensionMeters);
        // this.wrist.setTargetPositionDegrees(desiredArmState.wristAngleDegrees);
    }

    private boolean readyToScore(ArmPosition desiredArmPosition) {
        boolean shoulderReady = Math.abs(desiredArmPosition.shoulderAngleDegrees - arm.getShoulderAngleDegrees()) < 1;
        boolean extensionReady = Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02;
        boolean wristReady = Math.abs(desiredArmPosition.wristAngleDegrees - wrist.getWristAngleDegrees()) < 2;
        boolean driveAngleGood = drivetrain.isAngleAligned();

        Logger.recordOutput("chickenHead/shoulderReady", shoulderReady);
        Logger.recordOutput("chickenHead/extensionReady", extensionReady);
        Logger.recordOutput("chickenHead/wristReady", wristReady);
        Logger.recordOutput("chickenHead/driveAngleGood", driveAngleGood);

        return shoulderReady && extensionReady && wristReady && driveAngleGood;
    }

    public boolean readyToScore() {
        return this.readyToScore;
    }

    @Override
    public void end(boolean wasInterrupted) {
        // clear advantage scope viz
        Logger.recordOutput("chickenHead/desiredCoralPose_fieldFrame", new Pose3d[0]);
        Logger.recordOutput("chickenHead/targetWristPose_fieldFrame", new Pose3d[0]);
        AdvantageScopeDrawingUtils.eraseDrawing("chickenHead/targetWristWireframe");



        Logger.recordOutput("chickenHead/driveAlignmentLine", new Pose2d[0]);
        Logger.recordOutput("chickenHead/lineThroughLeftCoral", new Pose2d[0]);
        Logger.recordOutput("chickenHead/lineThroughRightCoral", new Pose2d[0]);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void setDriveToOneCoralDistancePose() {
        Pose2d stalkPose = this.targetBranch.getStalk().getPose2d();
        boolean isFacingForward = drivetrain.isFacingReef();
        Direction sideCoralIsIn = placerGrabber.sideCoralIsIn();

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
        if (((sideCoralIsIn == Direction.left) & isFacingForward) || ((sideCoralIsIn == Direction.right) & !isFacingForward)) {
            adjustedY = Units.inchesToMeters(3.5);
        } else {
            adjustedY = Units.inchesToMeters(-3.5);
        }


        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotationAdjustment);
        Pose2d scoringPose = stalkPose.plus(targetPoseToRobotRelativeToStalk);
        drivetrain.setPoseMeters(scoringPose);
    }

    private ArmPosition getDesiredArmState() {
        // find desired wrist pose relative to robot
        Pose3d desiredWristPose_fieldFrame = this.getDesiredWristPose_fieldFrame();
        Pose3d robotPose_fieldFrame = new Pose3d(drivetrain.getPoseMeters());
        Pose3d desiredWristPose_robotFrame = desiredWristPose_fieldFrame.relativeTo(robotPose_fieldFrame);

        // rotate the desired wrist pose to be directly in front of or directly behind the robot
        // (depending on scoring direction). We're going to aim based on where we'll be shortly
        // instead of wherever we happen to be right now.
        Rotation2d yawToWrist_robotFrame = desiredWristPose_robotFrame.getTranslation().toTranslation2d().getAngle();
        Rotation2d yawAdjustment = drivetrain.isFacingReef() ?
                                   yawToWrist_robotFrame.times(-1) :                             // scoring on intake side, put wrist in front
                                   yawToWrist_robotFrame.times(-1).rotateBy(Rotation2d.k180deg); // scoring on pivot side, put wrist in back
        
        desiredWristPose_robotFrame = desiredWristPose_robotFrame.rotateBy(new Rotation3d(yawAdjustment));
        AdvantageScopeDrawingUtils.drawWrist("chickenHead/wristInFrontWireframe", this.robotFrameToFieldFrame(desiredWristPose_robotFrame));
        Logger.recordOutput("chickenHead/wristInFrontPose", this.robotFrameToFieldFrame(desiredWristPose_robotFrame));

        // find line of sight from shoulder to desired wrist location
        Translation3d currentShoulderLocation_robotFrame = this.getShoulderPose_robotFrame().getTranslation();
        Translation3d shoulderToWrist = desiredWristPose_robotFrame.getTranslation().minus(currentShoulderLocation_robotFrame);

        // lineOfSight from shoulder to wrist, the extension, and the bicep make right triangle.
        // We use the pythagorean theorem to solve for the extension length
        // |shoulderToWrist|^2 = |shoulderToElbow|^2 + |elbowToWrist|^2
        double shoulderToWristSquared = shoulderToWrist.getNorm() * shoulderToWrist.getNorm();

        Translation3d shoulderToElbow = ArmConstants.elbowLocation_shoulderFrame;
        double shoulerToElbowSquared = shoulderToElbow.getNorm() * shoulderToElbow.getNorm();

        double desiredElbowToWristLength = Math.sqrt(shoulderToWristSquared - shoulerToElbowSquared);

        // use lengths to solve for shoulder angle
        double radiansFromBicepToSightline = Math.atan2(desiredElbowToWristLength, shoulderToElbow.getNorm());
        double radiansFromHorizontalToSightline = Math.atan2(shoulderToWrist.getZ(), shoulderToWrist.getX());
        double radiansFromHorizontalToBicep = radiansFromHorizontalToSightline - radiansFromBicepToSightline;
        double radiansFromHorizontalToExtensionAxis = radiansFromHorizontalToBicep + Units.degreesToRadians(90);

        // use shoulder angle to find wrist angle
        Translation3d desiredWristDirection_robotFrame = new Translation3d(1, 0, 0).rotateBy(desiredWristPose_robotFrame.getRotation());
        double radiansFromHorizontalToWristAngle = Math.atan2(desiredWristDirection_robotFrame.getZ(), desiredWristDirection_robotFrame.getX());
        Logger.recordOutput("chickenHead/desiredWristFromHorizontal", Units.radiansToDegrees(radiansFromHorizontalToWristAngle));
        Logger.recordOutput("chickenHead/wristInRobotNoTransform", desiredWristPose_robotFrame);
        double radiansOfWristRelativeToExtensionAxis = radiansFromHorizontalToWristAngle - radiansFromHorizontalToExtensionAxis;

        // Now just a few basic conversions, and we're done!
        double targetShoulderDegrees = Units.radiansToDegrees(radiansFromHorizontalToExtensionAxis);
        double targetWristDegrees = Units.radiansToDegrees(radiansOfWristRelativeToExtensionAxis);

        Translation3d wristLocation_elbowFrame = new Translation3d(desiredElbowToWristLength, 0, 0);
        Translation3d tipOfFinalStage_elbowFrame = wristLocation_elbowFrame.minus(ArmConstants.tipOfFinalStageToWrist);
        double targetExtensionMeters = tipOfFinalStage_elbowFrame.minus(ArmConstants.retractionHardStop_elbowFrame).getNorm();

        return new ArmPosition(targetShoulderDegrees, targetWristDegrees, targetExtensionMeters);
    }

    private void aimDriveAtReef() {
        // aim the robot towards the desired wrist location when scoring on the intake side,
        // and aim away from the desired wrist location when scoring on the pivot side.
        // We don't just copy over the yaw of the desired wrist pose itself because that can sometimes
        // be undefined (e.g. when the desired wrist pose has the wrist pointing straight up, which could happen on L4)
        Translation3d desiredWristLocation_fieldFrame = this.getDesiredWristPose_fieldFrame().getTranslation();
        Translation2d vectorFromRobotToWrist = desiredWristLocation_fieldFrame.toTranslation2d().minus(drivetrain.getPoseMeters().getTranslation());
        
        boolean scoringOnIntakeSide = drivetrain.isFacingReef();
        Rotation2d directionToPoint = scoringOnIntakeSide ?
                                      vectorFromRobotToWrist.getAngle() :
                                      vectorFromRobotToWrist.times(-1).getAngle();

        drivetrain.fieldOrientedDriveWhileAiming(translationController.get(), directionToPoint);

        // Some logging for viz in advantage scope
        Pose2d alignmentLine = new Pose2d(desiredWristLocation_fieldFrame.toTranslation2d(), directionToPoint);
        Logger.recordOutput("chickenHead/driveAlignmentLine", new Pose2d[] {
            alignmentLine.plus(new Transform2d(-10, 0, Rotation2d.kZero)),
            alignmentLine,
            alignmentLine.plus(new Transform2d(10, 0, Rotation2d.kZero))
        });

        Transform2d lineThroughLeftCoral_driveFrame = new Transform2d(0, ArmConstants.intakeToLeftCoral.getY(), Rotation2d.kZero);
        Transform2d lineThroughRightCoral_driveFrame = new Transform2d(0, ArmConstants.intakeToRightCoral.getY(), Rotation2d.kZero);

        Pose2d lineThroughLeftCoral_fieldFrame = drivetrain.getPoseMeters().plus(lineThroughLeftCoral_driveFrame);
        Pose2d lineThroughRightCoral_fieldFrame = drivetrain.getPoseMeters().plus(lineThroughRightCoral_driveFrame);

        Logger.recordOutput("chickenHead/lineThroughLeftCoral", new Pose2d[] {
            lineThroughLeftCoral_fieldFrame.plus(new Transform2d(-10, 0, Rotation2d.kZero)),
            lineThroughLeftCoral_fieldFrame,
            lineThroughLeftCoral_fieldFrame.plus(new Transform2d(10, 0, Rotation2d.kZero))
        });

        Logger.recordOutput("chickenHead/lineThroughRightCoral", new Pose2d[] {
            lineThroughRightCoral_fieldFrame.plus(new Transform2d(-10, 0, Rotation2d.kZero)),
            lineThroughRightCoral_fieldFrame,
            lineThroughRightCoral_fieldFrame.plus(new Transform2d(10, 0, Rotation2d.kZero))
        });
    }

    private Pose3d getDesiredWristPose_fieldFrame() {
        // find where the coral should be, and then place the wrist
        // so that it holds the coral in its desired pose.
        Pose3d desiredCoralPose_fieldFrame = this.getDesiredCoralPose_fieldFrame();

        // use the inverse of the typical transformation to work backwards
        Transform3d wristPose_coralFrame = this.getWristPose_robotFrame().minus(this.getHeldCoralPose_robotFrame());
        Pose3d wristPose_fieldFrame = desiredCoralPose_fieldFrame.plus(wristPose_coralFrame);

        // some logging / viz before returning
        Logger.recordOutput("chickenHead/targetWristPose_fieldFrame", new Pose3d[] {wristPose_fieldFrame});
        AdvantageScopeDrawingUtils.drawWrist("chickenHead/targetWristWireframe", wristPose_fieldFrame);

        return wristPose_fieldFrame;
    }

    private Pose3d getDesiredCoralPose_fieldFrame() {
        // 1) Start by computing the ideal pose for the coral (i.e along the stalk axis)
        Pose3d tipOfTargetBranch = targetBranch.getPose();

        // TODO: picture / diagram / ascii art
        // Orange wheels are pointing away from the reef when scoring on intake side,
        // and pointing towards the reef when scoring on the pivot side.
        boolean backOfCoralShouldFaceReef = drivetrain.isFacingReef();
        Rotation3d idealCoralOrientation_branchFrame = backOfCoralShouldFaceReef ? 
                                                       new Rotation3d(Units.degreesToRadians(180), 0, 0) :
                                                       new Rotation3d(0, Units.degreesToRadians(180), 0);
        
        double idealDistanceFromBranch = FieldConstants.coralLengthMeters/2.0; // coral flush with branch
        idealDistanceFromBranch += FieldConstants.coralOuterRadiusMeters;      // required room to rotate the coral when off angle
        idealDistanceFromBranch += Units.inchesToMeters(1);                    // just a bit of a buffer
        Transform3d idealCoralPose_branchFrame = new Transform3d(idealDistanceFromBranch, 0, 0, idealCoralOrientation_branchFrame);
        Pose3d idealCoralPose_fieldframe = tipOfTargetBranch.plus(idealCoralPose_branchFrame);

        // 2) Rotate this ideal pose based on the robot's current position to allow for true
        //    3d chicken heading! The pivot point is on the side of the coral that will end up
        //    facing the reef when we raise the arm, and the pivot axis is the world's Z axis.
        //
        //    While being directly in front of the branch is always ideal,
        //    this adjustment makes it not strictly necessary.
        //    TODO: better description / visual aid
        Transform3d sideOfCoralClosestToBranch_coralFrame = backOfCoralShouldFaceReef ? 
                                                            new Transform3d(-FieldConstants.coralLengthMeters/2.0, 0, 0, Rotation3d.kZero) :
                                                            new Transform3d(+FieldConstants.coralLengthMeters/2.0, 0, 0, Rotation3d.kZero);

        Translation3d pivotPoint_fieldFrame = idealCoralPose_fieldframe.plus(sideOfCoralClosestToBranch_coralFrame).getTranslation();

        // 2.5) Find the field coords of a point on the robot that's in line with the held coral.
        //      We dont' use the held coral itself, because the vector from the pivot point to
        //      the held coral risks becoming too small to have a well defined angle as the
        //      robot approaches the reef.
        Transform2d pointInLineWithHeldCoral_robotFrame = new Transform2d(0, getHeldCoralPose_robotFrame().getY(), Rotation2d.kZero);
        Translation2d pointInLineWithHeldCoral_fieldFrame = drivetrain.getPoseMeters().plus(pointInLineWithHeldCoral_robotFrame).getTranslation();

        // 2.75) See how off angle the robot is, then rotate the ideal pose by that amount,
        //       so the axis of the desired coral is in line with the held coral
        Rotation2d radiallyOutwardFromReef = targetBranch.getFace().getOrientation2d();
        Translation2d reefCoralTowardsHeldCoral = pointInLineWithHeldCoral_fieldFrame.minus(pivotPoint_fieldFrame.toTranslation2d());

        // 2.9) Compute and apply the yaw adjustment, log, then we're done.
        Rotation2d yawAdjustment = reefCoralTowardsHeldCoral.getAngle().minus(radiallyOutwardFromReef);

        Pose3d desiredCoralPose_fieldFrame = idealCoralPose_fieldframe.rotateAround(pivotPoint_fieldFrame, new Rotation3d(yawAdjustment));
        Logger.recordOutput("chickenHead/desiredCoralPose_fieldFrame", new Pose3d[] {desiredCoralPose_fieldFrame});
        return desiredCoralPose_fieldFrame;
    }

    private Pose3d robotFrameToFieldFrame(Pose3d poseInRobotFrame) {
        Pose3d robotPose_fieldFrame = new Pose3d(drivetrain.getPoseMeters());
        Transform3d poseAsTransform = new Transform3d(poseInRobotFrame.getTranslation(), poseInRobotFrame.getRotation());
        return robotPose_fieldFrame.plus(poseAsTransform);
    }

    private Pose3d getShoulderPose_robotFrame() {
        double shoulderPitchRadians = -Units.degreesToRadians(arm.getShoulderAngleDegrees()); // negate for wpilib convention of positive pitch = down.
        return new Pose3d(ArmConstants.shoulderLocation_robotFrame, new Rotation3d(0, shoulderPitchRadians, 0));
    }

    private Pose3d getElbowPose_robotFrame() {
        Pose3d shoulderPose_robotFrame = this.getShoulderPose_robotFrame();
        Transform3d elbowPose_shoulderFrame = new Transform3d(ArmConstants.elbowLocation_shoulderFrame, Rotation3d.kZero);
        return shoulderPose_robotFrame.plus(elbowPose_shoulderFrame);
    }

    private Pose3d getWristPose_robotFrame() {
        Pose3d elbowPose_robotFrame = this.getElbowPose_robotFrame();
        Transform3d hardStop_elbowFrame = new Transform3d(ArmConstants.retractionHardStop_elbowFrame, Rotation3d.kZero);
        Transform3d tipOfExtension_hardStopFrame = new Transform3d(arm.getExtensionMeters(), 0, 0, Rotation3d.kZero);
        double wristPitchRelativeToExtension = -Units.degreesToRadians(wrist.getWristAngleDegrees()); // negate for wpilib convention of positive pitch = down.
        Transform3d wristPose_tipOfExtensionFrame = new Transform3d(ArmConstants.tipOfFinalStageToWrist, new Rotation3d(0, wristPitchRelativeToExtension, 0));

        return elbowPose_robotFrame.plus(hardStop_elbowFrame).plus(tipOfExtension_hardStopFrame).plus(wristPose_tipOfExtensionFrame);
    }

    private Pose3d getHeldCoralPose_robotFrame() {
        Pose3d wristPose_robotFrame = this.getWristPose_robotFrame();

        Translation3d coralLocation_wristFrame = ArmConstants.intakeToLeftCoral;
        if (placerGrabber.rightHasCoral()) {
            coralLocation_wristFrame = ArmConstants.intakeToRightCoral;
        }

        Transform3d coralPose_wristFrame = new Transform3d(coralLocation_wristFrame, Rotation3d.kZero);
        return wristPose_robotFrame.plus(coralPose_wristFrame);
    }
}
