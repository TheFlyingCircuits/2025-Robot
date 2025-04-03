package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
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
        boolean shoulderReady = Math.abs(desiredArmPosition.shoulderAngleDegrees - arm.getShoulderAngleDegrees()) < 1;
        boolean extensionReady = Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02;
        boolean wristReady = Math.abs(desiredArmPosition.wristAngleDegrees - wrist.getWristAngleDegrees()) < 2;
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


    private Pose2d adjustedReefScoringPose(Pose2d stalkPose, Direction sideCoralIsIn, boolean isFacingForward) {
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
        Pose2d targetPose = adjustedReefScoringPose(reefBranch.get().getStalk().getPose2d(), coralSide, isFacingReef.get());

        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        
        ChassisSpeeds driverControl = translationController.get();
        if (Math.hypot(driverControl.vxMetersPerSecond, driverControl.vyMetersPerSecond) < 1) {
            double maxSpeed = 1;
            if (DriverStation.isAutonomous()) {
                maxSpeed = 2.0;
            }
            drivetrain.pidToPose(targetPose, maxSpeed);
            // drivetrain.fieldOrientedDrive(driverControl.div(3), true);
            // drivetrain.fieldOrientedDriveOnALine(driverControl.div(3.0), targetPose);
        }
        else {
            drivetrain.fieldOrientedDrive(driverControl.div(3), true);
        }


        desiredArmPosition = calculateArmScoringPosition();
        Logger.recordOutput("aimAtReef/armDesiredDegrees", desiredArmPosition.shoulderAngleDegrees);
        Logger.recordOutput("aimAtReef/wristDesiredDegrees", desiredArmPosition.wristAngleDegrees);
        Logger.recordOutput("aimAtReef/extensionDesiredMeters", desiredArmPosition.extensionMeters);

        // immediately start moving shoulder
        arm.setShoulderTargetAngle(desiredArmPosition.shoulderAngleDegrees);

        boolean closeToReef = targetPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 1;
        boolean movingSlow = drivetrain.getSpeedMetersPerSecond() < 2;
        boolean shoulderNearTarget = Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 10;
        if (closeToReef && movingSlow && shoulderNearTarget) {
            // Only start moving extension & wrist when shoulder is near the setpoint
            arm.setExtensionTargetLength(desiredArmPosition.extensionMeters);
            this.extensionTargetSet = true;
                
            double maxWristVolts = 8;
            if (reefBranch.get().getLevel() == 4) maxWristVolts = 6;
            wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees, maxWristVolts);
        }
        else {
            // Stow extension and wrist when the shoulder isn't ready yet
            this.extensionTargetSet = false;
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
            wrist.setTargetPositionDegrees(WristConstants.maxAngleDegrees - 5);
        }

        // leds.progressBar(arm.getExtensionMeters() / desiredArmPosition.extensionMeters);
    }
}
