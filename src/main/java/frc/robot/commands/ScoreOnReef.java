package frc.robot.commands;

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
import frc.robot.Constants.UniversalConstants.Direction;
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
    public ScoreOnReef(Drivetrain drivetrain, Arm arm, Wrist wrist, Supplier<ChassisSpeeds> translationController, Supplier<ReefBranch> reefBranch, Leds leds, Supplier<Direction> sideCoralIsIn, Supplier<Boolean> isFacingReef) {
    
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.wrist = wrist;
        this.translationController = translationController;
        this.reefBranch=reefBranch;
        this.leds = leds;
        this.coralSideSupplier=sideCoralIsIn;
        this.isFacingReef = isFacingReef;
        addRequirements(drivetrain, arm.shoulder, arm.extension, wrist);
    }

    public boolean readyToScore() {
        boolean shoulderReady = Math.abs(desiredArmPosition.shoulderAngleDegrees - arm.getShoulderAngleDegrees()) < 1;
        boolean extensionReady = Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02;
        boolean wristReady = Math.abs(desiredArmPosition.wristAngleDegrees - wrist.getWristAngleDegrees()) < 1;
        boolean driveAngleGood = drivetrain.isAngleAligned();
        boolean driveTranslationGood = drivetrain.translationControllerAtSetpoint();

        Logger.recordOutput("scoreOnReef/shoulderReady", shoulderReady);
        Logger.recordOutput("scoreOnReef/extensionReady", extensionReady);
        Logger.recordOutput("scoreOnReef/wristReady", wristReady);
        Logger.recordOutput("scoreOnReef/driveAngleGood", driveAngleGood);
        Logger.recordOutput("scoreOnReef/driveTranslationGood", driveTranslationGood);

        return shoulderReady && extensionReady && wristReady && driveAngleGood && driveTranslationGood;
    }

    public boolean shouldStartEarlyExitTimer() {
        return this.extensionTargetSet && (Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02);
    }


    // bumper 37 inches
    private Pose2d adjustedReefScoringPose(Pose2d stalkPose, Direction sideCoralIsIn, boolean isFacingForward) {
        double adjustedX;
        //17.5 inches of robot space, 4.5 inches is one coral dist
        if (reefBranch.get().getLevel() == 4) {
            adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(17) + Units.inchesToMeters(4.5+1);
        }
        else {
            adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(17) + Units.inchesToMeters(9);
        }

        if (!isFacingForward) {
            adjustedX -= Units.inchesToMeters(1);
        }

        double adjustedY;
        Rotation2d rotation = Rotation2d.fromDegrees(180);

        if (isFacingForward) {
            rotation = Rotation2d.fromDegrees(180);
        } else {
            rotation = new Rotation2d();
        }


        
        if (((sideCoralIsIn == Direction.left) & isFacingForward) || ((sideCoralIsIn == Direction.right) & !isFacingForward)) {
            adjustedY = Units.inchesToMeters(3.25);
        } else {
            adjustedY = -Units.inchesToMeters(3.25);
        }


        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotation);
        return stalkPose.plus(targetPoseToRobotRelativeToStalk);
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
        
        if (isFacingReef.get()) {            
            switch (reefBranch.get().getLevel()) {
                case 1:
                    return new ArmPosition(13, WristConstants.maxAngleDegrees - 5, ArmConstants.minExtensionMeters);
                case 2:
                    //2 coral distance
                    return new ArmPosition(35.3, 98, 0.845);
                case 3:
                    //2 coral distance
                    return new ArmPosition(52.2, 80, 1.08);
                case 4:
                    //1 coral distance
                    return new ArmPosition(71, 35, 1.6);
            }
        }
        else {
            switch (reefBranch.get().getLevel()) {
                case 1:
                    break;
                case 2:
                    break; //no good angle
                case 3:
                    //2 coral distance
                    return new ArmPosition(108, 105, 0.86);
                case 4:
                    //1 coral distance
                    return new ArmPosition(95, 144, 1.59);
            }
        }

        return new ArmPosition();

    }
    
    @Override
    public void initialize() {
        this.extensionTargetSet = false;
        coralSide = coralSideSupplier.get();
        Logger.recordOutput("ScoringOnReef", true);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("ScoringOnReef", false);
    }

    @Override
    public void execute() {
        // System.out.println("scoringOnReef");
        Pose2d targetPose = adjustedReefScoringPose(reefBranch.get().getStalk().getPose2d(), coralSide, isFacingReef.get());

        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        
        ChassisSpeeds driverControl = translationController.get();
        if (Math.hypot(driverControl.vxMetersPerSecond, driverControl.vyMetersPerSecond) < 1) {
            drivetrain.pidToPose(targetPose, 1.5);
        }
        else {
            drivetrain.fieldOrientedDrive(driverControl.div(3), true);
        }


        desiredArmPosition = calculateArmScoringPosition();

        Logger.recordOutput("scoreOnReef/armDesiredDegrees", desiredArmPosition.shoulderAngleDegrees);
        Logger.recordOutput("scoreOnReef/wristDesiredDegrees", desiredArmPosition.wristAngleDegrees);
        Logger.recordOutput("scoreOnReef/extensionDesiredMeters", desiredArmPosition.extensionMeters);


        boolean closeToReef = targetPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 1;
        boolean movingSlow = drivetrain.getSpeedMetersPerSecond() < 1;
        if (closeToReef && movingSlow) {
            arm.setShoulderTargetAngle(desiredArmPosition.shoulderAngleDegrees);

            boolean shoulderNearTarget = Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 10;
            if (shoulderNearTarget) {
                arm.setExtensionTargetLength(desiredArmPosition.extensionMeters);
                this.extensionTargetSet = true;
                
                double maxWristVolts = 8;
                if (reefBranch.get().getLevel() == 4) maxWristVolts = 6;
                wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees, maxWristVolts);
            }
            else {
                this.extensionTargetSet = false;
                arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
                wrist.setTargetPositionDegrees(WristConstants.maxAngleDegrees - 5);
            }
        }
        else {
            this.extensionTargetSet = false;
            arm.setShoulderTargetAngle(45);
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
            wrist.setTargetPositionDegrees(WristConstants.maxAngleDegrees - 5);
        }
        

        // leds.progressBar(arm.getExtensionMeters() / desiredArmPosition.extensionMeters);
    }
}
