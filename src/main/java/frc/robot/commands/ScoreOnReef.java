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
    private Supplier<String> coralSideSupplier;
    private String coralSide;
    private Supplier<Boolean> isFacingForward;
    private ArmPosition desiredArmPosition;

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
        this.coralSideSupplier=sideCoralIsIn;
        this.isFacingForward=isFacingReef;
        addRequirements(drivetrain, arm.shoulder, arm.extension, wrist);
    }

    public boolean readyToScore() {
        if ((Math.abs(desiredArmPosition.shoulderAngleDegrees - arm.getShoulderAngleDegrees())) < 0.6 && 
                (Math.abs(desiredArmPosition.extensionMeters - arm.getExtensionMeters()) < 0.02) &&
                    (Math.abs(desiredArmPosition.wristAngleDegrees - wrist.getWristAngleDegrees()) < 0.5) && 
                        drivetrain.isAligned() &&
                            drivetrain.translationControllerAtSetpoint()) {
            return true;
        }
        return false;
    }

    // bumper 37 inches
    private Pose2d adjustedReefScoringPose(Pose2d stalkPose, String sideCoralIsIn, boolean isFacingForward) {
        double adjustedX;
        //17.5 inches of robot space, 4.5 inches is one coral dist
        if (reefBranch.get().getLevel() == 4) {
            adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(17) + Units.inchesToMeters(4.5); 
        }
        else {
            adjustedX = FieldConstants.stalkInsetMeters + Units.inchesToMeters(17) + Units.inchesToMeters(9);
        }
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
                //2 coral distance
                return new ArmPosition(35.3, 98, 0.845);
            case 3:
                //2 coral distance
                return new ArmPosition(52.2, 80, 1.08);
            case 4:
                //1 coral distance
                return new ArmPosition(70.5, 35, 1.6);
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
    public void initialize() {
        coralSide = coralSideSupplier.get();
    }

    @Override
    public void execute() {
        Pose2d targetPose = adjustedReefScoringPose(reefBranch.get().getStalk().getPose2d(), coralSide, isFacingForward.get());

        //drivetrain.fieldOrientedDriveOnALine(translationController.get(), new Pose2d(targetPose.getTranslation(), adjustedRotation));
        drivetrain.pidToPose(targetPose, 1);

        desiredArmPosition = calculateArmScoringPosition();

        Logger.recordOutput("scoreOnReef/armDesiredDegrees", desiredArmPosition.shoulderAngleDegrees);
        Logger.recordOutput("scoreOnReef/wristDesiredDegrees", desiredArmPosition.wristAngleDegrees);
        Logger.recordOutput("scoreOnReef/extensionDesiredMeters", desiredArmPosition.extensionMeters);

        arm.setShoulderTargetAngle(desiredArmPosition.shoulderAngleDegrees);
        
        if (Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 10) {
            arm.setExtensionTargetLength(desiredArmPosition.extensionMeters);
        }
        else {
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
        }


        // if (Math.abs(arm.getExtensionMeters() - desiredArmPosition.extensionMeters) < 0.1) {
        //     wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees);
        // }
        if (Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 10) {
            wrist.setTargetPositionDegrees(desiredArmPosition.wristAngleDegrees, 8);
        }
        else {
            wrist.setTargetPositionDegrees(WristConstants.maxAngleDegrees - 5, 8);
        }
        

        leds.progressBar(arm.getExtensionMeters() / desiredArmPosition.extensionMeters);
    }
}
