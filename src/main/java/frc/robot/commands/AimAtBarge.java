// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtBarge extends Command {

    Drivetrain drivetrain;
    Arm arm;
    Wrist wrist;
    PlacerGrabber placerGrabber;
    Supplier<ChassisSpeeds> translationController;
    boolean shouldDrive;
    ArmPosition desiredArmPosition;

    public AimAtBarge(Drivetrain drivetrain, Arm arm, Wrist wrist, PlacerGrabber placerGrabber,Supplier<ChassisSpeeds> translationController) {
        this.drivetrain=drivetrain;
        this.arm=arm;
        this.wrist=wrist;
        this.placerGrabber=placerGrabber;
        this.translationController=translationController;
        
        
    }

    public boolean readyToScore() {
        boolean extensionReady = arm.getExtensionMeters() > ArmConstants.maxExtensionMeters - 0.4;

        Logger.recordOutput("aimAtBarge/extensionReady", extensionReady);

        return extensionReady;
    }

    private Pose2d adjustedReefScoringPose() {

        boolean isBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

        double centerLineXMeters = 8.811;

        double centerLineToTapeMeters = 0.59;
        double adjustedXMeters = centerLineToTapeMeters + (DrivetrainConstants.bumperWidthMeters/2) + 0; //final adition is fudge factor
        adjustedXMeters *= isBlueAlliance ? -1 : 1;

        Pose2d scoringPose = new Pose2d(
            centerLineXMeters + adjustedXMeters, 
            drivetrain.getPoseMeters().getY(),
            isBlueAlliance ? Rotation2d.kZero : Rotation2d.k180deg
        );

        Logger.recordOutput("bargeScore/targetDrivePose", scoringPose);
        return scoringPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shouldDrive = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        desiredArmPosition = ArmPosition.backBarge;
        Pose2d targetPose = adjustedReefScoringPose();

        Logger.recordOutput("bargeScore/armDesiredDegrees", desiredArmPosition.shoulderAngleDegrees);
        Logger.recordOutput("bargeScore/wristDesiredDegrees", desiredArmPosition.wristAngleDegrees);
        Logger.recordOutput("bargeScore/extensionDesiredMeters", desiredArmPosition.extensionMeters);


        /**** DRIVETRAIN ALIGNMENT ****/

        ChassisSpeeds driverControl = translationController.get();
        if (Math.hypot(driverControl.vxMetersPerSecond, driverControl.vyMetersPerSecond) < 1) {
            double maxSpeed = 1;
            if (DriverStation.isAutonomous()) {
                maxSpeed = 2.0;
            }
            
            if (shouldDrive)
                drivetrain.pidToPose(targetPose, maxSpeed);
            else drivetrain.fieldOrientedDrive(new ChassisSpeeds(), true);
        }
        else {
            drivetrain.fieldOrientedDrive(driverControl.div(3), true);
        }
        /**** ARM ALIGNMENT ****/

        boolean shoulderNearTarget = Math.abs(arm.getShoulderAngleDegrees() - desiredArmPosition.shoulderAngleDegrees) < 10;

        // immediately start moving shoulder
        arm.setShoulderTargetAngle(desiredArmPosition.shoulderAngleDegrees);

    

        if ((drivetrain.translationControllerAtSetpoint() || !shouldDrive) && shoulderNearTarget) {
            // Only start moving extension & wrist when shoulder is near the setpoint
    
            arm.setExtensionTargetLength(desiredArmPosition.extensionMeters);

            shouldDrive = false;

        }
        else {
            // Stow extension and wrist when the shoulder isn't ready yet
            arm.setExtensionTargetLength(ArmConstants.minExtensionMeters);
            wrist.setTargetPositionDegrees(WristConstants.homeAngleDegrees);
        }



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
