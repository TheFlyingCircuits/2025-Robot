package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

public class SourceIntake extends Command{

    Drivetrain drivetrain;
    Arm arm;
    Wrist wrist;
    Pose2d targetRobotPose2d;
    PlacerGrabber placerGrabber;

    public SourceIntake(Drivetrain drivetrain, Arm arm, Wrist wrist, PlacerGrabber placerGrabber) {
        this.drivetrain=drivetrain;
        this.arm=arm;
        this.wrist=wrist;
        this.placerGrabber=placerGrabber;
        
    }
    

    @Override
    public void initialize() {
        FieldElement sourceSide = drivetrain.getClosestSourceSide();
        Transform2d pickupLocationRelativeToSource = new Transform2d(Units.inchesToMeters(18), 0, new Rotation2d());
        targetRobotPose2d = sourceSide.getPose2d().plus(pickupLocationRelativeToSource);
    }

    @Override
    public void execute() {
        drivetrain.pidToPose(targetRobotPose2d);
        if (drivetrain.translationControllerAtSetpoint()) {
            double desiredArmAngle = 0;
            double desiredArmExtention = 0;
            double desiredWristAngle = 0;
            if ((arm.getShoulderAngleDegrees() - 0.5) < desiredWristAngle || (arm.getShoulderAngleDegrees() + 0.5) > desiredWristAngle) {
                arm.setExtensionTargetLength(desiredArmExtention);
                wrist.setTargetPositionDegrees(desiredWristAngle);
            }
            arm.setShoulderTargetAngle(desiredArmAngle);
            placerGrabber.setSideRollerVolts(3);
            placerGrabber.setFrontRollerVolts(3);
            if(placerGrabber.doesHaveCoral()) {
                return;
            }

        } 
        
    }
}
