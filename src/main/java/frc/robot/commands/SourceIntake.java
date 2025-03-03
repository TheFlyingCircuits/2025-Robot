package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.wrist.Wrist;

public class SourceIntake extends Command{

    Drivetrain drivetrain;
    Arm arm;
    Wrist wrist;
    Pose2d targetRobotPose2d;

    public SourceIntake(Drivetrain drivetrain, Arm arm, Wrist wrist) {
        this.drivetrain=drivetrain;
        this.arm=arm;
        this.wrist=wrist;
        
    }
    

    @Override
    public void initialize() {
        FieldElement sourceSide = drivetrain.getClosestSourceSide();
        Translation2d sourceTranslation2d = sourceSide.getLocation2d();
        double adjustedX = sourceTranslation2d.getX() + Units.inchesToMeters(18);
        Transform2d robotTransform2d = new Transform2d(adjustedX, 0, new Rotation2d());
        targetRobotPose2d = new Pose2d(robotTransform2d.getX(), robotTransform2d.getY(), new Rotation2d());
    }

    @Override
    public void execute() {
        drivetrain.pidToPose(targetRobotPose2d);
        if (drivetrain.translationControllerAtSetpoint()) {
            double desiredArmAngle = 0;
            double desiredArmExtention = 0;
            double desiredWristAngle = 0;
            arm.setShoulderTargetAngleCommand(desiredArmAngle);
            arm.setExtensionTargetLengthCommand(desiredArmExtention);
            wrist.setTargetPositionDegrees(desiredWristAngle);
        } 
        
    }
}
