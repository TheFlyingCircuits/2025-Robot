// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupAlgae extends ParallelCommandGroup {

  /** Creates a new PickupAlgae. */
  
    public PickupAlgae(
            boolean isHighAlgae,
            Arm arm,
            PlacerGrabber placerGrabber,
            Drivetrain drivetrain,
            Wrist wrist,
            Supplier<ChassisSpeeds> translationController
        ) {

        addCommands(
            arm.shoulder.setTargetAngleCommand(isHighAlgae ? 65.3: 47),
            new WaitUntilCommand(() -> arm.getShoulderAngleDegrees() > (isHighAlgae ? 60 : 43))
                .andThen(
                    new ParallelCommandGroup(
                        arm.extension.setTargetLengthCommand((isHighAlgae ? 0.92 : 0.71)),
                        wrist.setTargetPositionCommand(isHighAlgae ? -52 : -46)
                    )
                ),
            placerGrabber.setPlacerGrabberVoltsCommand(10, 0),
            drivetrain.run(() -> {
                drivetrain.fieldOrientedDriveOnALine(
                    translationController.get(),
                    drivetrain.getClosestReefFace().getPose2d()
                        .transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)));
        }));
    }


}
