// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;
import frc.robot.subsystems.vision.ColorCamera;
import frc.robot.subsystems.vision.TagCameraOld;
import frc.robot.subsystems.vision.testing.PoseEstimatorTest;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private void initAdvantageKit() {
    Logger.recordMetadata("projectName", "2025Robot");
    Logger.addDataReceiver(new NT4Publisher());
    if (Constants.atCompetition) {
        Logger.addDataReceiver(new WPILOGWriter()); // <- log to USB stick
    }
    new PowerDistribution();    // Apparently just constructing a PDH
                                // will allow it's values to be logged? 
                                // This is what the advantage kit docs imply at least.
    Logger.start();
  }


  private final RobotContainer m_robotContainer;

  // TagCameraOld testCam = new TagCameraOld("testCamName", new Transform3d());
  // ColorCamera testColorCam = new ColorCamera("intakeCam", new Pose3d());
  PoseEstimatorTest poseTester = new PoseEstimatorTest();

  public Robot() {
    initAdvantageKit();
    m_robotContainer = new RobotContainer();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();



    
    ArrayList<Pose3d> facePoses = new ArrayList<>();
    ArrayList<Pose3d> stalkPoses = new ArrayList<>();
    ArrayList<Pose3d> branchPoses = new ArrayList<>();

    for (ReefFace face : FieldElement.ALL_REEF_FACES) {
      facePoses.add(face.getPose());
    }
    for (ReefStalk stalk : FieldElement.ALL_STALKS) {
      stalkPoses.add(stalk.getPose());
    }
    for (ReefBranch branch : FieldElement.ALL_BRANCHES) {
      branchPoses.add(branch.getPose());
    }

    Logger.recordOutput("facePoses", facePoses.toArray(new Pose3d[0]));
    Logger.recordOutput("stalkPoses", stalkPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("branchPoses", branchPoses.toArray(new Pose3d[0]));

    // for (int i = 0; i < 1; i += 1) {
    //   testCam.simPeriodic(m_robotContainer.drivetrain.wheelsOnlyPoseEstimator.getEstimatedPosition());
    // }
    // testCam.periodic();
    // testColorCam.periodic();
    poseTester.periodic();

    Pose3d robot1Pose = new Pose3d();
    Pose3d robot2Pose = new Pose3d(new Translation3d(FieldConstants.midField), new Rotation3d());
    Pose3d[] robotPose1;
    Pose3d[] robotPose2;
    Pose3d[] tagsUsed1;
    Pose3d[] tagsUsed2;
    if (DriverStation.isEnabled()) {
      robotPose1 = new Pose3d[] {robot1Pose};
      robotPose2 = new Pose3d[] {robot2Pose};
      tagsUsed1 = new Pose3d[] {FieldElement.FRONT_REEF_FACE.getPose(), FieldElement.FRONT_RIGHT_REEF_FACE.getPose()};
      tagsUsed2 = new Pose3d[] {FieldElement.BARGE.getPose()};
    }
    else {
      robotPose1 = new Pose3d[] {robot1Pose};
      robotPose2 = new Pose3d[0];
      tagsUsed1 = new Pose3d[] {FieldElement.FRONT_REEF_FACE.getPose()};
      tagsUsed2 = new Pose3d[0];
    }

    Logger.recordOutput("pose1Optional", robotPose1);
    Logger.recordOutput("pose2Optional", robotPose2);
    Logger.recordOutput("tagsUsed1", tagsUsed1);
    Logger.recordOutput("tagsUsed2", tagsUsed2);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.drivetrain.setPoseToVisionMeasurement();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
