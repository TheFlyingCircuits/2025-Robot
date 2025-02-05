// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Reefscape.FieldElement;
import frc.robot.Reefscape.FieldElement.ReefBranch;
import frc.robot.Reefscape.FieldElement.ReefFace;
import frc.robot.Reefscape.FieldElement.ReefStalk;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  XboxController controller;



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

  public Robot() {
    initAdvantageKit();
    m_robotContainer = new RobotContainer();
    DriverStation.silenceJoystickConnectionWarning(true);
    controller = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // FieldElementEnum.vizPoses();
    // FieldElementInterface.ReefFace
    ArrayList<Pose3d> facePoses = new ArrayList<>();
    ArrayList<Pose3d> stalkPoses = new ArrayList<>();
    ArrayList<Pose3d> branchPoses = new ArrayList<>();

    for (ReefFace face : FieldElement.ReefFace.values()) {
      facePoses.add(face.getPose());
    }
    for (ReefStalk stalk : FieldElement.ReefStalk.values()) {
      stalkPoses.add(stalk.getPose());
    }
    for (ReefBranch branch : FieldElement.ReefBranch.values()) {
      branchPoses.add(branch.getPose());
    }

    Logger.recordOutput("facePoses", facePoses.toArray(new Pose3d[0]));
    Logger.recordOutput("stalkPoses", stalkPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("branchPoses", branchPoses.toArray(new Pose3d[0]));
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
  public void teleopPeriodic() {
  }

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
