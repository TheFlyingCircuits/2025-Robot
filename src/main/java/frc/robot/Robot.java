// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;

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

  public Robot() {
    initAdvantageKit();
    m_robotContainer = new RobotContainer();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // FieldElementEnum.vizPoses();
    // FieldElementInterface.ReefFace
    ArrayList<Pose3d> facePoses = new ArrayList<>();
    ArrayList<Pose3d> stalkPoses = new ArrayList<>();
    ArrayList<Pose3d> branchPoses = new ArrayList<>();

    // MyEnumTest.English.A.sayHi();

    // System.out.println("PRINTING: "+ MyBaseClass.getAll()[0]);

    // System.out.println('\n');
    // System.out.println("Looping through faces to get stalk names: ");
    // for (ExtendedReefFace face : ExtendedReefFace.getAll()) {
    //   System.out.println("face: " + face);
    // }
    // for (ReefFaceEnum faceEnum : InterfaceForEnums.ReefFaceEnum.values()) {
    //   System.out.println("leftStalkName: "+faceEnum.getLeftStalk().getName());
    // }
    // System.out.println("is FieldElement ready?: " + InterfaceForEnums.myPick.getName());
    // System.out.println('\n');

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

    // FieldElement test = FieldElement.NonReefElement.BARGE;
    // FieldElement ugh = FieldElement.ReefFace.handheldTag;
    // FieldElement ugh2 = FieldElement.ReefFace.DemoTarget.handheldTag;
    // // FieldElement.ReefBranch.handheldTag;
    // // I really want handheldTag to be an instance variable of FieldElement, but
    // // it can't have varibles because it's an interface. I can make them
    // // public static final, but then it's accessable through subclasses, which I don't want.

    // int gotcha = MyAbstractClass.myAbstractInt;
    // int hmmm = MyAbstractExtension.myAbstractInt;
    // FieldElement myFieldElement = new MyFieldElement();
    // FieldElement kdkdkdkdk = myFieldElement.handheldTag;
    // FieldElement myOtherElement = FieldElement.ReefFace.BACK_LEFT;
    // FieldElement wrongAndNoWarning = FieldElement.ReefFace.handheldTag;
    // FieldElement unavoidable = FieldElement.ReefFace.BACK.FRONT.FRONT_RIGHT.BACK_LEFT;
    // FieldElementClass maybe = FieldElementClass.BARGE;
    // FieldElementClass baby = FieldElementClass.FRONT_LEFT_REEF_FACE;
    // FieldElementClass;
    // // FieldElementClass.BACK_LEFT_REEF_FACE.
    // FieldElement.ReefStalk.A.reefFace.branches
    // FieldElement dontNeedVerbosity = ReefFace.FRONT_LEFT;
    // myOtherElement.handheldTag;
    // FieldConstants.classTest;
    // // enum for moveableTargets?

    // // What I want
    // // FieldElement target = FieldElement.FRONT_REEF_FACE
    // // or
    // // FieldElement target = FieldElement.ReefFace.FRONT_LEFT;
    // // FieldElement target = FieldElement.handheldTag;

    // // What I don't want
    // // FieldElement.ReefFace.demoTag;
    // // FieldElement.MovingTarget.demoTag;
    // //
    // // if FieldElement is the datatype, and also the home to static instance variables,
    // // then you can't avoid accessing variables in bizarre ways, but it shouldn't come up in autocomplete?
    // // you will at least get sqiggly for using the demo tag on an instance variable.
    // //
    // // doesnt prevent FieldElement.ReefFace.demoTag though...
    // //
    // // What I really want is an enum who's elements can have different types
    // // Part of the focus of enums in java seems to be specifically the ability to enumerate,
    // // which makes sense, but I'm not really using that. Their usecase is more often about
    // // handling all possile cases of something, but we just want to store a bunch of constants.
    // //
    // // One option is to put instances in FieldConstants
    // //
    // // FieldElement myTarget = FieldConstants.FRONT_REEF_FACE;
    // // but that doesn't match the intuitive enum like syntax I'm going for.
    // //
    // // I can have a moveableTarget enum implement FieldElement, and let that enum have a single member
    // // but I don't like the extra indirection
    // //
    // // FieldElement demoTarget = FieldElement.MoveableTarget.HANDHELD_TAG
    // //
    // // That's actually not too bad.
    // //
    // // One other option is to use static methods in Vision.java?
    // // idk how I feedl about that separtion though...
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
