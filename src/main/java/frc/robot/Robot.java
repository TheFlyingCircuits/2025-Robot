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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    Timer gcTimer = new Timer();

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

        System.gc();

        initAdvantageKit();
        m_robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
        

        gcTimer.restart();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        m_robotContainer.periodic();


        // System.gc();

        if (gcTimer.advanceIfElapsed(2)) {
            System.gc();
            // System.out.println("Hello world!");
        }
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
        m_autonomousCommand = m_robotContainer.autoChoosingAuto();

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
