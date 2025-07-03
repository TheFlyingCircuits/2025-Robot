// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {

    private Command autoCommand = null;
    private Timer gcTimer = new Timer();
    private final RobotContainer m_robotContainer;


    public Robot() {

        System.gc();

        initAdvantageKit();
        m_robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
        SmartDashboard.putData(CommandScheduler.getInstance());

        gcTimer.restart();
    }


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


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        m_robotContainer.periodic();

        if (gcTimer.advanceIfElapsed(5)) {
            // System.gc();
        }

        if (Robot.isSimulation()) {
            SimulatedArena.getInstance().simulationPeriodic();
        }
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

        m_robotContainer.drivetrain.allowTeleportsNextPoseUpdate();
        m_robotContainer.drivetrain.fullyTrustVisionNextPoseUpdate();

        if (autoCommand == null) {
            autoCommand = m_robotContainer.autoChoosingAuto();
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (autoCommand == null) {
            return;
        }
        autoCommand.schedule();;
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autoCommand == null) {
            return;
        }
        autoCommand.cancel();
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
