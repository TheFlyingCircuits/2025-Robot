// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModuleIOMapleSim implements SwerveModuleIO {

    // reference to module simulation
    private final SwerveModuleSimulation moduleSimulation;
    // reference to the simulated drive motor
    private final SimulatedMotorController.GenericMotorController driveMotor;
    // reference to the simulated turn motor
    private final SimulatedMotorController.GenericMotorController steerMotor;

    public SwerveModuleIOMapleSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures a generic motor controller for drive motor
        // set a current limit of 60 amps
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Current.ofRelativeUnits(60, Amps));
        this.steerMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Current.ofRelativeUnits(60, Amps));
    }

    @Override
    public void setDriveVoltage(double voltage) {
        this.driveMotor.requestVoltage(Volts.of(voltage));
    }

    @Override
    public void setAngleVoltage(double voltage) {
        this.steerMotor.requestVoltage(Volts.of(voltage));
    }
    
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.angleAbsolutePositionDegrees = this.moduleSimulation.getSteerAbsoluteAngle().in(Degrees);
        inputs.driveAppliedVoltage = this.moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveCurrent = this.moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
        inputs.drivePositionMeters = this.moduleSimulation.getDriveWheelFinalPosition().in(Rotation)
            *SwerveModuleConstants.wheelCircumferenceMeters;
        inputs.driveVelocityMetersPerSecond = this.moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond)
            *SwerveModuleConstants.wheelCircumferenceMeters;
    }
}
