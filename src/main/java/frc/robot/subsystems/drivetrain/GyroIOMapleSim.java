// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DrivetrainConstants;

/** Add your docs here. */
public class GyroIOMapleSim implements GyroIO {


    GyroSimulation gyroSim;

    public GyroIOMapleSim(GyroSimulation gyroSim) {
        this.gyroSim = gyroSim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.robotYawRotation2d = gyroSim.getGyroReading();
    };

    

    /** Adds angle to yawDegrees..
     * @param angleDegrees - the angle to add to yawDegrees.
     */
    @Override
    public void setRobotYaw(double angleDegrees) {
        gyroSim.setRotation(Rotation2d.fromDegrees(angleDegrees));
    };
}
