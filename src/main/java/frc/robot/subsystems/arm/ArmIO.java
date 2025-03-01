// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.ArmConstants;
public interface ArmIO {

    @AutoLog
    public class ArmIOInputs {
        /**
         * Angle of the arm in degrees. An angle of 0 represents a perfectly horizontal, forward facing arm.
         * Raising the arm from this position will make the angle positive, while lowering it will be negative.
         */
        public double shoulderAngleDegrees = 0.0;
        /**
         * Velocity of the arm in degrees per second.
         * Follows the same direction as armAngleDegrees.
         */
        public double shoulderVelocityDegreesPerSecond = 0.0;

        public double shoulderAppliedVolts = 0.0;
        public double shoulderTorqueCurrent = 0.0;

        public double extensionLengthMeters = ArmConstants.minExtensionMeters;
        public double extensionLengthMetersPerSecond = 0.0;
        public double extensionMetersPerSecondSquared = 0.0;

        public double extensionAppliedVolts = 0.0;
        public double extensionStatorCurrent = 0.0;

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {};

    /** Sets the torquecurrent that the shoulder motors get. A positive value will raise the arm. */
    public default void setShoulderMotorAmps(double amps) {};


    /** 
     * Run the two arm motors at the specified voltage.
     * A positive value will raise the arm, while a negative value will lower it.
    */
    public default void setShoulderMotorVolts(double volts) {};

    /**
     * Sets the voltage that the extension motor gets. A positive value will extend the arm.
     */
    public default void setExtensionMotorVolts(double volts) {};

    public default void setShoulderTargetAngle(double degrees) {};

    public default void setExtensionTargetLength(double meters) {};


}
