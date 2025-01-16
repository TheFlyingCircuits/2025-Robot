// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
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

        public double shoulderAccelerationDegreesPerSecondSquared = 0.0;


        public double extensionLengthMeters = 0.0;

        public double extensionLengthMetersPerSecond = 0.0;


        public boolean atUpperAngleLimit = false;
        public boolean atLowerAngleLimit = false;

        public boolean atUpperLengthLimit = false;
        public boolean atLowerLengthLimit = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {};

    public default void setArmAccelerationDegreesPerSecondSquared(double degreesPerSecondSquared) {};

    

    /** 
     * Run the two arm motors at the specified voltage.
     * A positive value will raise the arm, while a negative value will lower it.
    */
    public default void setArmMotorVolts(double volts) {};

    public default void setExtensionMotorVolts(double volts) {};

    


}
