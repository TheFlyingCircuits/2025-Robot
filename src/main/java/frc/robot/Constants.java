// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.PlayingField.FieldConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static boolean atCompetition = false;

    public final class UniversalConstants {
        public final static double gravityMetersPerSecondSquared = 9.81;
        public final static double defaultPeriodSeconds = 0.02;

        public final static String canivoreName = "CTRENetwork";

        public enum Direction {
            left,
            right
        }

    }

    public final static class ArmConstants {
        

        public final static double armMassKg = 10;

        /**
         * Vertical distance from the center of the shoulder pivot to the line at the center of the arm.
         */
        public final static double shoulderBracketLengthMeters = -0.2;
        public final static Translation2d shoulderPivotPositionMeters = new Translation2d(-0.4, 0.4);
        
        /** Minimum length of the arm when fully retracted, 0.64 meters*/
        public final static double minExtensionMeters = Units.inchesToMeters(25);
        /** Maximum length of the arm when fully extended, 1.6 meters*/
        public final static double maxExtensionMeters = Units.inchesToMeters(38)+minExtensionMeters;


        /** Rotations of the motor per rotations of the belt pulley */
        public final static double extensionGearReduction = 5;
        /** Radius of the pulley that the extension belt runs on. */
        public static final double extensionPulleyRadiusMeters = Units.inchesToMeters(1.504/2);
        /** Circumference of the pulley that the extension belt runs on. */
        public final static double beltPulleyCircumferenceMeters = 2*extensionPulleyRadiusMeters*Math.PI; 
        /** How far the belt of the extension moves per rotation of the motor. */
        public final static double beltMetersPerMotorRotation = beltPulleyCircumferenceMeters / extensionGearReduction;
        /** How far the tip of the arm extends per rotation of the motor. */
        public final static double extensionMetersPerMotorRotation = beltMetersPerMotorRotation * 2;



        public final static double kSExtensionVolts = 0.25;
        public final static double kGExtensionVolts = 0.52;
        public final static double kVExtensionVoltsSecondsPerRadian = 3.35;
        public final static double kAExtensionVoltsSecondsSquaredPerRadian = 0;
        public final static double kPExtensionVoltsPerMeter = 25.;
        public final static double kDExtensionVoltsPerMeterPerSecond = 1.5;


        /**Minimum angle of the arm, in degrees.*/
        public final static double armMinAngleDegrees = -0.5;
        /**Maximum angle of the arm, in degrees. This value should be positive and greater than 90, as it is beyond the vertical. */
        public final static double armMaxAngleDegrees = 135.0;  



        /**Rotations of the motor per rotations of the arm; a number greater than 1 represents a reduction. */
        public final static double shoulderGearReduction = 246.67;
        


        
        public final static double kSArmVolts = 0.0;
        public final static double kGArmVolts = 0.;
        public final static double kVArmVoltsSecondsPerRadian = 0;
        public final static double kAArmVoltsSecondsSquaredPerRadian = 0;


        //Motor IDs
        public final static int leftShoulderMotorID = 8;
        public final static int rightShoulderMotorID = 9;
        public final static int frontExtensionMotorID = 10;
        public final static int backExtensionMotorID = 11;
        public final static int leftPivotEncoderID = 4;
        public final static int rightPivotEncoderID = 5;
        


        
    }


    public final static class DrivetrainConstants {
        // KINEMATICS CONSTANTS

        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */
        public static final double trackwidthMeters = Units.inchesToMeters(21.75);
        /**
         * Distance between the center point of the front wheels and the center point of the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(21.75);
        /**
         * Distance from the center of the robot to each swerve module.
         */
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0); //0.4177


        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0)
        );

        public static final double frameWidthMeters = Units.inchesToMeters(27);

        public static final double frameWithBumpersWidthMeters = Units.inchesToMeters(27 + 7);



        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight line, based off of the empirical free speed of the drive Krakens.
         */
        public static final double krakenFreeSpeedRPM = 5800;
        public static final double krakenFreeSpeedRotationsPerSecond = krakenFreeSpeedRPM / 60.;
        public static final double maxAchievableVelocityMetersPerSecond = krakenFreeSpeedRotationsPerSecond *
            SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters; // ~5.23 using a theoretical wheel radius of 2 inches m/s
                                                                                                       // ~5.06 when adding 1/16 of an inch of wheel sink into the carpet.
                                                                                                       // ~5.10 using an emperical measurement of wheel radius on fresh wheels.
                                                                                                       // Actual top speed based on testing is ~4.7 m/s
                                                                                                       // (calculating top speed using kv yeilds [12 / 2.42] ~ 4.96 m/s,
                                                                                                       //  but I don't think we can actually achieve this because 
                                                                                                       //  the battery voltage will likely drop below 12 when all drive motors are running)
                                                                                                       // To give ourselves a little breathing room, we use a max speed of 4.5 m/s in auto.

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this speed.
         * <br>
         * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = maxAchievableVelocityMetersPerSecond; 

        /**
         * The maximum achievable angular velocity of the robot in radians per second.
         * <br>
         * This is a measure of how fast the robot can rotate in place, based off of maxAchievableVelocityMetersPerSecond.
         */
        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond / drivetrainRadiusMeters; // Theoretical ~1.93 rotations per second
                                                                                                                                                 // using 4.7 m/s for max linear speed yeilds ~1.79 rotations per second
                                                                                                                                                 // using 4.5 m/s for max linear speed yeilds ~1.71 rotations per second
                                                                                                                                                 // we use 1.0 rotations per second in auto to be extra conservative
                                                                                                                                                 // because any time you're rotating, you're taking away from your translational speed.

        /**
         * This is the max desired angular velocity that will be achievable in teleop.
         * <br>
         * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
         * <br>
         * This value will be tuned based off of driver preference.
         */
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = Units.rotationsToRadians(0.85);


        public static final PathConstraints pathfindingConstraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(360));
    }

    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.175;
        public static final double maxThrottle = 1.0;
    }


    public final static class SwerveModuleConstants {
        /** Rotations of the drive wheel per rotations of the drive motor. */
        public static final double driveGearReduction = (15.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        /** Rotations of the steering column per rotations of the angle motor. */
        public static final double steerGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
        // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
        public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
        public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31

        // PID + FEEDFORWARD CONSTANTS FOR MOTORS
        // PID for drive motors.
        public static final double drivekPVoltsPerMeterPerSecond = 0;
        public static final double drivekIVoltsPerMeter = 0.;
        public static final double drivekDVoltsPerMeterPerSecondSquared = 0.;

        // PID for angle motors.
        public static final double anglekPVoltsPerDegree = 0.08;
        public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk 
        public static final double anglekDVoltsPerDegreePerSecond = 0.;

        public static final double drivekSVolts = 0.2383;
        public static final double drivekVVoltsSecondsPerMeter = 2.52;
        public static final double drivekAVoltsSecondsSquaredPerMeter = 0.;
        
        // Motor configs
        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        
        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    }

    public final static class GyroConstants {
        public static final int pigeonID = 0;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = 0.7333114147186279;
        public static final double mountPosePitchDegrees = -0.11852765083312988;
        public static final double mountPoseRollDegrees = -1.0425487756729126;
    }

    public final static class VisionConstants {

        //Camera, IP, hostname
        //fronLeft, 10.17.87.50, Photon-RPi4-FL
        //frontRight, 10.17.87.51, Photon-RPi4-FR
        //backRight, 10.17.87.52, Photon-RPi4-BR
        //backLeft, 10.17.87.53, Photon-RPi4-BL
        public final static AprilTagFieldLayout aprilTagFieldLayout = FieldConstants.tagLayout;
                                                       

        public final static Transform3d robotToFrontLeft = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.248), Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, -Math.toRadians(8), -Math.toRadians(17.772))
        );

        public final static Transform3d robotToFrontRight = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.248), -Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, -Math.toRadians(8), Math.toRadians(17.772))
        );

        public final static Transform3d robotToBackLeft = new Transform3d(
            new Translation3d(-Units.inchesToMeters(7.248), Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, Math.toRadians(8), Math.toRadians(17.772-180))
        );

        public final static Transform3d robotToBackRight = new Transform3d(
            new Translation3d(-Units.inchesToMeters(7.248), -Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, Math.toRadians(8), Math.toRadians(-17.772+180))
        );

        public final static Transform3d robotToCoralCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(6), Units.inchesToMeters(26.)),
            new Rotation3d(0, Math.toRadians(19), Math.toRadians(-12))
        );

        public final static String[] tagCameraNames = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight"
        };

        public final static Transform3d tagCameraTransforms[] = {
            robotToFrontLeft,
            robotToFrontRight,
            robotToBackLeft,
            robotToBackRight
        };

    }

    public final static class WristConstants {
        public final static double maxAngleDegrees = 162;
        public final static double minAngleDegrees = -10;

        /**rotations of the wrist motor per rotations of the wrist */
        public final static double gearReduction = 9. * (52./12.) * (37./15.);
    }


    public final static class LEDConstants {
        public final static int ledPWMPort = 0;

        //total number of leds
        public final static int ledsPerStrip = 60;
        

        public final static double stripLengthMeters = 1.0;

        public final static double ledsPerMeter = (1.0 * ledsPerStrip) / stripLengthMeters;

        public final static double metersPerLed = 1/ledsPerMeter;

        /**
         * Hues for specific colors
         * Values use the openCV convention where hue ranges from [0, 180)
         */
        public final static class Hues {

            public final static int orangeSignalLight = 4;
            public final static int blueBumpers = 114;
            public final static int redBumpers = 0;
            public final static int redTrafficLight = 0;//0;
            public final static int greenTrafficLight = 40;//60;
            public final static int betweenBlueAndRed = 150; // a purple/pink that's between blue and red.

        }
    }

}
