
package frc.robot.subsystems.drivetrain;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionTargetSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.vision.ColorCamera;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputsLogged;
import frc.robot.subsystems.vision.VisionIO.VisionMeasurement;

public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private VisionIO visionIO;
    private VisionIOInputsLogged visionInputs;
    private ColorCamera intakeCam = new ColorCamera("intakeCam", VisionConstants.robotToCoralCamera);

    private SwerveModule[] swerveModules;

    private SwerveDrivePoseEstimator fusedPoseEstimator;
    private SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;

    /** error measured in degrees, output is in degrees per second. */
    private PIDController angleController;

    /** error measured in meters, output is in meters per second. */
    private PIDController translationController;

    /** used to rotate about the intake instead of the center of the robot */
    private Transform2d centerOfRotation_robotFrame = new Transform2d();
    private double intakeX_robotFrame = (DrivetrainConstants.frameWithBumpersWidthMeters / 2.0);
    private Transform2d frontBumper_robotFrame = new Transform2d(intakeX_robotFrame, 0, Rotation2d.kZero);
    private Transform2d effectiveLeftIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, PlacerGrabber.outerWidthMeters/2.0, Rotation2d.kZero));
    private Transform2d effectiveRightIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, -PlacerGrabber.outerWidthMeters/2.0, Rotation2d.kZero));

    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO,
        VisionIO visionIO
    ) {
        this.gyroIO = gyroIO;
        gyroInputs = new GyroIOInputsAutoLogged();

        this.visionIO = visionIO;
        visionInputs = new VisionIOInputsLogged();

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0),
            new SwerveModule(frSwerveModuleIO, 1),
            new SwerveModule(blSwerveModuleIO, 2),
            new SwerveModule(brSwerveModuleIO, 3)
        };

        gyroIO.setRobotYaw(0);

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);

        
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);

        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        wheelsOnlyPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics,
            gyroInputs.robotYawRotation2d,
            getModulePositions(), 
            new Pose2d());

        //angleController = new PIDController(11, 0, 0.5); // kP has units of degreesPerSecond per degree of error.
        angleController = new PIDController(5, 0, 0.3); 
        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(1); // degrees, degreesPerSecond.

        translationController = new PIDController(3.75, 0, 0.1); // kP has units of metersPerSecond per meter of error.
        translationController.setTolerance(0.01, 1.0); // 1 centimeters

        //configPathPlanner();  
    }

    private void configPathPlanner() {

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = new RobotConfig(0, 0, null);
        }

        AutoBuilder.configure(
            this::getPoseMeters, // Robot pose supplier
            (Pose2d dummy) -> {}, // Method to reset odometry (will be called if your auto has a starting pose)
                                  // Note: We never let PathPlanner set the pose, we always seed pose using cameras and apriltags.
            () -> {return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds, DriveFeedforwards ff) -> {this.robotOrientedDrive(speeds, true);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants // These are different from our angleController gain(s), after testing.
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored
              // We by default draw the paths on the red side of the field, mirroring them if we are on the blue alliance.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

        // Register logging callbacks so that PathPlanner data shows up in advantage scope.
        PathPlannerLogging.setLogActivePathCallback( (activePath) -> {
            Logger.recordOutput("PathPlanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback( (targetPose) -> {
            // update the desired angle in the angle controller
            // this is only to allow the LEDs to show progress in auto.
            // The actual angle controller that sends commands in auto is the one from PathPlanner.
            double measuredAngleDegrees = getPoseMeters().getRotation().getDegrees();
            double desiredAngleDegrees = targetPose.getRotation().getDegrees();
            angleController.calculate(measuredAngleDegrees, desiredAngleDegrees);
            Logger.recordOutput("PathPlanner/TrajectorySetpoint", targetPose);
        });
    }


    //**************** DRIVING ****************/


    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
     * @param closedLoop - Whether or not to used closed loop PID control to control the speed of the drive wheels.
    */
    public void robotOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds, centerOfRotation_robotFrame.getTranslation());
        setModuleStates(swerveModuleStates, closedLoop);
    }

    private void enableRotationAroundFrontBumper() {
        this.centerOfRotation_robotFrame = this.frontBumper_robotFrame;
    }
    public void resetCenterOfRotation() {
        this.centerOfRotation_robotFrame = new Transform2d();
    }


    /**
     * Drives the robot at a desired chassis speeds. The coordinate system
     * is the same as the one as the one for setPoseMeters().
     * 
     * @param desiredChassisSpeeds - Field relative chassis speeds, in m/s and rad/s. 
     * @param closedLoop - Whether or not to drive the drive wheels with using feedback control.
     */
    public void fieldOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        Rotation2d currentOrientation = getPoseMeters().getRotation();
        ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds, currentOrientation);
        this.robotOrientedDrive(robotOrientedSpeeds, closedLoop);
    }

    
    /**
     * Drives the robot at a desired chassis speeds, while automatically aiming
     * at a rotation target. The coordinate system is the same as the one as the 
     * one for setPoseMeters().
     * 
     * @param desiredTranslationalSpeeds - Field relative chassis speeds, in m/s. The rotation speed target is not used. 
     * @param desiredAngle - Rotation2d of the target angle to aim at. This angle is CCW positive, with 0 
     * pointing away from the blue alliance wall.
     */
    public void fieldOrientedDriveWhileAiming(ChassisSpeeds desiredTranslationalSpeeds, Rotation2d desiredAngle) {
        // Use PID controller to generate a desired angular velocity based on the desired angle
        double measuredAngle = getPoseMeters().getRotation().getDegrees();
        double desiredAngleDegrees = desiredAngle.getDegrees();
        double desiredRadiansPerSeconds =  Math.toRadians(angleController.calculate(measuredAngle, desiredAngleDegrees));
        if (angleController.atSetpoint()) {
            desiredRadiansPerSeconds = 0;
        }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            desiredTranslationalSpeeds.vxMetersPerSecond,
            desiredTranslationalSpeeds.vyMetersPerSecond,
            desiredRadiansPerSeconds
        );

        this.fieldOrientedDrive(desiredSpeeds, true);
    }

    /**
     * A drive function that's useful for aligning to different field elements.
     * The robot will snap to the line on the field that passes through
     * {@code lineToDriveOn.getTranslation()} and points in the direction of
     * {@code lineToDriveOn.getRotation()}. The driver can still move
     * the robot along this line.
     * @param rawSpeedRequest
     * @param lineToDriveOn
     */
    public void fieldOrientedDriveOnALine(ChassisSpeeds rawSpeedRequest, Pose2d lineToDriveOn) {
        // 0) Extract some data for calculations
        Translation2d pointOnLine = lineToDriveOn.getTranslation();
        Translation2d directionVectorAlongLine = new Translation2d(lineToDriveOn.getRotation().getCos(), lineToDriveOn.getRotation().getSin());

        // 1) Find the vector from the robot's current position on the field to a point on the line
        Translation2d effectiveCenter = getPoseMeters().getTranslation(); //getPoseMeters().plus(centerOfRotation_robotFrame).getTranslation();
        Translation2d vectorFromRobotToAnchor = pointOnLine.minus(effectiveCenter);

        // 2) Split this vector into 2 components, one along the line, and one perpendicular to the line.
        //    Our distance to the line will be the magnitude of the perpendicular component.
        double projectionOntoLine = vectorFromRobotToAnchor.getX() * directionVectorAlongLine.getX() + vectorFromRobotToAnchor.getY() * directionVectorAlongLine.getY();
        Translation2d componentAlongLine = directionVectorAlongLine.times(projectionOntoLine);
        Translation2d componentTowardsLine = vectorFromRobotToAnchor.minus(componentAlongLine);
        double distanceFromRobotToLine = componentTowardsLine.getNorm();

        // 3) Use a proportional controller to decide how quickly we should drive
        //    towards the line based on our perpendicular distance to the line.
        double speedTowardsLine = Math.abs(translationController.calculate(distanceFromRobotToLine, 0));
        if (translationController.atSetpoint()) {
            speedTowardsLine = 0;
        }

        // 4) Find the direction we should travel when driving at that speed
        ChassisSpeeds directionTowardsLine = new ChassisSpeeds();
        if (distanceFromRobotToLine > 0) {
            directionTowardsLine.vxMetersPerSecond = componentTowardsLine.getX() / distanceFromRobotToLine;
            directionTowardsLine.vyMetersPerSecond = componentTowardsLine.getY() / distanceFromRobotToLine;
        }
        
        // 5) Start building the desiredVelocity by moving towards the line
        ChassisSpeeds desiredVelocity = directionTowardsLine.times(speedTowardsLine);

        // 6) Incorporate the driver's requested speeds along the line,
        //    ignoring any requested speeds that are perpendicular to the line.
        projectionOntoLine = rawSpeedRequest.vxMetersPerSecond * directionVectorAlongLine.getX() + rawSpeedRequest.vyMetersPerSecond * directionVectorAlongLine.getY();
        desiredVelocity.vxMetersPerSecond += projectionOntoLine * directionVectorAlongLine.getX();
        desiredVelocity.vyMetersPerSecond += projectionOntoLine * directionVectorAlongLine.getY();

        // 7) Now rotate the robot so it's facing in the same direction as the line
        this.fieldOrientedDriveWhileAiming(desiredVelocity, lineToDriveOn.getRotation());
    }
    
    /**
     * Follows a straight line to a Pose2d on the field. The path is determined by following the max
     * acceleration that the robot can take to reach that pose (similar to a trapezoid profile).
     */
    public void beeLineToPose(Pose2d targetPose) {

        double maxAccel = 2.35; // 2.35 [meters per second per second] (emperically determined)
        // maxAccel = 3.0;

        Translation2d targetLocation = targetPose.getTranslation();
        Translation2d effectiveRobotLocation = getPoseMeters().plus(this.centerOfRotation_robotFrame).getTranslation();
        Translation2d robotToTarget = targetLocation.minus(effectiveRobotLocation);
        double distanceToTarget = robotToTarget.getNorm();


        // Physics 101: under constant accel -> v_final^2 = v_initial^2 + 2 * accel * displacement
        // displacement = finalDistanceToTarget - currentDistanceToTarget = 0 - currentDistanceToTarget
        // accel = maxAccel
        // v_final = 0 (because we want to come to a controlled stop when arriving at the target)
        // after some algebra -> v_initial = sqrt(-2 * accel * displacement)
        double desiredSpeed = Math.sqrt(-2 * maxAccel * (0 - distanceToTarget));

        // direction to drive is towards the target
        Rotation2d directionToDrive = robotToTarget.getAngle();
        Rotation2d directionToPoint = targetPose.getRotation();

        ChassisSpeeds desiredVelocity = new ChassisSpeeds();
        desiredVelocity.vxMetersPerSecond = desiredSpeed * directionToDrive.getCos();
        desiredVelocity.vyMetersPerSecond = desiredSpeed * directionToDrive.getSin();

        // Don't use fieldOrientedDriveOnALine because the direction we want to point the robot
        // may not be the same as the direction to drive in!
        this.fieldOrientedDriveWhileAiming(desiredVelocity, directionToPoint);
    }


    /**
     * Uses PID control to reach a target pose2d.
     * 
     * @param maxSpeedMetersPerSecond - Max speed that the robot will travel at while PID-ing. The output of the translation is clamped to never exceed this value.
     */
    public void pidToPose(Pose2d desired, double maxSpeedMetersPerSecond) {
        Logger.recordOutput("drivetrain/pidSetpointMeters", desired);

        Pose2d current = getPoseMeters();

        Translation2d error = desired.getTranslation().minus(current.getTranslation());

        Logger.recordOutput("drivetrain/pidErrorMeters", error);
        
        double pidOutputMetersPerSecond = -translationController.calculate(error.getNorm(), 0);


        if (translationController.atSetpoint()) {
            pidOutputMetersPerSecond = 0;
        }

        pidOutputMetersPerSecond = MathUtil.clamp(pidOutputMetersPerSecond, -maxSpeedMetersPerSecond, maxSpeedMetersPerSecond);

        double xMetersPerSecond = pidOutputMetersPerSecond*error.getAngle().getCos();
        double yMetersPerSecond = pidOutputMetersPerSecond*error.getAngle().getSin();
        
        fieldOrientedDriveWhileAiming(
            new ChassisSpeeds(
                xMetersPerSecond,
                yMetersPerSecond,
                0
            ),
            desired.getRotation()
        );
    }

    //could be used for a drivetrain command in the future; leave this as its own function
    private void setModuleStates(SwerveModuleState[] desiredStates, boolean closedLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex], closedLoop);
        }
    }


    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveModules) {
            swervePositions[mod.moduleIndex] = mod.getPosition();
        }

        return swervePositions;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveStates = new SwerveModuleState[4];

        for (SwerveModule mod : swerveModules) {
            swerveStates[mod.moduleIndex] = mod.getState();
        }

        return swerveStates;
    }


    //**************** ODOMETRY / POSE ESTIMATION ****************/

    /**
     * Sets the current position of the robot on the field in meters.
     * <p>
     * A positive X value brings the robot towards the red alliance,
     * and a positive Y value brings the robot left as viewed by the blue alliance.
     * Rotations are counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * @param pose
     */
    public void setPoseMeters(Pose2d pose) {
        fusedPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
        wheelsOnlyPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
    }

    /**
     * Gets the current position of the robot on the field in meters, 
     * based off of our odometry and vision estimation.
     * This value considers the origin to be the right side of the blue alliance.
     * <p>
     * A positive X value brings the robot towards the red alliance, and a positive Y value
     * brings the robot towards the left side as viewed from the blue alliance.
     * <p>
     * Rotations are discontinuous counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * 
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return fusedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the rotation reported by the gyro.
     * This rotation is continuous and counterclockwise positive.
     * 
     * This is not necessarily equivalent to the one reported by getPoseMeters(), and it is recommended
     * to use that rotation in almost every case.
     * 
     * This is usable for calibrating the wheel radii, where a continuous angle is required.
     * @return
     */
    public Rotation2d getGyroRotation2d() {
        return gyroInputs.robotYawRotation2d;
    }

    /**
     * Sets the angle of the robot's pose so that it is facing forward, away from your alliance wall. 
     * This allows the driver to realign the drive direction and other calls to our angle.
     */
    public void setRobotFacingForward() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }

        Rotation2d newAngle = Rotation2d.fromDegrees(0);
        if (alliance.get() == Alliance.Red) {
            newAngle = Rotation2d.fromDegrees(180);
        }

        Translation2d location = getPoseMeters().getTranslation();

        setPoseMeters(new Pose2d(location, newAngle));
    }


    /**
     * Takes the best estimated pose from the vision, and sets our current poseEstimator pose to this one.
     */
    public void setPoseToVisionMeasurement() {
        if (visionInputs.visionMeasurements.size() > 0) {
            setPoseMeters(visionInputs.visionMeasurements.get(0).robotFieldPose);
        }
    }

    /**
     * Sets the translation to the best estimated pose from the vision, while rotation remains
     * the same as what is currently being read.
     */
    private void setTranslationToVisionMeasurement() {
        if (!visionInputs.visionMeasurements.isEmpty()) {
            setPoseMeters(
                new Pose2d(
                    visionInputs.visionMeasurements.get(0).robotFieldPose.getTranslation(),
                    getPoseMeters().getRotation()
                )
            );
        }
    }

    public boolean seesTag() {
        return visionInputs.visionMeasurements.size() > 0;
    }


    private void updatePoseEstimator() {
        double totalAccelMetersPerSecondSquared = Math.hypot(gyroInputs.robotAccelX, gyroInputs.robotAccelY);
        totalAccelMetersPerSecondSquared = Math.hypot(totalAccelMetersPerSecondSquared, gyroInputs.robotAccelZ);

        Logger.recordOutput("drivetrain/accelMagnitude", totalAccelMetersPerSecondSquared);

        // if (totalAccelMetersPerSecondSquared > 10) {
        //     hasBeenBumped = true;
        // }

        // if (hasBeenBumped && !visionInputs.visionMeasurements.isEmpty()) {
        //     hasBeenBumped = false;
        //     setTranslationToVisionMeasurement();
        // }

        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());


        List<Pose2d> trackedTags = new ArrayList<Pose2d>();
        for (VisionMeasurement visionMeasurement : visionInputs.visionMeasurements) {


            Translation2d visionTranslation = visionMeasurement.robotFieldPose.getTranslation();
            Translation2d estimatedTranslation = fusedPoseEstimator.getEstimatedPosition().getTranslation();

            // Dont' allow the robot to teleport (Can cause problems when we get bumped)
            double teleportToleranceMeters = 4.0;
            if (visionTranslation.getDistance(estimatedTranslation) > teleportToleranceMeters) { 
                continue;
            }

            // This measurment passes all our checks, so we add it to the fusedPoseEstimator
            fusedPoseEstimator.addVisionMeasurement(
                visionMeasurement.robotFieldPose, 
                visionMeasurement.timestampSeconds, 
                visionMeasurement.stdDevs
            );

            // Log which tags have been used.
            for (int id : visionMeasurement.tagsUsed) {
                Pose2d tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(id).get().toPose2d();
                trackedTags.add(tagPose);
            }
        }

        Logger.recordOutput("drivetrain/trackedTags", trackedTags.toArray(new Pose2d[0]));
    }

    public Translation3d fieldCoordsFromRobotCoords(Translation3d robotCoords) {
        Translation3d robotLocation_fieldFrame = new Translation3d(getPoseMeters().getX(), getPoseMeters().getY(), 0);
        Rotation3d robotOrientation_fieldFrame = new Rotation3d(0, 0, getPoseMeters().getRotation().getRadians());

        return robotCoords.rotateBy(robotOrientation_fieldFrame).plus(robotLocation_fieldFrame);
    }

    public Translation2d fieldCoordsFromRobotCoords(Translation2d robotCoords) {
        return fieldCoordsFromRobotCoords(new Translation3d(robotCoords.getX(), robotCoords.getY(), 0)).toTranslation2d();
    }

    public Translation3d robotCoordsFromFieldCoords(Translation3d fieldCoords) {
        Translation3d robotLocation_fieldFrame = new Translation3d(getPoseMeters().getX(), getPoseMeters().getY(), 0);
        Rotation3d robotOrientation_fieldFrame = new Rotation3d(0, 0, getPoseMeters().getRotation().getRadians());
        Transform3d robotAxesFromFieldPerspective = new Transform3d(robotLocation_fieldFrame, robotOrientation_fieldFrame);
        Transform3d fieldAxesFromRobotPerspecitve = robotAxesFromFieldPerspective.inverse();

        return fieldCoords.rotateBy(fieldAxesFromRobotPerspecitve.getRotation()).plus(fieldAxesFromRobotPerspecitve.getTranslation());
    }

    public Translation2d robotCoordsFromFieldCoords(Translation2d fieldCoords) {
        return robotCoordsFromFieldCoords(new Translation3d(fieldCoords.getX(), fieldCoords.getY(), 0)).toTranslation2d();
    }

    public ChassisSpeeds getFieldOrientedVelocity() {
        ChassisSpeeds robotOrientedSpeeds = DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotOrientedSpeeds, getPoseMeters().getRotation());
    }

    public double getSpeedMetersPerSecond() {
        
        ChassisSpeeds v = getFieldOrientedVelocity();
        double s = Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
        
        return s;
    }

    //**************** TARGET TRACKING ****************/

    /**
     * Returns the best (largest) coral that is valid (within the field boundary and within a certain distance).
     * Returns an empty optional if no such coral is detected.
     */
    public Optional<Translation2d> getBestCoralLocation() {
        Optional<Translation3d> closest = this.getClosestCoralToRobot();
        if (closest.isPresent()) {
            return Optional.of(closest.get().toTranslation2d());
        }
        return Optional.empty();
    }

    public boolean seesAnyCoral() {
        return intakeCam.seesAnyGamepieces();
    }

    public Optional<Translation3d> getClosestCoralToRobot() {
        Optional<Translation3d> closest = intakeCam.getClosestGamepieceTo(getPoseMeters().getTranslation());
        if (closest.isEmpty()) {
            Logger.recordOutput("coralTracking/closestToRobot", new Translation3d[0]);
        }
        else {
            Logger.recordOutput("coralTracking/closestToRobot", new Translation3d[] {closest.get()});
        }
        return closest;
    }
    public Optional<Translation3d> getClosestCoralToIntake(Direction leftOrRight) {
        Transform2d intakePose_robotFrame = this.effectiveLeftIntakePose_robotFrame;
        if (leftOrRight == Direction.right) {
            intakePose_robotFrame = this.effectiveRightIntakePose_robotFrame;
        }
        Pose2d intakePose_fieldFrame = getPoseMeters().plus(intakePose_robotFrame);

        Optional<Translation3d> closest = intakeCam.getClosestGamepieceTo(intakePose_fieldFrame.getTranslation());
        
        String logPrefix = "coralTracking/closestTo";
        String logSuffix = "LeftIntake";
        if (leftOrRight == Direction.right) {
            logSuffix = "RightIntake";
        }

        if (closest.isEmpty()) {
            Logger.recordOutput(logPrefix+logSuffix, new Translation3d[0]);
        }
        else {
            Logger.recordOutput(logPrefix+logSuffix, new Translation3d[] {closest.get()});
        }

        return closest;
    }

    public Optional<Translation3d> getIntendedCoral(ChassisSpeeds requestedVelocity) {
        double vX = requestedVelocity.vxMetersPerSecond;
        double vY = requestedVelocity.vyMetersPerSecond;
        double requestedMetersPerSecond = Math.hypot(vX, vY);

        // Can't determine intention if there's no coral, or if the driver isn't announcing their intention loudly enough
        // (i.e. the angle of the driver's requested velocity isn't firmly defined).
        if (!seesAnyCoral() || (requestedMetersPerSecond < 0.05)) {
            // TODO: maybe just default to closestToIntake if speed is small?
            Logger.recordOutput("coralTracking/bestProjectionOntoRequestedVelocity", new Translation3d[0]);
            return Optional.empty();
        }

        // init target coral as the first one seen.
        Translation2d effectiveRobotLocationOnField = getPoseMeters().plus(this.centerOfRotation_robotFrame).getTranslation();
        Translation3d bestMatchCoral = intakeCam.getValidGamepieces_fieldCoords().get(0);

        // See if there are any other corals that do a better job of matching the driver's intended target.
        for (Translation3d candidateCoral : intakeCam.getValidGamepieces_fieldCoords()) {
            Rotation2d intakeTowardsCandidate = candidateCoral.toTranslation2d().minus(effectiveRobotLocationOnField).getAngle();
            Rotation2d intakeTowardsBestMatch = bestMatchCoral.toTranslation2d().minus(effectiveRobotLocationOnField).getAngle();

            // dot product the driver's requested velocity onto the direction vector towards each coral
            double candidateProjection = vX * intakeTowardsCandidate.getCos() + vY * intakeTowardsCandidate.getSin();
            double bestMatchProjection = vX * intakeTowardsBestMatch.getCos() + vY * intakeTowardsBestMatch.getSin();

            if (candidateProjection > bestMatchProjection) {
                bestMatchCoral = candidateCoral;
            }
        }

        Logger.recordOutput("coralTracking/bestProjectionOntoRequestedVelocity", new Translation3d[] {bestMatchCoral});
        return Optional.of(bestMatchCoral);
    }



    /**
     * Drives towards the given location while pointing the intake at that location
     * @param noteLocation
     */
    //TODO: REMOVE ARGUMENT AND DECIDE ON ONE APPROACH
    public void driveTowardsCoral(Translation2d coralLocation) {

        Translation2d frontOfRobot = fieldCoordsFromRobotCoords(new Translation2d(Units.inchesToMeters(10), 0));
        Translation2d coralToRobot = coralLocation.minus(frontOfRobot);

        Rotation2d directionToPointIn = coralToRobot.getAngle();

        
        //if the coral is far, approach with strafe (otherwise rotate back straight)
        if (coralToRobot.getNorm() > DrivetrainConstants.frameWidthMeters) {
            //if the coral is left side of the robot, rotate left
            if (robotCoordsFromFieldCoords(coralLocation).getAngle().getSin() > 0) 
                directionToPointIn = directionToPointIn.minus(Rotation2d.fromDegrees(10));
            else
                directionToPointIn = directionToPointIn.plus(Rotation2d.fromDegrees(10));
        }

        this.pidToPose(new Pose2d(coralLocation, directionToPointIn), 1.5);
    }

    public void driveTowardsClosestCoralToRobot() {
        // stop if no corals
        Optional<Translation3d> closest = this.getClosestCoralToRobot();
        if (closest.isEmpty()) {
            this.robotOrientedDrive(new ChassisSpeeds(), true);
            Logger.recordOutput("coralTracking/selectedTarget", new Translation3d[0]);
            return;
        }

        // simple straight driving if we see something
        Translation2d coralOnField = closest.get().toTranslation2d();
        Translation2d robotToCoral = coralOnField.minus(getPoseMeters().getTranslation());
        Logger.recordOutput("coralTracking/selectedTarget", new Translation3d[] {closest.get()});

        this.pidToPose(new Pose2d(coralOnField, robotToCoral.getAngle()), 1.5);
    }


    public void driveTowardsClosestCoralToIntake(ChassisSpeeds rawSpeedRequest) {
        // do nothing if no gamepieces found
        if (!this.seesAnyCoral()) {
            // this.robotOrientedDrive(new ChassisSpeeds(), true);
            this.fieldOrientedDrive(rawSpeedRequest, true);
            Logger.recordOutput("coralTracking/intakeAlignmentLine", new Pose2d[0]);
            Logger.recordOutput("coralTracking/selectedTarget", new Translation3d[0]);
            return;
        }

        // gather some data to decide between right and left
        Translation3d closestToLeft = this.getClosestCoralToIntake(Direction.left).get();
        Translation3d closestToRight = this.getClosestCoralToIntake(Direction.right).get();
        Pose2d leftIntakeOnField = getPoseMeters().plus(effectiveLeftIntakePose_robotFrame);
        Pose2d rightIntakeOnField = getPoseMeters().plus(effectiveRightIntakePose_robotFrame);
        double distanceToLeft = leftIntakeOnField.getTranslation().getDistance(closestToLeft.toTranslation2d());
        double distanceToRight = rightIntakeOnField.getTranslation().getDistance(closestToRight.toTranslation2d());

        // make the decision
        Pose2d relevantIntakePose_fieldFrame = leftIntakeOnField; 
        Translation3d targetCoral_fieldFrame = closestToLeft;
        if (distanceToRight < distanceToLeft) {
            relevantIntakePose_fieldFrame = rightIntakeOnField;
            targetCoral_fieldFrame = closestToRight;
        }
        Logger.recordOutput("coralTracking/selectedTarget", new Translation3d[] {targetCoral_fieldFrame});

        // start the tracking
        Translation2d vectorFromIntakeToCoral = targetCoral_fieldFrame.toTranslation2d().minus(relevantIntakePose_fieldFrame.getTranslation());
        Pose2d intakePoseAtPickup = new Pose2d(targetCoral_fieldFrame.toTranslation2d(), vectorFromIntakeToCoral.getAngle());
        Pose2d robotPoseAtPickup = intakePoseAtPickup.plus(effectiveLeftIntakePose_robotFrame.inverse());
        if (distanceToRight < distanceToLeft) {
            robotPoseAtPickup = intakePoseAtPickup.plus(effectiveRightIntakePose_robotFrame.inverse());
        }

        // this.fieldOrientedDriveOnALine(rawSpeedRequest, robotPoseAtPickup);
        this.pidToPose(robotPoseAtPickup, 1.5);
        

        Pose2d lineStart = robotPoseAtPickup.plus(new Transform2d(-20, 0, Rotation2d.kZero));

        Logger.recordOutput("coralTracking/intakeAlignmentLine", new Pose2d[] {lineStart, robotPoseAtPickup});

        // // start the tracking (old)
        // Rotation2d targetOrientation = targetCoral2d.minus(effectiveRobotCenter).getAngle();
        // Pose2d lineThroughEffectiveCenter = new Pose2d(effectiveRobotCenter, targetOrientation);
        // Pose2d anchor = lineThroughEffectiveCenter.plus(new Transform2d(0, -offsetY, Rotation2d.kZero));

        // Pose2d lineToDriveOn = new Pose2d(anchor.getTranslation(), targetOrientation);
        // Logger.recordOutput("assistedIntake/lineToDriveOn", lineToDriveOn);

        // Translation2d lineToCoral = targetCoral_fieldFrame.minus(lineToDriveOn.getTranslation());
        // double distanceAlongLine = lineToCoral.getX() * targetOrientation.getCos() + lineToCoral.getY() * targetOrientation.getSin();
        // Pose2d pickupPose = lineToDriveOn.plus(new Transform2d(distanceAlongLine, 0, Rotation2d.kZero));
        // Logger.recordOutput("assistedIntake/pickupPose", pickupPose);

        // if (DriverStation.isAutonomous()) {
        //     this.pidToPose(pickupPose, 2);
        // }
        // else {
        //     this.fieldOrientedDriveOnALine(rawSpeedRequest, lineToDriveOn);
        // }
    }


    public Command driveTowardsCoralWithForwardRotationCommand(Supplier<ChassisSpeeds> rawSpeedRequest) {
        return this.runOnce(this::enableRotationAroundFrontBumper).andThen(this.driveTowardsCoralCommand(rawSpeedRequest));
    }
    public Command driveTowardsCoralCommand(Supplier<ChassisSpeeds> rawSpeedRequest) { return this.run(() -> {

        // Optional<Translation3d> targetCoral = this.getClosestCoralToRobot();
        // Optional<Translation3d> targetCoral = this.getIntendedCoral(rawSpeedRequest.get());
        // Optional<Translation3d> targetCoral = this.driveTowardsClosestCoralToIntake(rawSpeedRequest.get());

        // Logger.recordOutput("coralTracking/selectedTarget", targetCoral.isPresent() ? (new Translation3d[] {targetCoral.get()}) : (new Translation3d[0]));

        // just regular driving if we can't find an intended target
        if (!this.seesAnyCoral() || true) {
            this.fieldOrientedDrive(rawSpeedRequest.get(), true);
            return;
        }

        // this.driveTowardsClosestCoralToRobot();
        this.driveTowardsClosestCoralToIntake(rawSpeedRequest.get());

        // // aim assist if we've found an intended target
        // Translation2d intakeLocationOnField = getPoseMeters().plus(intakePose_robotFrame).getTranslation();
        // Translation2d intakeToCoral = targetCoral.get().toTranslation2d().minus(intakeLocationOnField);

        // this.fieldOrientedDriveOnALine(rawSpeedRequest.get(), new Pose2d(intakeLocationOnField, intakeToCoral.getAngle()));
        // this.fieldOrientedDriveWhileAiming(rawSpeedRequest.get(), intakeToCoral.getAngle()); // angle spins out as you get close


    }).finallyDo(this::resetCenterOfRotation);}

    public ReefFace getClosestReefFace() {
        ReefFace[] reefFaces = FieldElement.ALL_REEF_FACES;
        ReefFace closestReefFace = reefFaces[0];

        for (ReefFace reefFace : reefFaces) {
            // distance formula 
            double distance = reefFace.getLocation2d().getDistance(getPoseMeters().getTranslation());
            double closestDistance = closestReefFace.getLocation2d().getDistance(getPoseMeters().getTranslation());
            if (distance < closestDistance) {
                closestReefFace = reefFace;
            }
        }

        return closestReefFace;
    }

    public ReefStalk getClosestReefStalk() {
        ReefStalk[] reefStalks = FieldElement.ALL_STALKS;
        ReefStalk closestReefStalk = reefStalks[0];

        for (ReefStalk reefStalk : reefStalks) {
            // distance formula 
            double distance = reefStalk.getLocation2d().getDistance(getPoseMeters().getTranslation());
            double closestDistance = closestReefStalk.getLocation2d().getDistance(getPoseMeters().getTranslation());
            if (distance < closestDistance) {
                closestReefStalk = reefStalk;
            }
        }

        return closestReefStalk;
    }

    public FieldElement getClosestSourceSide() {
        if (FieldElement.LEFT_LOADING_STATION.getLocation2d().getDistance(getPoseMeters().getTranslation()) < 
                FieldElement.RIGHT_LOADING_STATION.getLocation2d().getDistance(getPoseMeters().getTranslation()) ) {
            return FieldElement.LEFT_LOADING_STATION;
        } else {
            return FieldElement.RIGHT_LOADING_STATION;
        }
    }

    public boolean inScoringDistance() {
        Transform2d distanceToNearestStalk = this.getClosestReefFace().getPose2d().minus(this.getPoseMeters());
        return distanceToNearestStalk.getTranslation().getNorm() < 1;
    }



    public boolean isAngleAligned() {
        return angleController.atSetpoint();
    }

    public boolean translationControllerAtSetpoint() {
        return translationController.atSetpoint();
    }

    public double getAngleError() {
        return angleController.getError();
    }

    public double getTranslationError() {
        return translationController.getError();
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        visionIO.updateInputs(visionInputs);
        for (SwerveModule mod : swerveModules)
            mod.periodic();

        if (gyroIO instanceof GyroIOSim) //calculates sim gyro
            gyroIO.calculateYaw(getModulePositions());
          

        Logger.processInputs("gyroInputs", gyroInputs);
        Logger.processInputs("visionInputs", visionInputs);

        updatePoseEstimator();
        intakeCam.periodic(fusedPoseEstimator);


        Logger.recordOutput("drivetrain/fusedPose", fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/wheelsOnlyPose", wheelsOnlyPoseEstimator.getEstimatedPosition());

        Translation2d centerOfRotationOnField = getPoseMeters().plus(this.centerOfRotation_robotFrame).getTranslation();
        Logger.recordOutput("centerOfRotationOnField", new Translation3d(centerOfRotationOnField));

        Logger.recordOutput(
            "drivetrain/swerveModuleStates",
            new SwerveModuleState[] {
              swerveModules[0].getState(),
              swerveModules[1].getState(),
              swerveModules[2].getState(),
              swerveModules[3].getState()
          });

        Logger.recordOutput(
            "drivetrain/swerveModulePositions", 
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            });

        Logger.recordOutput("drivetrain/anglePIDSetpoint", Rotation2d.fromDegrees(angleController.getSetpoint()));
        Logger.recordOutput("drivetrain/isAngleAligned", isAngleAligned());
        Logger.recordOutput("drivetrain/isTranslationAligned", translationControllerAtSetpoint());


        // Coral tracking visualization (logging done in functions)
        this.getClosestCoralToRobot();
        this.getClosestCoralToIntake(Direction.left);
        this.getClosestCoralToIntake(Direction.right);


        Logger.recordOutput("drivetrain/speedMetersPerSecond", getSpeedMetersPerSecond());
    }

    @Override
    public void simulationPeriodic() {
        // Move the simulation forward by 1 timestep
        FieldConstants.simulatedTagLayout.update(wheelsOnlyPoseEstimator.getEstimatedPosition());
        FieldConstants.simulatedCoralLayout.update(wheelsOnlyPoseEstimator.getEstimatedPosition());

        // Log the poses of the simulated gamepieces for visualization in advantage scope.
        Set<VisionTargetSim> simulatedCorals = FieldConstants.simulatedCoralLayout.getVisionTargets("coral");
        List<Pose3d> simCoralPoses = new ArrayList<>();
        for (VisionTargetSim simulatedCoral : simulatedCorals) {
            simCoralPoses.add(simulatedCoral.getPose());

            Translation3d simLocation = simulatedCoral.getPose().getTranslation();
            double lilRotationsPerSecond = 0.25;
            double lilRadiansPerSecond = Units.rotationsToRadians(lilRotationsPerSecond);
            Rotation3d simRotation = new Rotation3d(0, 0, Timer.getTimestamp() * lilRadiansPerSecond);
            simulatedCoral.setPose(new Pose3d(simLocation, simRotation));

        }
        Logger.recordOutput("simulatedCorals", simCoralPoses.toArray(new Pose3d[0]));
    }
}
