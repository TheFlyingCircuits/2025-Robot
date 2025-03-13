
package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;
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
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        setModuleStates(swerveModuleStates, closedLoop);
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
        Translation2d vectorFromRobotToAnchor = pointOnLine.minus(getPoseMeters().getTranslation());

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
        Translation2d robotLocation = getPoseMeters().getTranslation();
        Translation2d robotToTarget = targetLocation.minus(robotLocation);
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


    public boolean translationControllerAtSetpoint() {
        return translationController.atSetpoint();
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
        Optional<Translation3d> closest = intakeCam.getClosestValidGamepiece();
        if (closest.isPresent()) {
            return Optional.of(closest.get().toTranslation2d());
        }
        return Optional.empty();
    }

    public boolean doesSeeCoral() {
        return getBestCoralLocation().isPresent();
    }

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

    /**
     * Drives towards the given location while pointing the intake at that location
     * @param noteLocation
     */
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


    public boolean isAngleAligned() {
        return angleController.atSetpoint();
    }

    public double getAngleError() {
        return angleController.getError();
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


        // Coral tracking visualization
        if (getBestCoralLocation().isPresent()) {
            Translation2d noteFieldFrame = getBestCoralLocation().get();
            Logger.recordOutput("drivetrain/trackedCoralPose", new Pose2d(noteFieldFrame, new Rotation2d()));
            Logger.recordOutput("drivetrain/trackedCoralDistance", noteFieldFrame.getNorm());
        }
        else {
            Logger.recordOutput("drivetrain/trackedCoralPose", getPoseMeters());
        }


        Logger.recordOutput("drivetrain/speedMetersPerSecond", getSpeedMetersPerSecond());
    }
}
