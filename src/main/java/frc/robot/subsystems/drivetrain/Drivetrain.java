
package frc.robot.subsystems.drivetrain;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.UniversalConstants.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;
import frc.robot.subsystems.placerGrabber.PlacerGrabber;
import frc.robot.subsystems.vision.ColorCamera;
import frc.robot.subsystems.vision.SingleTagCam;
import frc.robot.subsystems.vision.SingleTagPoseObservation;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputsLogged;

public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private VisionIO visionIO;
    private VisionIOInputsLogged visionInputs;
    private ColorCamera intakeCam = new ColorCamera("intakeCam", VisionConstants.robotToCoralCameraCalibrated);
    private SingleTagCam[] tagCams = {
        new SingleTagCam(VisionConstants.tagCameraNames[0], VisionConstants.tagCameraTransforms[0]), // front left
        new SingleTagCam(VisionConstants.tagCameraNames[1], VisionConstants.tagCameraTransforms[1]), // front right
        new SingleTagCam(VisionConstants.tagCameraNames[2], VisionConstants.tagCameraTransforms[2]), // back left
        new SingleTagCam(VisionConstants.tagCameraNames[3], VisionConstants.tagCameraTransforms[3])  // back right
    };

    //for states minus a front camera
    // private SingleTagCam[] tagCams = {
    //     new SingleTagCam(VisionConstants.tagCameraNames[0], VisionConstants.tagCameraTransforms[0]), // front right
    //     new SingleTagCam(VisionConstants.tagCameraNames[1], VisionConstants.tagCameraTransforms[1]), // back left
    //     new SingleTagCam(VisionConstants.tagCameraNames[2], VisionConstants.tagCameraTransforms[2]), // back right
    //     // new SingleTagCam(VisionConstants.tagCameraNames[3], VisionConstants.tagCameraTransforms[3])  // back right
    // };

    private boolean fullyTrustVisionNextPoseUpdate = false;
    private boolean allowTeleportsNextPoseUpdate = false;
    private boolean hasAcceptablePoseObservationsThisLoop = false;

    private SwerveModule[] swerveModules;

    private SwerveDrivePoseEstimator fusedPoseEstimator;
    private SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;

    /** error measured in degrees, output is in degrees per second. */
    private PIDController angleController;

    /** error measured in meters, output is in meters per second. */
    private PIDController translationController;

    private ProfiledPIDController profiledController;

    private SimpleMotorFeedforward motorFeedForward;

    /** used to rotate about the intake instead of the center of the robot */
    private Transform2d centerOfRotation_robotFrame = new Transform2d();
    private double intakeX_robotFrame = (DrivetrainConstants.bumperWidthMeters / 2.0);
    private Transform2d frontBumper_robotFrame = new Transform2d(intakeX_robotFrame, 0, Rotation2d.kZero);
    private double effectiveIntakeLateralOffset = /*PlacerGrabber.outerWidthMeters/2.0;*/((PlacerGrabber.outerWidthMeters/2.0) + (DrivetrainConstants.bumperWidthMeters/2.0))/2.0;
    private Rotation2d effectiveIntakeOrientation_robotFrame = Rotation2d.fromDegrees(0);
    private Transform2d effectiveLeftIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, effectiveIntakeLateralOffset, effectiveIntakeOrientation_robotFrame));
    private Transform2d effectiveRightIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, -effectiveIntakeLateralOffset, effectiveIntakeOrientation_robotFrame.times(-1)));
    
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
            new SwerveModule(flSwerveModuleIO, 0, "frontLeft"),
            new SwerveModule(frSwerveModuleIO, 1, "frontRight"),
            new SwerveModule(blSwerveModuleIO, 2, "backLeft"),
            new SwerveModule(brSwerveModuleIO, 3, "backRight")
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
        translationController.setTolerance(0.02, 1.0); // meters, metersPerSecond

        SmartDashboard.putData("drivetrain/angleController", angleController);
        SmartDashboard.putData("drivetrain/translationController", translationController);

        profiledController = new ProfiledPIDController(2.8, 0, 0.125, new TrapezoidProfile.Constraints(
            4, 4));
        profiledController.setTolerance(0.01, 0.01);


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

    public void setCenterOfRotation_robotFrame(Translation2d centerInRobotCoords) {
        this.centerOfRotation_robotFrame = new Transform2d(centerInRobotCoords, Rotation2d.kZero);
    }
    public void resetCenterOfRotation() {
        this.centerOfRotation_robotFrame = new Transform2d();
    }
    public void setIntakeToActualSize() {
        double effectiveIntakeLateralOffset = PlacerGrabber.outerWidthMeters/2.0;
        Rotation2d effectiveIntakeOrientation_robotFrame = Rotation2d.fromDegrees(0);
        this.effectiveLeftIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, effectiveIntakeLateralOffset, effectiveIntakeOrientation_robotFrame));
        this.effectiveRightIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, -effectiveIntakeLateralOffset, effectiveIntakeOrientation_robotFrame.times(-1)));
    }
    public void setIntakeToWideSize() {
        double effectiveIntakeLateralOffset = /*PlacerGrabber.outerWidthMeters/2.0;*/((PlacerGrabber.outerWidthMeters/2.0) + (DrivetrainConstants.bumperWidthMeters/2.0))/2.0;
        Rotation2d effectiveIntakeOrientation_robotFrame = Rotation2d.fromDegrees(0);
        this.effectiveLeftIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, effectiveIntakeLateralOffset, effectiveIntakeOrientation_robotFrame));
        this.effectiveRightIntakePose_robotFrame = frontBumper_robotFrame.plus(new Transform2d(0, -effectiveIntakeLateralOffset, effectiveIntakeOrientation_robotFrame.times(-1)));
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
        double desiredDegreesPerSecond = angleController.calculate(measuredAngle, desiredAngleDegrees);
        if (angleController.atSetpoint()) {
            desiredDegreesPerSecond = 0;
        }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            desiredTranslationalSpeeds.vxMetersPerSecond,
            desiredTranslationalSpeeds.vyMetersPerSecond,
            Units.degreesToRadians(desiredDegreesPerSecond)
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
        Translation2d robotLocation = getPoseMeters().getTranslation();
        Translation2d vectorFromRobotToAnchor = pointOnLine.minus(robotLocation);

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

        double maxAccel = 3; // 2.35 [meters per second per second] (emperically determined)
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

        // copy and pasted tollerance from pid to pose

        Translation2d error = targetPose.getTranslation().minus(getPoseMeters().getTranslation());

        double fillerValue = -translationController.calculate(error.getNorm(), 0);

        if (translationController.atSetpoint()) {
            desiredSpeed = 0;
        }

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

    public void resetProfile(Pose2d desired) {

        Pose2d current = getPoseMeters();

        Translation2d error = desired.getTranslation().minus(current.getTranslation());
        profiledController.reset(error.getNorm(), getSpeedMetersPerSecond());
    }

    public void profileToPose(Pose2d desired) {
        Logger.recordOutput("drivetrain/pidSetpointMeters", desired);

        Pose2d current = getPoseMeters();

        Translation2d error = desired.getTranslation().minus(current.getTranslation());

        Logger.recordOutput("drivetrain/pidErrorMeters", error);
        
        double profiledOutputMetersPerSecond = -profiledController.calculate(error.getNorm(), 0)
         - profiledController.getSetpoint().velocity;


        // copy and pasted tollerance from pid to pose

        double fillerValue = -translationController.calculate(error.getNorm(), 0);

        if (translationController.atSetpoint()) {
            profiledOutputMetersPerSecond = 0;
        }


        double xMetersPerSecond = profiledOutputMetersPerSecond*error.getAngle().getCos();
        double yMetersPerSecond = profiledOutputMetersPerSecond*error.getAngle().getSin();
        
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
    public void setOrientation(Rotation2d orientation) {
        // keep location the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(currentPose.getTranslation(), orientation));
    }
    public void setLocation(Translation2d locationOnField) {
        // keep orientation the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(locationOnField, currentPose.getRotation()));
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
        Rotation2d forwardOnRed = Rotation2d.k180deg;
        Rotation2d forwardOnBlue = Rotation2d.kZero;
        Rotation2d forwardNow = getPoseMeters().getRotation();
        this.setOrientation(FlyingCircuitUtils.getAllianceDependentValue(forwardOnRed, forwardOnBlue, forwardNow));
    }


    public void fullyTrustVisionNextPoseUpdate() {
        this.fullyTrustVisionNextPoseUpdate = true;
    }
    public void allowTeleportsNextPoseUpdate() {
        this.allowTeleportsNextPoseUpdate = true;
    }
    public boolean seesAcceptableTag() {
        return this.hasAcceptablePoseObservationsThisLoop;
    }
    private void updatePoseEstimator() {
        // log flags that were set in between last pose update and now
        Logger.recordOutput("drivetrain/fullyTrustingVision", this.fullyTrustVisionNextPoseUpdate);
        Logger.recordOutput("drivetrain/allowingPoseTeleports", this.allowTeleportsNextPoseUpdate);

        // update with wheel deltas
        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());

        // get all pose observations from each camera
        List<SingleTagPoseObservation> allFreshPoseObservations = new ArrayList<>();
        for (SingleTagCam tagCam : tagCams) {
            allFreshPoseObservations.addAll(tagCam.getFreshPoseObservations());
        }

        // process pose obvervations in chronological order
        allFreshPoseObservations.sort(new Comparator<SingleTagPoseObservation>() {
            public int compare(SingleTagPoseObservation a, SingleTagPoseObservation b) {
                return Double.compare(a.timestampSeconds(), b.timestampSeconds());
            } 
        });

        // Filter tags
        List<Pose3d> acceptedTags = new ArrayList<>();
        List<Pose3d> rejectedTags = new ArrayList<>();
        for (SingleTagPoseObservation poseObservation : allFreshPoseObservations) {

            Translation2d observedLocation = poseObservation.robotPose().getTranslation().toTranslation2d();
            Translation2d locationNow = getPoseMeters().getTranslation();

            // reject tags that are too far away
            if (poseObservation.tagToCamMeters() > 5) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // reject tags that are too ambiguous
            if (poseObservation.ambiguity() > 0.2) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // Don't allow the robot to teleport. Disallowing teleports can cause problems when we get bumped
            // and experience lots of wheel slip, which is why we have the "allowTeleportsNextPoseUpdate" flag
            // (used at driver's discretion (typically via y-button)).
            double teleportToleranceMeters = 4.0;
            if (observedLocation.getDistance(locationNow) > teleportToleranceMeters && (!this.allowTeleportsNextPoseUpdate)) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // Don't use any tag that isn't on the reef
            if (!poseObservation.usesReefTag()) {
                rejectedTags.add(poseObservation.getTagPose());
                continue;
            }

            // This measurment passes all our checks, so we add it to the fusedPoseEstimator
            acceptedTags.add(poseObservation.getTagPose());
            Matrix<N3, N1> stdDevs = this.fullyTrustVisionNextPoseUpdate ? VecBuilder.fill(0, 0, 0) : poseObservation.getStandardDeviations();

            fusedPoseEstimator.addVisionMeasurement(
                poseObservation.robotPose().toPose2d(), 
                poseObservation.timestampSeconds(), 
                stdDevs
            );
        }

        // reset flags for next time
        this.fullyTrustVisionNextPoseUpdate = false;
        this.allowTeleportsNextPoseUpdate = false;
        this.hasAcceptablePoseObservationsThisLoop = acceptedTags.size() > 0;

        // log the accepted and rejected tags
        Logger.recordOutput("drivetrain/acceptedTags", acceptedTags.toArray(new Pose3d[0]));
        Logger.recordOutput("drivetrain/rejectedTags", rejectedTags.toArray(new Pose3d[0]));
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

    //**************** CORAL TRACKING ****************/

    public boolean seesAnyCoral() {
        return intakeCam.seesAnyGamepieces();
    }

    public Optional<Pose3d> getClosestCoralToRobot() {
        return intakeCam.getClosestGamepieceTo(getPoseMeters().getTranslation());
    }
    public Optional<Pose3d> getClosestCoralToEitherIntake() {
        if (!this.seesAnyCoral()) {
            return Optional.empty();
        }

        Pose2d leftIntakeOnField = getPoseMeters().plus(effectiveLeftIntakePose_robotFrame);
        Pose2d rightIntakeOnField = getPoseMeters().plus(effectiveRightIntakePose_robotFrame);
        Pose3d closestToLeft = intakeCam.getClosestGamepieceTo(leftIntakeOnField.getTranslation()).get();
        Pose3d closestToRight = intakeCam.getClosestGamepieceTo(rightIntakeOnField.getTranslation()).get();
        double distanceToLeft = leftIntakeOnField.getTranslation().getDistance(closestToLeft.getTranslation().toTranslation2d());
        double distanceToRight = rightIntakeOnField.getTranslation().getDistance(closestToRight.getTranslation().toTranslation2d());

        return (distanceToLeft <= distanceToRight) ? Optional.of(closestToLeft) : Optional.of(closestToRight);
    }
    private Optional<Pose3d> getIntendedCoral(ChassisSpeeds requestedVelocity) {
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

        // init bestMatchCoral as the first one seen.
        Translation2d robotLocation = getPoseMeters().getTranslation();
        Pose3d bestMatchCoral = intakeCam.getValidGamepieces_fieldCoords().get(0);

        // See if there are any other corals that do a better job of matching the driver's intended target.
        for (Pose3d candidateCoral : intakeCam.getValidGamepieces_fieldCoords()) {
            Rotation2d robotTowardsCandidate = candidateCoral.getTranslation().toTranslation2d().minus(robotLocation).getAngle();
            Rotation2d robotTowardsBestMatch = bestMatchCoral.getTranslation().toTranslation2d().minus(robotLocation).getAngle();

            // dot product the driver's requested velocity onto the direction vector towards each coral
            double candidateProjection = vX * robotTowardsCandidate.getCos() + vY * robotTowardsCandidate.getSin();
            double bestMatchProjection = vX * robotTowardsBestMatch.getCos() + vY * robotTowardsBestMatch.getSin();

            if (candidateProjection > bestMatchProjection) {
                bestMatchCoral = candidateCoral;
            }
        }

        Logger.recordOutput("coralTracking/bestProjectionOntoRequestedVelocity", new Pose3d[] {bestMatchCoral});
        return Optional.of(bestMatchCoral);
    }

    public Pose2d getCenteredCoralPickupPose(Pose3d coralPose_fieldCoords) {
        // pickup by driving straight at coral
        Translation2d robotToCoral = coralPose_fieldCoords.getTranslation().toTranslation2d().minus(getPoseMeters().getTranslation());
        return new Pose2d(coralPose_fieldCoords.getTranslation().toTranslation2d() , robotToCoral.getAngle());
    }
    public Pose2d getClosestIntakeToCoral(Pose3d coralPose_fieldCoords) {
        Pose2d leftIntakeOnField = getPoseMeters().plus(effectiveLeftIntakePose_robotFrame);
        Pose2d rightIntakeOnField = getPoseMeters().plus(effectiveRightIntakePose_robotFrame);
        double distanceToLeft = leftIntakeOnField.getTranslation().getDistance(coralPose_fieldCoords.getTranslation().toTranslation2d());
        double distanceToRight = rightIntakeOnField.getTranslation().getDistance(coralPose_fieldCoords.getTranslation().toTranslation2d());
        return (distanceToLeft <= distanceToRight) ? leftIntakeOnField : rightIntakeOnField;
    }
    public Pose2d getOffsetCoralPickupPose(Pose3d coralPose_fieldCoords) {
        // pickup by aligning the intake's left omniwheels or the right omniwheels
        // to the coral (whichever is closer).
        Pose2d intakePose_fieldFrame = this.getClosestIntakeToCoral(coralPose_fieldCoords);
        Translation2d intakeToCoral = coralPose_fieldCoords.getTranslation().toTranslation2d().minus(intakePose_fieldFrame.getTranslation());
        Pose2d intakePoseAtPickup = new Pose2d(coralPose_fieldCoords.getTranslation().toTranslation2d(), intakeToCoral.getAngle());

        Transform2d robotPose_intakeFrame = getPoseMeters().minus(intakePose_fieldFrame);
        Pose2d robotPoseAtPickup = intakePoseAtPickup.plus(robotPose_intakeFrame);

        return robotPoseAtPickup;
    }
    public Pose2d getStrafingPickupPose(Pose3d coralPose_fieldCoords) {
        // Find where the coral is relative to the robot
        Pose2d coralPose_robotCoords = coralPose_fieldCoords.toPose2d().relativeTo(getPoseMeters());

        // Just strafe when the coral gets close to the bumper
        double sideswipeRange = (DrivetrainConstants.bumperWidthMeters/2.0) + ArmConstants.orangeWheels_wristFrame.getX() + (FieldConstants.coralLengthMeters/2.0);
        
        boolean shouldStrafe = (0 <= coralPose_robotCoords.getX()) && (coralPose_robotCoords.getX() <= sideswipeRange);
        if (shouldStrafe) {
            Transform2d pickupPose_robotFrame = new Transform2d(0, coralPose_robotCoords.getY(), Rotation2d.kZero);
            return getPoseMeters().plus(pickupPose_robotFrame);
        }

        // otherwise, approach in the normal fashion to line up the coral with the alignment point on the robot
        return this.getOffsetCoralPickupPose(coralPose_fieldCoords);
    }
    public Pose2d getLollipopPickupPose(Pose3d coralPose_fieldCoords) {
        // Find where the coral is relative to the robot
        Pose2d coralPose_robotCoords = coralPose_fieldCoords.toPose2d().relativeTo(getPoseMeters());

        // Just drive forward when the coral gets close to the bumper
        //sums to ~1.2 meters
        double pickupRange = (DrivetrainConstants.bumperWidthMeters/2.0) + ArmConstants.orangeWheels_wristFrame.getX() + 0.2;
        
        boolean shouldDriveForward = (0 <= coralPose_robotCoords.getX()) && (coralPose_robotCoords.getX() <= pickupRange);

        if (shouldDriveForward) {
            Transform2d pickupPose_robotFrame = new Transform2d(0.5, 0, Rotation2d.kZero); //drive straight forward
            return getPoseMeters().plus(pickupPose_robotFrame);
        }

        // otherwise, approach in the normal fashion to line up the coral with the alignment point on the robot
        return this.getOffsetCoralPickupPose(coralPose_fieldCoords);
    }



    //**************** REEF TRACKING ****************/

    private FieldElement getClosestFieldElement(FieldElement[] fieldElements) {
        // start by assuming the first is closest
        FieldElement closest = fieldElements[0];
        double minDistance = getPoseMeters().getTranslation().getDistance(closest.getLocation2d());

        // see if any other than the first are closer
        for (int i = 1; i < fieldElements.length; i += 1) {
            FieldElement candidate = fieldElements[i];
            double distance = getPoseMeters().getTranslation().getDistance(candidate.getLocation2d());

            if (distance < minDistance) {
                closest = candidate;
                minDistance = distance;
            }
        }

        return closest;
    }
    public ReefFace getClosestReefFace() {
        return (ReefFace) this.getClosestFieldElement(FieldElement.ALL_REEF_FACES);
    }
    public ReefStalk getClosestReefStalk() {
        return (ReefStalk) this.getClosestFieldElement(FieldElement.ALL_STALKS);
    }
    public FieldElement getClosestLoadingStation() {
        return this.getClosestFieldElement(FieldElement.ALL_LOADING_STATIONS);
    }

    public boolean inScoringDistance() {
        Transform2d distanceToNearestStalk = this.getClosestReefFace().getPose2d().minus(this.getPoseMeters());
        return distanceToNearestStalk.getTranslation().getNorm() < 1;
    }
    public boolean isFacingReef() {
        // Find the center of the reef, then get the vector from the robot's
        // current location on the field to the reef.
        Translation2d frontFace = FieldElement.FRONT_REEF_FACE.getLocation2d();
        Translation2d backFace = FieldElement.BACK_REEF_FACE.getLocation2d();
        Translation2d centerOfReef = frontFace.plus(backFace).div(2.0);
        Translation2d robotToReef = centerOfReef.minus(getPoseMeters().getTranslation());

        // Get the direction the robot is pointed in
        Rotation2d robotOrientaion = getPoseMeters().getRotation();

        // The robot is considered to be facing the reef if the projection
        // of [the robot's orientation] onto [the vector from the robot to the reef]
        // is positive.
        double dotProduct = (robotOrientaion.getCos() * robotToReef.getX()) + (robotOrientaion.getSin() * robotToReef.getY());
        return dotProduct > 0;
    }




    public double getAngleError() {
        return angleController.getError();
    }
    public boolean isAngleAligned() {
        return angleController.atSetpoint();
    }

    public double getTranslationError() {
        return translationController.getError();
    }
    public boolean translationControllerAtSetpoint() {
        return translationController.atSetpoint();
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
        // ^^^ intakeCam.periodic() should come after updatePoseEstimator()
        //     so the coral tracking has the most up to date pose info.


        Logger.recordOutput("drivetrain/fusedPose", fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/wheelsOnlyPose", wheelsOnlyPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/speedMetersPerSecond", getSpeedMetersPerSecond());

        Logger.recordOutput("drivetrain/swerveModuleStates", getModuleStates());
        Logger.recordOutput("drivetrain/swerveModulePositions", getModulePositions());

        Logger.recordOutput("drivetrain/anglePIDSetpoint", Rotation2d.fromDegrees(angleController.getSetpoint()));
        Logger.recordOutput("drivetrain/isAngleAligned", isAngleAligned());
        Logger.recordOutput("drivetrain/isTranslationAligned", translationControllerAtSetpoint());

        Translation2d centerOfRotationOnField = getPoseMeters().plus(this.centerOfRotation_robotFrame).getTranslation();
        Logger.recordOutput("drivetrain/centerOfRotationOnField", new Translation3d(centerOfRotationOnField));


        // Coral tracking visualization
        if (seesAnyCoral()) {
            Pose3d closestCoralToRobot = this.getClosestCoralToRobot().get();
            Pose3d closestCoralToIntake = this.getClosestCoralToEitherIntake().get();
            Pose3d closestIntake = new Pose3d(this.getClosestIntakeToCoral(closestCoralToIntake));
            Logger.recordOutput("coralTracking/closestToRobot", new Pose3d[] {closestCoralToRobot});
            Logger.recordOutput("coralTracking/closestToIntake", new Pose3d[] {closestIntake, closestCoralToIntake});
            Logger.recordOutput("coralTracking/centeredPickupPose", new Pose2d[] {this.getCenteredCoralPickupPose(closestCoralToIntake)});
            Logger.recordOutput("coralTracking/offsetPickupPose", new Pose2d[] {this.getOffsetCoralPickupPose(closestCoralToIntake)});
            Logger.recordOutput("coralTracking/strafingPickupPose", new Pose2d[] {this.getStrafingPickupPose(closestCoralToIntake)});
        }
        else {
            Logger.recordOutput("coralTracking/closestToRobot", new Pose3d[0]);
            Logger.recordOutput("coralTracking/closestToIntake", new Pose3d[0]);
            Logger.recordOutput("coralTracking/centeredPickupPose", new Pose2d[0]);
            Logger.recordOutput("coralTracking/offsetPickupPose", new Pose2d[0]);
            Logger.recordOutput("coralTracking/strafingPickupPose", new Pose2d[0]);
        }

        // this.compareCamPoses();
    }

    @Override
    public void simulationPeriodic() {
        // Log the poses of the simulated gamepieces for visualization in advantage scope.
        Set<VisionTargetSim> simulatedCorals = FieldConstants.simulatedCoralLayout.getVisionTargets("coral");
        List<Pose3d> simCoralPoses = new ArrayList<>();
        for (VisionTargetSim simulatedCoral : simulatedCorals) {

            // Translation3d simLocation = simulatedCoral.getPose().getTranslation();
            // double lilRotationsPerSecond = 0.25;
            // double lilRadiansPerSecond = Units.rotationsToRadians(lilRotationsPerSecond);
            // Rotation3d simRotation = new Rotation3d(0, 0, Timer.getTimestamp() * lilRadiansPerSecond);
            // simulatedCoral.setPose(new Pose3d(simLocation, simRotation));

            // Transform2d deadzonePose_robotFrame = this.effectiveLeftIntakePose_robotFrame.plus(new Transform2d(ArmConstants.orangeWheels_wristFrame.getX(), 0, Rotation2d.fromDegrees(45)));
            // Pose2d stuckCoralPoseFieldFrame = getPoseMeters().plus(deadzonePose_robotFrame.plus(new Transform2d(Units.inchesToMeters(1), 0, Rotation2d.kZero)));
            // stuckCoralPoseFieldFrame = getPoseMeters().plus(this.effectiveLeftIntakePose_robotFrame).plus(new Transform2d(0,0,Rotation2d.fromDegrees(45)).plus(new Transform2d(0.0, 0, Rotation2d.kZero)));
            // simulatedCoral.setPose(new Pose3d(stuckCoralPoseFieldFrame).plus(new Transform3d(0,0,FieldConstants.coralOuterRadiusMeters, Rotation3d.kZero)));

            // Pose3d closestFace = new Pose3d(this.getClosestReefFace().getPose2d());
            // double radius = DrivetrainConstants.drivetrainRadiusMeters * 2.5;
            // Transform3d closestFaceToCenter = new Transform3d(radius + Units.inchesToMeters(12), 0, FieldConstants.coralOuterRadiusMeters, Rotation3d.kZero);
            // Translation3d center = closestFace.plus(closestFaceToCenter).getTranslation();

            // double radsPerSecond = Units.rotationsToRadians(1.0/30.0);
            // double rads = radsPerSecond * Timer.getTimestamp();
            // Translation3d rotatingVector = new Translation3d(Math.cos(rads), Math.sin(rads), 0);
            // Rotation3d orientation = new Rotation3d(0, 0, rads + Units.degreesToRadians(90));
            // Pose3d freshPose = new Pose3d(center.plus(rotatingVector.times(radius)), orientation);
            // simulatedCoral.setPose(freshPose);


            simCoralPoses.add(simulatedCoral.getPose());
        }
        Logger.recordOutput("simulatedCorals", simCoralPoses.toArray(new Pose3d[0]));


        // Move the simulation forward by 1 timestep
        FieldConstants.simulatedTagLayout.update(wheelsOnlyPoseEstimator.getEstimatedPosition());
        FieldConstants.simulatedCoralLayout.update(wheelsOnlyPoseEstimator.getEstimatedPosition());
    }

    public boolean simulatedIntakeIsNearCoral(Direction intakeSide) {
        // find where the intake is on the field
        double intakeX_robotFrame = (DrivetrainConstants.bumperWidthMeters/2.0) + (ArmConstants.orangeWheels_wristFrame.getX()/2.0);
        double intakeY_robotFrame = ArmConstants.wristOuterWidthMeters/2.0;
        if (intakeSide == Direction.right) {
            intakeY_robotFrame *= -1;
        }
        Transform2d intakePose_robotFrame = new Transform2d(intakeX_robotFrame, intakeY_robotFrame, Rotation2d.kZero);
        Pose2d intakePose_fieldFrame = getPoseMeters().plus(intakePose_robotFrame);

        // see if any simulated corals are near it
        Set<VisionTargetSim> simulatedCorals = FieldConstants.simulatedCoralLayout.getVisionTargets("coral");
        for (VisionTargetSim simulatedCoral : simulatedCorals) {

            Pose2d coralRelativeToIntake = simulatedCoral.getPose().toPose2d().relativeTo(intakePose_fieldFrame);

            boolean alignedX = Math.abs(coralRelativeToIntake.getX()) < (FieldConstants.coralLengthMeters/2.0);
            boolean alignedY = Math.abs(coralRelativeToIntake.getY()) < Units.inchesToMeters(1);

            if (alignedX && alignedY) {
                return true;
            }
        }

        return false;
    }

    private void compareCamPoses() {
        Pose2d reefFace = FieldElement.FRONT_REEF_FACE.getPose2d();

        double metersToReefFacingEdgeOfTape = Units.inchesToMeters(12);
        double pushOutDistanceMeters = metersToReefFacingEdgeOfTape + (DrivetrainConstants.bumperWidthMeters/2.0);

        Transform2d offset = new Transform2d(pushOutDistanceMeters, 0, Rotation2d.k180deg);
        this.setPoseMeters(reefFace.plus(offset));


        Pose3d robotPose = new Pose3d(this.getPoseMeters());

        for (int i = 0; i < VisionConstants.tagCameraNames.length; i += 1) {
            String name = VisionConstants.tagCameraNames[i];
            Transform3d calibrated = VisionConstants.tagCameraTransforms[i];
            Transform3d uncalibrated = VisionConstants.oldTagCameraTransforms[i];

            Logger.recordOutput("drivetrain/camPoseComparision/"+name+"/calibrated", robotPose.plus(calibrated));
            Logger.recordOutput("drivetrain/camPoseComparision/"+name+"/uncalibrated", robotPose.plus(uncalibrated));
        }
    }
}
