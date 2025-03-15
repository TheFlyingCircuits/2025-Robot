package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PlayingField.FieldConstants;

public class ColorCamera {

    private final PhotonCamera cam;
    private Translation3d camLocation_robotFrame;
    private Rotation3d camOrientation_robotFrame;

    // "Sensor" readings
    private Optional<Translation3d> closestValidGamepiece = Optional.empty();


    public ColorCamera(String name, Translation3d camLocation_robotFrame, Rotation3d camOrientation_robotFrame) {
        this.cam = new PhotonCamera(name);
        this.camLocation_robotFrame = camLocation_robotFrame;
        this.camOrientation_robotFrame = camOrientation_robotFrame;

        // TODO: customize simSettings instead of using a preset
        //       (not strictly necessary, but could be nice).
        SimCameraProperties simSettings = SimCameraProperties.PI4_LIFECAM_320_240();
        PhotonCameraSim simCamera = new PhotonCameraSim(cam, simSettings);

        // minTargetAreaPixels was set based on the smallest apriltag we should probably detect.
        // IRL, the tags are squares made up of 100 tiles (36 data bit tiles, 28 tiles of black border
        // around the data bits, and then 36 tiles of white border around the black border).
        // Each individual tile on an apriltag should probably land on at least a 3x3 block
        // of pixels in the image plane to be detected reliably.
        int apriltagTiles = 100;
        int pixelsPerTile = 3 * 3;
        simCamera.setMinTargetAreaPixels(apriltagTiles * pixelsPerTile);
        FieldConstants.simulatedCoralLayout.addCamera(simCamera, new Transform3d(camLocation_robotFrame, camOrientation_robotFrame));
    }
    public ColorCamera(String name, Pose3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }
    public ColorCamera(String name, Transform3d camPose_robotFrame) {
        this(name, camPose_robotFrame.getTranslation(), camPose_robotFrame.getRotation());
    }


    /** Returns closest gamepiece that is valid (within the field boundary and within a certain distance of the robot).
     *  Returns an empty optional if no such gamepiece is detected. */
    public Optional<Translation3d> getClosestValidGamepiece() {
        // The actual closestValidGamepiece is calculated and cahced once per
        // main loop in ColorCamera.periodic(). This allows us to call
        // getClosestValidGamepiece() as many times as we want without unecessarily
        // redoing our calculations.
        return this.closestValidGamepiece;
    }


    public void periodic(SwerveDrivePoseEstimator fusedPoseEstimator) {
        // Reset for a new iteration of the main loop
        this.closestValidGamepiece = Optional.empty();
        double metersToClosestGamepiece = -1;
        List<Translation3d> validGamepieces = new ArrayList<>();
        List<Translation3d> invalidGamepieces = new ArrayList<>();
        // Logger.processInputs(cam.getName(), cam);


        // Get the most recent frame from the camera,
        // and find where the robot was when that frame was captured.
        PhotonPipelineResult mostRecentFrame = cam.getLatestResult();
        Optional<Pose2d> interpolatedRobotPose = fusedPoseEstimator.sampleAt(mostRecentFrame.getTimestampSeconds());
        Pose3d robotPoseWhenPicTaken = new Pose3d(interpolatedRobotPose.orElse(fusedPoseEstimator.getEstimatedPosition())); // default to most recent pose if there isn't any pose history.

        // Iterate through all detected gamepieces to find the closest
        // (that's also in the field and not too far away from the robot).
        Pose2d robotPoseNow = fusedPoseEstimator.getEstimatedPosition();
        for (PhotonTrackedTarget target : mostRecentFrame.targets) {
            // Get the gamepiece's location relative to the robot.
            // Yaw from PhotonVision is positive to the right, but we use the wpilib convention of positive to the left.
            double pitch = Units.degreesToRadians(target.pitch);
            double yaw = -1 * Units.degreesToRadians(target.yaw);
            Translation3d gamepieceLocation_robotCoords = getGamepieceLocationInRobotCoords(pitch, yaw, FieldConstants.coralOuterRadiusMeters);

            // robotCoords -> fieldCoords
            Translation3d gamepieceLocation_fieldCoords = gamepieceLocation_robotCoords.rotateBy(robotPoseWhenPicTaken.getRotation()).plus(robotPoseWhenPicTaken.getTranslation());

            // Don't track gamepieces outside the field perimeter.
            double inFieldToleranceMeters = 0.1;
            if (!FlyingCircuitUtils.isInField(gamepieceLocation_fieldCoords, inFieldToleranceMeters)) {
                invalidGamepieces.add(gamepieceLocation_fieldCoords);
                continue;
            }

            // Don't track gamepieces too far away from the robot.
            double maxMetersFromRobot = 2.5;
            double minMetersFromRobot = 0.6;//(DrivetrainConstants.frameWidthMeters / 2.0) + Units.inchesToMeters(8);
            double metersFromRobot = gamepieceLocation_fieldCoords.toTranslation2d().getDistance(robotPoseNow.getTranslation()); // gamepieceLocation_robotCoords.toTranslation2d().getNorm() doesn't take into account any robot travel since the pic was taken.
            if (metersFromRobot > maxMetersFromRobot || metersFromRobot < minMetersFromRobot) {
                invalidGamepieces.add(gamepieceLocation_fieldCoords);
                continue;
            }

                      
            // The gamepiece passes all checks, so now we see if it's closer than our current closest.
            //
            // TODO: In 2024, we always tracked the first note that passed all of our validity checks,
            //       meaning the sort order in photon vision determined the tie breaker between multiple valid notes
            //       (we used biggest first).
            //       While that worked, I'd like to try this method where track the closest of all the valid gamepieces
            //       because it's less error prone (i.e. impervious to forgetting to set the sort order in photon vision),
            //       and it also takes advantage of the new latency compensated field locations of the gamepieces.
            //       However, if this version isn't working, we might go back to the old way, which is why the TODO is here.
            validGamepieces.add(gamepieceLocation_fieldCoords);
            if (this.closestValidGamepiece.isEmpty() || (metersFromRobot < metersToClosestGamepiece)) {
                this.closestValidGamepiece = Optional.of(gamepieceLocation_fieldCoords);
                metersToClosestGamepiece = metersFromRobot;
            }
        }


        // Logging, then we're done!
        String logPrefix = cam.getName()+"/mostRecentFrame/";
        Logger.recordOutput(logPrefix+"timestampSeconds", mostRecentFrame.getTimestampSeconds());
        Logger.recordOutput(logPrefix+"validGamepieces", validGamepieces.toArray(new Translation3d[0]));
        Logger.recordOutput(logPrefix+"invalidGamepieces", invalidGamepieces.toArray(new Translation3d[0]));

        // AdvantageKit doesn't support logging optionals, so we log "closestValidGamepiece"
        // as an array of size 0 when it isn't present, and an array of size 1 when it is present.
        Logger.recordOutput(cam.getName()+"/closestValidGamepiece/location_fieldCoords", (closestValidGamepiece.isPresent()) ? new Translation3d[] {closestValidGamepiece.get()} : new Translation3d[0]);
        Logger.recordOutput(cam.getName()+"/closestValidGamepiece/distanceMeters", metersToClosestGamepiece);
    }


    /**
     * Imagine a vector that points from the camera to a game piece, as viewed from the camera's perspective
     * (i.e. with the wpilib convention of +X = forward (out of the lens), +Y = left, +Z = up).
     * The {@code pitch} and {@code yaw} that are input to this function are the sphereical coordinates
     * of that vector. See https://www.desmos.com/3d/tmsuk0sh1k for a demo!
     * 
     * @param pitchRadians is the angle that vector makes with the camera's XY plane, with positive pitch indicating the game piece
     *                     appears in the top half of the image, and negative pitch indicating the game piece appears
     *                     in the bottom half of the image. In other words {@code pitch = arctan(Z / sqrt(X^2 + Y^2))}.
     * 
     * @param yawRadians is the angle between that vector's projection into the camera's XY plane, and the camera's own X axis.
     *                   Positive yaw indicates the game piece appears in the left half of the image, and negative yaw indicates
     *                   the game piece appears in the right half of the image. In other words {@code yaw = arctan2(Y, X)}
     * 
     * @param gamepieceHeightMeters
     * 
     * @return The full 3D location of the gamepiece in the robot's reference frame.
     */
    private Translation3d getGamepieceLocationInRobotCoords(double pitchRadians_camFrame, double yawRadians_camFrame, double gamepieceHeightMeters) {
        // Use the reported pitch and yaw to calculate a unit vector in the camera
        // frame that points towards the game piece. Note that the pitch has to be negated
        // to convert from the spherical coordinates convention for angles to the
        // right-hand-rule convention for angles.
        Translation3d camTowardsGamepiece_camCoords = new Translation3d(1, new Rotation3d(0, -pitchRadians_camFrame, yawRadians_camFrame));


        // Get the coordinates of that direction vector in the robot's frame.
        // Note that there's no need to incorporate the translation vector between the
        // robot and the camera at this point, because we're transforming a direction
        // vector as opposed to a location vector.
        // (i.e. the gamepiece is to the north-east in the camera's frame, and we need to 
        //  know what direction that corresponds to in the robot's frame. Figuring this out
        //  only requires that you know how the camera is oriented relative to the robot,
        //  and doesn't require knowing where the camera is translated relative to the robot).
        // Another way to realize you don't need the translation for this coordinate transform is to
        // notice that adding a translation vector to a direction vector isn't consistent with units!
        // (i.e. you can't add meters to something that's unitless).
        Translation3d camTowardsGamepiece_robotCoords = camTowardsGamepiece_camCoords.rotateBy(camOrientation_robotFrame);


        // Use the known height of the gamepiece to solve for the distance between the camera and the gamepice
        // vectorFromRobotToCamera + vectorFromCameraToGamepiece = vectorFromRobotToGamepiece
        //
        // vectorFromRobotToCamera + (distanceFromCameraToGamepiece) * directionVectorFromCameraToGamePiece = vectorFromRobotToGamepiece
        //
        // vectorFromRobotToCamera.z + (distanceFromCameraToGamepiece) * directionVectorFromCameraToGamePiece.z = vectorFromRobotToGamepiece.z
        //        (known)                         (unknown)              (known, just calculated in prev step)   (known, the gamepiece's height off the floor)
        //
        double metersFromCameraToGamepiece = (gamepieceHeightMeters - camLocation_robotFrame.getZ()) / camTowardsGamepiece_robotCoords.getZ();
        
        Translation3d camToGamepiece_robotCoords = camTowardsGamepiece_robotCoords.times(metersFromCameraToGamepiece);
        return camLocation_robotFrame.plus(camToGamepiece_robotCoords);
    }





















    // Originally from Drivetrain.java
    // /**
    //  * Returns closest gamepiece that is valid (within the field boundary and within a certain distance of the robot).
    //  * Returns an empty optional if no such gamepiece is detected.
    //  * THE FIELD POSES CAN BE CACHED, BUT THE CLOSEST MUST BE RECOMPUTED EVERY ITERATION BECAUSE ROBOT WILL MOVE WHILE WAITING FOR FRAMES!
    //  * MUST MEASURE DISTANCE TO POSE NOW TO FIND CLOSEST, BUT FIELD POSE OF CORAL COMES FROM WHEN PIC WAS TAKEN!
    //  * THAT COULD ALSO UPDATE BECAUSE OF VISION THOUGH, SO I GUESS EVERYTHING NEEDS TO BE RECOMPUTED ONCE PER MAIN LOOP
    //  * EVEN IF THERE WERE NO NEW FRAMES!
    //  */
    // public Optional<Translation3d> getClosestGamepieceFieldCoords(SwerveDrivePoseEstimator fusedPoseEstimator) {
    //     // keep track of which gamepieces are accepted/rejected for logging/debugging
    //     List<Translation3d> validGamepieces = new ArrayList<>();
    //     List<Translation3d> invalidGamepieces = new ArrayList<>();
    //     Optional<Translation3d> closestValidGamepiece = Optional.empty();
    //     double metersToClosestGamepiece = -1;

    //     // find where the robot was when the cam took the picture, and where the robot is now
    //     Pose3d robotPoseNow = new Pose3d(fusedPoseEstimator.getEstimatedPosition());
    //     Optional<Pose2d> latencyCompensatedRobotPose = fusedPoseEstimator.sampleAt(this.getMostRecentFrame().timestampSeconds);
    //     Pose3d robotPoseWhenPicTaken = latencyCompensatedRobotPose.isPresent() ? new Pose3d(latencyCompensatedRobotPose.get()) : robotPoseNow;


    //     // iterate though each gamepiece that was seen to find the closest one to the robot.
    //     for (Translation3d gamepieceLocation_robotFrame : this.getMostRecentFrame().gamepiecesInRobotFrame) {
    //         // find the location of the gamepiece on the field
    //         Translation3d gamepieceLocation_fieldFrame = gamepieceLocation_robotFrame.rotateBy(robotPoseWhenPicTaken.getRotation()).plus(robotPoseWhenPicTaken.getTranslation());

    //         // don't track gamepieces outside the field perimeter
    //         double inFieldToleranceMeters = 0.5;
    //         if (!FlyingCircuitUtils.isInField(gamepieceLocation_fieldFrame, inFieldToleranceMeters)) {
    //             invalidGamepieces.add(gamepieceLocation_fieldFrame);
    //             continue;
    //         }

    //         // don't track gamepieces too far away from the robot
    //         double maxMetersFromRobot = 2.5;
    //         double metersFromRobot = gamepieceLocation_fieldFrame.toTranslation2d().getDistance(robotPoseNow.getTranslation().toTranslation2d()); // gamepieceLocation_robotFrame.toTranslation2d().getNorm() doesn't take latency compensation into account.
    //         if (metersFromRobot > maxMetersFromRobot) {                                                                                           // we want the distance to the coral now, not in the past when the pic was taken.
    //             invalidGamepieces.add(gamepieceLocation_fieldFrame);
    //             continue;
    //         }
                      
    //         // gamepiece passes all checks, so now we see if it's closer than our current closest
    //         // TODO: in 2024, we just accepted the first note that passed all the checks
    //         //       which meant the note we got was dependent on the sort order in photon vision
    //         //       (we used biggest first).
    //         //       While that worked, I want to try this method where we explicitly get the closest
    //         //       because this is more impervious to forgetting to set the sort order in photon vision,
    //         //       and also uses the new latency compensated pose.
    //         validGamepieces.add(gamepieceLocation_fieldFrame);
    //         if (closestValidGamepiece.isEmpty() || (metersFromRobot < metersToClosestGamepiece)) {
    //             closestValidGamepiece = Optional.of(gamepieceLocation_fieldFrame);
    //             metersToClosestGamepiece = metersFromRobot;
    //         }
    //     }

    //     Logger.recordOutput("validCoralsFieldFrame", validGamepieces.toArray(new Translation3d[0]));
    //     Logger.recordOutput("invalidCoralsFieldFrame", validGamepieces.toArray(new Translation3d[0]));

    //     // AdvantageKit doesn't support logging optionals, so we log "closestValidCoral"
    //     // as an array of size 0 when it isn't present, and an array of size 1 when it is present.
    //     Logger.recordOutput("closestValidCoralFieldFrame", (closestValidGamepiece.isPresent()) ? new Translation3d[] {closestValidGamepiece.get()} : new Translation3d[0]);
    //     Logger.recordOutput("metersToClosestCoral", metersToClosestGamepiece);
    //     return closestValidGamepiece;
    //     // TODO: maybe cache this once in periodic() so we can call
    //     //       it multiple times in the main loop without needlessly
    //     //       recomputing it? Same thing with getClosestReefFace/Stalk too?
    //     //       that would also ensure the logging happens even when not coral tracking
    //     //
    //     // TODO: maybe put this in ColorCamera.java and just take the poseEstimator as input?
    // }



    @SuppressWarnings("unused")
    private void designNotesThatUsedToBeInPeriodic() {
        // Where does "getClosestValidCoralFieldFrame()" go?
        // -> On one hand, it feels like the whole responsibility of the ColorCam is to
        //    find the gamepieces, and getting their field locaitons is an important part of that!
        //    Plus, most other coral-related methods make sense to call on the intakeCam.
        //    Asking if intakeCam.seesCoral() feels more intuitive than asking if drivetrain.seesCoral().
        //    (though having a "seesCoral()" function at all is kinda redundant if we just return optionals for getClosest())
        //    It also feels silly to have the drivetrain pass itself args when drivingTowardsCoral().
        //    Caching the tracked pose so it isn't recomputed multiple times per main loop also feels
        //    like a better fit for ColorCam than drivetrain.
        //    While it may be slightly weird for intakeCam.getClosestCoral() to return the coral
        //    that's closest to the robot rather than the intakeCam, thats nothing that can't be solved
        //    by some more explicit naming, or may not even be a problem that needs to be solved!
        //    It wouldn't fit either drivetrain or intakeCam if we were measuring distance to the intake
        //    itself, so maybe this is a moot point.
        // 
        // -> On the other hand, filtering the corals and finding the closest requires localizing them on the field,
        //    and that that requires info from the drivetrain that would otherwise be declared private (namely the pose estimator).
        //    Its unclear to me what the cleanest way to actually get this information in ColorCam would be...
        //
        //    One option is to pass fusedPoseEstimator to the intakeCam in intakeCam.periodic() (or a more specific name if we go that route).
        //    Consequences -> 1) intakeCam must live in drivetrain, so the main accessors will need to be forwarded through drivetrain.
        //                    2) periodic() could be split in two (making callsite a bit more explicit) because we know
        //                       camera cache-refreshers will only be called once in drivetrain.periodic(). We could do:
        //                    {
        //                        // Cache in camera, forward through drivetrain
        //                        intakeCam.checkForNewFrames();
        //                        intakeCam.findClosestGamepiece(fusedPoseEstimator);
        //
        //                        drivetrain.getClosestCoral() -> {return intakeCam.getClosestCoral()}
        //
        //                        // or
        //
        //                        // Cache in drivetrain
        //                        intakeCam.checkForNewFrames();
        //                        closestCoral = intakeCam.getClosestCoral(fusedPoseEstimator)
        //                        Logger.recordOutput("closestCoral", closestCoral)
        //
        //                        drivetrain.getClosestCoral() -> {return this.closestCoral}
        //                    }
        //
        //                    In both cases, I don't like the amount of indirection and the requirment to have
        //                    multiple methods with basically the same name. Idk how to name the methods so that
        //                    you're not expecting something on the LHS of the assignment...
        //                    I don't like how far you have to go from the callsite to inspect the implementation.
        //                    I'm not currenlty a huge fan of having closestCoral be a property of drivetrain.
        //
        //
        //    Another option is to pass fusedPoseEstimator to the intakeCam's constructor, but this is effectively the same as above.
        //
        //    Another option is to pass a poseSupplier to the intakeCam:
        //    Consequences -> 1) intakeCam doesn't have to live in drivetrain, no accessors need to be forwarded
        //                       (and it can be its own subsystem?).
        //                    2) Code would look like:
        //                    {
        //                        // poseSupplier passed in constructor
        //                        closestCoral = intakeCam.getClosestCoral();
        //                        if (closestCoral.isPresent()) {
        //                            drivetrain.driveTowardsCoral(closetCoral.get());
        //                        }
        //
        //                        // or
        //
        //                        // poseSupplier passed in constructor
        //                        if (intakeCam.seesCoral()) {
        //                            drivetrain.driveTowardsCoral(intakeCam.getClosestCoral());
        //                        }
        //
        //                        // or
        //
        //                        // no supplier needed, but no caching. Nice and explicit tho...
        //                        Pose2d latencyCompensatedPose = drivetrain.getPose(intakeCam.getMostRecentFrame().timestampSeconds);
        //                        Optional<Translation3d> closestCoral = intakeCam.getClosestCoral(latencyCompensatedPose);
        //
        //                        // or
        //
        //                        // no latency compensation, but also no caching
        //                        Optional<Translation3d> closestCoral = intakeCam.getClosestCoral(drivetrain.getPose());
        //                    }
        //
        //                    This option feels like it makes the most sense from a technical perspecive (I think?).
        //                    However, I'm really not sure about passing a poseSupplier. In general, keeping a reference
        //                    to a supplier or consumer feels a little stupid to me. Why not just pass args directly!
        //                    There's one sense in which passing the poseSupplier is akin to allowing the intakeCam
        //                    to subscribe to pose updates from the drivetrain, which feels ok, but I'm still unsure...
        //                    I don't know how to name function objects, because nounifying-verbs feels wrong and cumbersome.
        //
        //                    I can avoid the naming conundrum if I just pass the whole drivetrain into the intakeCam,
        //                    but that definietly feels excessive...
        //
        //                    Implementing the FrameTracker may give a way to avoid the naming conundrum? That's why
        //                    it feels related?
        //
        //
        // Maybe I feel bad because the reference frame stuff isn't all going together?
        // (i.e. getting robotToCoral is seperated from getting fieldToCoral).
        // would it make more sense if I re-wrote it to put all the frame transforms together?
        //
        //
        // Does poseEstimator in robot instead of drivetrain fix the off by one error when resetting pose?
        //
        //
        // Maybe rename "updateCachedFrame()" to "checkForNewFrames()" or something along those lines
        // if I just want to call it directly instead of wrapping in a periodic to be more explicit
        // about what's happening at the callsite? Other options I don't like as much because they aren't
        // as descriptive, but could maybe work: runPipeline(), processFrame()
        //
        // It's ok for the tag cams to take a SwerveDrivePoseEstimator in their constructor,
        // because their whole purpose is to feed it. They need to know who to feed.
        // it acts like the poseConsumer in advantage kit examples, which itself is essentially
        // a subscriber in a pub/sub system.
        // However, it feels a bit odd that pose updates are done magically for the drivetrain at a distance.
        // Normally when you subscribe to something, its some data you need to perform a computation.
        // In this scenario, the drivetrian subscribes to the vision estimates, but its the tag cams that
        // are doing the real work with the data. I think this is why it feels so indirect / weird.
        //
        // It might be better if the (color) camera subscribes to drivetrain poses. The drivetrain pose is something
        // the color camera needs to compute the field pose of its observed game pieces. I guess the question is
        // whether or not that is really the responsibility of the color cam, or if its work ends at finding the
        // robot-relative coords at a point in the past (timestamp).
        // -> I think passing a poseSupplier() to the camera constructor feels weird because its not very
        //    general. We'd never use any input for that other than drivetrain.getPose(). Ever!
        //
        // ********************************
        // pose estimation is a global activity in the sense that multiple robot subsystems
        // need to read it and to write it. Therefore "robotPose" is a property of the robot
        // rather than the drivetrain. It can be passed to other susbsytems as needed.
        // it can directly be querried if the pose estimate has taken a particular tag into
        // account, rather than indirectly asking the cameras. It treats the cameras
        // and wheels as equals. Drivetrain can have its own internal wheels only pose estimator
        // if we want, but its prob better to have it be global so it can stay in sync with resets
        // to the fused one.
        //
        // other subsystems can take the pose estimator as input in their constructors if the pose estimator
        // is a real requirement for those subsystems to do their work (just like any other constructor).
        //
        // As for the cameras, they exist outside of drivetrain. This is good because a single camera
        // could have more than one datastream, and having them be robot members instead of drivetrain members
        // lets them be interfaced with directly.
        //
        // I think this solves all problems and could also work with referenceFrameTracker!
        // SwerveDrivePoseEstimator would be updated to get the pose of the robot.
        // then that pose would be pushed into the referenceFrameTracker to update the whole tree?
        // This last part is kinda fuzzy, but I do think the whole thing will work.
        //
        // However, for the sake of time, I think I'm going to just put the camera stuff inside drivetrain,
        // because that minimizes code changes. I'll keep in mind the possibility that I have a solution for the
        // parts that feel ugly, and I'll come back to it later.
        //
        // I'm also worried about how many of Drivetrain's methods rely on having the pose. And they really
        // do make sense as mostly drivetrian methods! I'm not totally sure what to do about this...
        // ********************************
        //
        //
        // How would my organization change if I had the referenceFrameTracker???
        // I should work off of that so that its easy to transition over if I want
        // to develop that idea later...
        // 
        // If I had that, I think color cam would be its own seperate thing in Robot,
        // while the tag cam would live in drivetrain.
        //
        // Drivetrian can drive towards arbitrary locations. It should get the desired
        // location from an external subsystem, like the coralTracker.
        //
        // whereas the tag cam's whole purpose is to contribute to the pose of the drivetrain,
        // so it makes sense to contain it there. However, the additional functionality
        // I'm considering with setFocus() and lastSightingOf()/hasSeenRecently() makes
        // the tag cams feel like their own distinct thing you'd interact with instead of asking
        // those questions through the drivetrain. I also liked the idea of the drive encoders
        // acting as an equivalent sensor that's on par with the cameras for deciding the robot's
        // pose, and managing the pose info at the RobotContainer level, which is then fed info
        // by both the drivetrain and cameras as sensors. That info could then be forwarded to
        // all the other mechanisims via frameTracker.
        //
        // there's also the option that the drivetrain maintains one seperate pose estimator per tag
        //
        // Maybe it all goes in drivetrain because ultimately the whole point of vision is
        // to figure out where things are, and the drivetrain is all about going to diferent places?
        //
        // what if setPose could take in a pose and a timestamp (i.e. expose addVisionMeasurement() through
        // drivetrain.setPose())? would that be a good / clean interface that makes sense?
        // internally drivetrain will adjust its pose history and wheel deltas
        // to use the given pose as an anchor (i.e. call addVisionMeasurement())?
        //
        // What about if we had tracking functions like beelineToPose or driveWhileAiming
        // or driveOnALine that were aware of the field element we were scoring on?
        // I think something like that could work. This has too much inference, and can go wrong.
        // It needs to be indirect.
        //
        // What if the fieldelements themselves had properties like when they were last seen
        // that were populated by the vision subsystem? (i.e. FieldElement.PROCESSOR.hasSeenRecently())
        // That doesn't totally feel right though, because the field elements don't need to know about the
        // cameras to be useful. I like that they're just plain pose information.
        //
        // At first I was thinking frameTracker would be responsible for tracking drivetrain pose,
        // but after reading ROS docs, I do think its a better idea to have it track 
        // just the relationships between poses, and each individual subsystem is responsible for
        // its own part of the chain. i.e. it manages its immediate neighbors?
        //
        // Should pipelines be a consideration (one camera capturing multiple kinds of data?)
        //
        // Should multiple cameras be a consideration (array of them in Vision.java, appearing
        // as a single camera for each purpose to Drivetrain.java for convenience?)
        // Wouldn't really matter with UFCS or if we were in control of the photonvison interface
        // over network tables.



        // private Pose3dTimeline robotPose; // FunctionFromDoubleToPose, FunctionFromTimestampToPose, Function<Double, Pose>, FunctionThatGetsTheRobotsPoseAtAGivenTimestamp, etc... How much info goes in the type vs the variable name? Types can have multiple instances, but here there's really only one instance. Would the type just be the same as the variable name?
        
        // @FunctionalInterface
        // public interface Pose3dTimeline { // TODO: naming things is hard
        //     public Pose3d sampleAt(double timestampSeconds);
        // }
        // @FunctionalInterface
        // public interface  Pose2dTimeline { // TODO: naming things is hard
        //     public Pose2d sampleAt(double timestampSeconds);
        // }
    }
    
}