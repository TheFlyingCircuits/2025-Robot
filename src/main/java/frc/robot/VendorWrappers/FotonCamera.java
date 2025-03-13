package frc.robot.VendorWrappers;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** TODO: docs */
public class FotonCamera extends PhotonCamera implements LoggableInputs {

    // Cached "sensor" values.
    // TODO: Maybe just put these in ColorCam.java and TagCam.java instead of a seperate wrapper?
    //       Right now, I'm thinking it feels like a lot to add AdvantageKit support directly
    //       in those classes, given they already have a decent level of conceptual difficulty.
    //       I think it might cause a conceptual overload and make the classes too difficult to
    //       navigate, though maybe I'm just really tired right now?
    //
    //       One definite advantage of putting the AdvantageKit stuff directly in ColorCam.java
    //       and TagCam.java is that it would make the inputs tab in advantage scope less verbose?
    //       We could also avoid protobuf logging? Are these really all that important though?

    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    private boolean isConnected = false;

    public FotonCamera(String name) {
        super(name);
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        // AdvantageKit's Deterministic Log Replay requires that data from a sensor
        // is read and cached once at the begining of each main loop (which is done in this.toLog()),
        // and that subsequent requests for sensor readings within the main loop all come
        // from the cache (which is what this function is for).
        //
        // This simplifies logs and eases debugging by ensuring the robot has a single value
        // for each of its sensors at any given moment (where "a given moment" corresponds to
        // a single iteration of our main loop, which we normally think of as a "snapshot in time").
        // There's no need to keep track of a single sensor reporting different values
        // to different subsystems within a single iteration of our main loop.
        return this.latestResult;
    }

    @Override
    public boolean isConnected() {
        return this.isConnected;
    }

    @Override
    public void toLog(LogTable table) {
        // read and cache sensor values
        this.isConnected = super.isConnected();
        this.latestResult = super.getLatestResult(); // TODO: silence warning & add docs.
        if (!this.isConnected) {
            // default to empty results when unplugged,
            // instead of the last valid result.
            //
            // We don't wanna get stuck chasing down the last gamepiece the camera saw
            // when it came unplugged a minute ago! Likewise for pose estimation.
            this.latestResult = new PhotonPipelineResult();
        }

        table.put("isConnected", this.isConnected);
        table.put("latestResult", this.latestResult);
    }

    @Override
    public void fromLog(LogTable table) {
        this.latestResult = table.get("latestResult", this.latestResult);
        this.isConnected = table.get("isConnected", this.isConnected);
    }

    // Maybe a way to work with advantage kit without any logging functions being exposed to the rest of the code?
    // I need to read up on if / when anonymous classes can mutate local variables though...
    // public void periodic() {
    //     Logger.processInputs(cam.getName()+"/mostRecentFrame", new LoggableInputs() {

    //         @Override
    //         public void toLog(LogTable table) {
    //             table.put("timestampSeconds", cachedFrame.timestampSeconds);
    //             table.put("gamepiecesInRobotFrame", cachedFrame.gamepiecesInRobotFrame);
    //         }

    //         @Override
    //         public void fromLog(LogTable table) {
    //             cachedFrame.timestampSeconds = table.get("timestampSeconds", cachedFrame.timestampSeconds);
    //             cachedFrame.gamepiecesInRobotFrame = table.get("gamepiecesInRobotFrame", cachedFrame.gamepiecesInRobotFrame);
    //         }
            
    //     });
    // }



    // private ColorCamFrame getMostRecentFrame() {
    //     // See if we've gotten any new frames since last time
    //     List<PhotonPipelineResult> freshFrames = cam.getAllUnreadResults();
    //     if (freshFrames.size() == 0) {
    //         // If the reason we haven't gotten any new frames is that the camera's unplugged,
    //         // then we certaintly can't see any gamepieces!
    //         if (!cam.isConnected()) {
    //             // timestamp of -1, and an empty list of gamepiece locations.
    //             return new ColorCamFrame(-1, new Translation3d[0]);
    //         } // Design Note: I think the error checking should actually be seperate.
    //           //              getMostRecentFrame() should always just get the most recent frame.
    //           //              Whether or not you want to act on that mostRecentFrame is
    //           //              a different computation. I think this should help cleanup ColorCam.periodic();
    //           //              and make it read better without having to keep track of globals that don't
    //           //              totally feel like they fit / have good names.

    //         // If the reason we haven't gotten any new frames is that we're just waiting
    //         // on the next frame to be processed, we assume that what the camera sees now
    //         // is basically the same as what it saw before, so we don't update the cached frame.
    //         // The robot shouldn't stop tracking gamepieces on a given loop of the robot code
    //         // just because we're waiting for the next frame from the camera. In the meantime,
    //         // we'll just act on the most recent information we had.
    //         return this.prevFrame;
    //     }

    //     // If our coprocessor is running very fast, then PhotonVision may have generated
    //     // multiple results since the last robot loop. However, we only care about the
    //     // most recent frame that was processed because we're only interested in
    //     // where the game pieces are now, and not where they were a second ago!
    //     PhotonPipelineResult mostRecentFrame = freshFrames.get(freshFrames.size()-1);

    //     // Get the location (relative to the robot) of each of the observed gamepieces.
    //     Translation3d[] gamepieceLocations_robotCoords = new Translation3d[mostRecentFrame.targets.size()];
    //     for (int i = 0; i < mostRecentFrame.targets.size(); i += 1) {
    //         // Yaw from PhotonVision is positive to the right, but we use the wpilib convention of positive to the left.
    //         double pitch = Units.degreesToRadians(mostRecentFrame.targets.get(i).pitch);
    //         double yaw = -1 * Units.degreesToRadians(mostRecentFrame.targets.get(i).yaw);
    //         gamepieceLocations_robotCoords[i] = getGamepieceLocationInRobotCoords(pitch, yaw, FieldConstants.coralOuterRadiusMeters);
    //     }

    //     // Add some timestamp info, set the prevFrame for the next time this function is called,
    //     // and then our job is done!
    //     return new ColorCamFrame(mostRecentFrame.getTimestampSeconds(), gamepieceLocations_robotCoords);
    // }
}
