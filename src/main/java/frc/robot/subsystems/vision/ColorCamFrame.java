package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Translation3d;

public class ColorCamFrame implements LoggableInputs {

    public double timestampSeconds = -1;
    public List<Translation3d> gamepieceLocations_robotCoords = new ArrayList<>();
    public List<Translation3d> validGamepieceLocations_fieldCoords = new ArrayList<>();
    public List<Translation3d> invalidGamepieceLocations_fieldCoords = new ArrayList<>();
    public Optional<Translation3d> closestValidGamepiece_fieldCoords = Optional.empty();
    public double metersToClosestGamepiece = -1;

    private PhotonPipelineResult rawDataFromPhotonVision = new PhotonPipelineResult();

    public ColorCamFrame(PhotonPipelineResult rawDataFromPhotonVision) {
        this.rawDataFromPhotonVision = rawDataFromPhotonVision;
        this.timestampSeconds = rawDataFromPhotonVision.getTimestampSeconds();
    }

    

    @Override
    public void toLog(LogTable table) {
        table.put("timestampSeconds", timestampSeconds);
        table.put("gamepieceLocations_robotCoords", gamepieceLocations_robotCoords.toArray(new Translation3d[0]));
        table.put("validGamepieceLocations_fieldCoords", validGamepieceLocations_fieldCoords.toArray(new Translation3d[0]));
        table.put("invalidGamepieceLocations_fieldCoords", invalidGamepieceLocations_fieldCoords.toArray(new Translation3d[0]));

        // AdvantageKit doesn't support logging optionals, so we log "closestValidGamepiece"
        // as an array of size 0 when it isn't present, and an array of size 1 when it is present.
        table.put("closestValidGamepiece/locationFieldCoords", closestValidGamepiece_fieldCoords.isPresent() ? new Translation3d[] {closestValidGamepiece_fieldCoords.get()} : new Translation3d[0]);
        table.put("closestValidGamepiece/distanceMeters", metersToClosestGamepiece);
    }

    @Override
    public void fromLog(LogTable table) {
        timestampSeconds = table.get("timestampSeconds", timestampSeconds);
        gamepieceLocations_robotCoords = Arrays.asList(table.get("gamepieceLocations_robotCoords", new Translation3d[0]));
        validGamepieceLocations_fieldCoords = Arrays.asList(table.get("validGamepieceLocations_fieldCoords", new Translation3d[0]));
        invalidGamepieceLocations_fieldCoords = Arrays.asList(table.get("invalidGamepieceLocations_fieldCoords", new Translation3d[0]));

        // AdvantageKit doesn't support logging optionals, so we log "closestValidGamepiece"
        // as an array of size 0 when it isn't present, and an array of size 1 when it is present.
        Translation3d[] optionalAsArray = table.get("closestValidGamepiece/locationFieldCoords", new Translation3d[0]);
        closestValidGamepiece_fieldCoords = optionalAsArray.length > 0 ? Optional.of(optionalAsArray[0]) : Optional.empty();

        metersToClosestGamepiece = table.get("closestValidGamepiece/distanceMeters", metersToClosestGamepiece);
    }
    
}
