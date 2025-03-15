package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.PlayingField.FieldConstants;

public class FlyingCircuitUtils {

    /**
     * Generates a field relative pose for the closest pickup for auto-intaking a note
     * by drawing a straight line to the note.
     * Once the robot is at this position, the robot should be
     * able to track the note itself.
     * @param robot - Current translation of the robot.
     * @param note - Translation of the target note.
     * @param radiusMeters - Distance from the note of the output pose.
     */
    public static Pose2d pickupAtNote(Translation2d robot, Translation2d note, double radiusMeters) {
        //vector pointing from the note to the robot
        Translation2d noteToBot = robot.minus(note);

        Translation2d targetTranslation = note.interpolate(robot, radiusMeters/noteToBot.getNorm());

        return new Pose2d(targetTranslation, noteToBot.getAngle());
    }

    /**
     * Util method to create a path following command given the name of the path in pathplanner.
     * Make sure to call this after the AutoBuilder is configured.
     */
    public static Command followPath(String pathName) {
        System.out.println("getting path: " + pathName);
        try {
             return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)).withName(pathName);
        }
        catch (IOException e) {
            System.out.println("IOException when reading path name");
            return null; 
        }
        catch (ParseException e) {
            System.out.println("ParseExeption when reading path name");
            return null;
        }
    }

    /**
     * Returns true if the fed position is inside the field perimeter.
     * @param toleranceMeters - Distance outside of the field that will still be considered "in the field";
     *                          i.e. the method will still return true.
     */
    public static boolean isInField(Translation2d location, double toleranceMeters) {
        // Modeling the field as a long octagon instead of just a rectangle is necessary
        // for accurately rejecting the coral that are outside the field at the loading stations.
        //
        // When using a tolerance, the current calculation isn't totally correct at the corners of the octagon
        // (see https://www.desmos.com/calculator/xr9ocyj96n for an illustration of the innaccuracy),
        // but it should still be close enough for our purposes.
        int[] cornerTagIDs = {1, 2, 12, 13};
        for (int tagID : cornerTagIDs) {
            // Get vector from tag to location
            Pose2d tagPose = FieldConstants.tagLayout.getTagPose(tagID).get().toPose2d();
            Translation2d tagToLocation = location.minus(tagPose.getTranslation());

            // Project that onto the tag's normal vector to get signed distance from the loading station wall.
            // Negative values indicate out of the field, because the tag's normal faces into the field.
            Rotation2d tagNormal = tagPose.getRotation();
            double signedDistance = tagToLocation.getX() * tagNormal.getCos() + tagToLocation.getY() * tagNormal.getSin();

            if (signedDistance < -toleranceMeters) {
                return false;
            }
        }

        // we've passed all the loading station checks if we've gotten to this point,
        // so all that's left to check is the length and width of the field.
        boolean insideX = ((0 - toleranceMeters) < location.getX()) && (location.getX() < (FieldConstants.maxX + toleranceMeters));
        boolean insideY = ((0 - toleranceMeters) < location.getY()) && (location.getY() < (FieldConstants.maxY + toleranceMeters));
        return insideX && insideY;
    }
    public static boolean isInField(Translation3d location, double toleranceMeters) {
        return FlyingCircuitUtils.isInField(location.toTranslation2d(), toleranceMeters);
    }


    public static double getPlanarDistanceMeters(Translation3d locaitonA, Translation3d locationB) {
        return locaitonA.toTranslation2d().getDistance(locationB.toTranslation2d());
    }



    public static <T> T getAllianceDependentValue(T valueOnRed, T valueOnBlue, T valueWhenNoComms) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return valueOnRed;
            }
            if (alliance.get() == Alliance.Blue) {
                return valueOnBlue;
            }
        }

        // Should never get to this point as long as we're connected to the driver station.
        return valueWhenNoComms;
    }

    /** Positive to the left of the line, negative to the right of the line */
    public static double signedDistanceToLine(Translation2d point, Pose2d line) {
        Translation2d anchor = line.getTranslation();
        Translation2d anchorToPoint = point.minus(anchor);

        // normalToLine = new Translation2d(-line.getRotation().getSin(), line.getRotation().getCos());
        double normalToLineX = -line.getRotation().getSin();
        double normalToLineY =  line.getRotation().getCos();

        // project onto line normal to get signed distance (same as <directionVectorAlongLine> cross <anchorToPoint>)
        return anchorToPoint.getX() * normalToLineX + anchorToPoint.getY() * normalToLineY;
    }

    // public String getBigCommandName(Command command) {
    //     String name = command.toString();
    //     command.raceWith(null)
    //     if (command instanceof )
    // }
}
