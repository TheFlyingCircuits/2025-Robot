package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;

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
     * Returns true if the fed position is outside of the field.
     * @param toleranceMeters - Distance outside of the field that will still be considered "in the field"; i.e. the method will still return
     * true.
     */
    public static boolean isOutsideOfField(Translation2d pos, double toleranceMeters) {

        return (pos.getY() > 8.19 + toleranceMeters) || (pos.getY() < 0 - toleranceMeters)
            ||(pos.getX() > 16.54 + toleranceMeters) || (pos.getX() < 0 - toleranceMeters);
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
}
