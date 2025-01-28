package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Reefscape.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignWithReef extends Command{
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private FieldElement target;

    /**
     *  @param translationController
     */
    public AlignWithReef(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController, FieldElement target) {
    
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        this.target = target;

        if (translationController != null) {
            super.addRequirements(drivetrain);
        }
    }

    @Override
    public void execute() {
        if (target == FieldElement.ReefFace.FRONT) {
            drivetrain.fieldOrientedDriveOnALine(translationController.get(), FieldElement.ReefFace.FRONT.getPose().toPose2d());
        }
    }
}
