package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.wrist.Wrist;

public class DashboardControlArm extends Command {

    private Arm arm;
    private Wrist wrist;

    public DashboardControlArm(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;

        // intakeWheels aren't added to requirements because we never write to them,
        // we only read from them.
        super.addRequirements(arm.shoulder, arm.extension, wrist);
        super.setName("DashboardControlArmComand");
    }

    @Override
    public void initialize() {
        Logger.recordOutput("dashboardArmControl/enabled", true);
        SmartDashboard.putNumber("dashboardArmControl/desiredShoulderDegrees", arm.getShoulderAngleDegrees());
        SmartDashboard.putNumber("dashboardArmControl/desiredExtensionMeters", arm.getExtensionMeters());
        SmartDashboard.putNumber("dashboardArmControl/desiredWristDegrees", wrist.getWristAngleDegrees());
    }

    @Override
    public void execute() {
        ArmPosition desiredArmState = new ArmPosition();
        desiredArmState.shoulderAngleDegrees = SmartDashboard.getNumber("dashboardArmControl/desiredShoulderDegrees", arm.getShoulderAngleDegrees());
        desiredArmState.extensionMeters = SmartDashboard.getNumber("dashboardArmControl/desiredExtensionMeters", arm.getExtensionMeters());
        desiredArmState.wristAngleDegrees = SmartDashboard.getNumber("dashboardArmControl/desiredWristDegrees", wrist.getWristAngleDegrees());
        this.arm.setShoulderTargetAngle(desiredArmState.shoulderAngleDegrees);
        this.arm.setExtensionTargetLength(desiredArmState.extensionMeters);
        this.wrist.setTargetPositionDegrees(desiredArmState.wristAngleDegrees);
    }

    @Override
    public void end(boolean wasInterrupted) {
        Logger.recordOutput("dashboardArmControl/enabled", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
