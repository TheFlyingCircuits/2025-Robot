// package frc.robot;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.PlayingField.ReefBranch;
// import frc.robot.subsystems.placerGrabber.PlacerGrabber;

// public class Autos extends RobotContainer{
    
//     private String typeOfIntake = "Ground";
    

//     private boolean readyToIntake = false;

//     private Command doesSeeCoralCommand() {
//         return new WaitUntilCommand(
//             () -> {return drivetrain.doesSeeCoral();})
//             .finallyDo(() -> {
//                 if (drivetrain.doesSeeCoral()) {
//                     typeOfIntake = "Ground";
//                 }
//                 else typeOfIntake = "Source";
//             });
//         }

//     private Command intake() {
//         return new ConditionalCommand(
//             intakeTowardsCoralInAuto(),
//             sourceIntake(),
//             () -> {return typeOfIntake == "Ground";});
//     }

//     private Command intakeIfDoesntHaveCoral() {
//         boolean doesHaveCoral = placerGrabber.doesHaveCoral();
//         if (doesHaveCoral) {
//             return null;
//         }
//         return sourceIntake();
//     }

//     public Command rightSideAuto() {

//         return new SequentialCommandGroup(
//             FlyingCircuitUtils.followPath("Right Start to Top Right"),
//             scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_F4),
//             FlyingCircuitUtils.followPath("Right Side to Right Source").withDeadline(doesSeeCoralCommand()), // will stop when either is completed?
//             intake(),
//             intakeIfDoesntHaveCoral(),
//             FlyingCircuitUtils.followPath("Right Source to Bottom Right"),
//             scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_C4),
//             FlyingCircuitUtils.followPath("Right Side to Right Source").withDeadline(doesSeeCoralCommand()),
//             intake(),
//             intakeIfDoesntHaveCoral(),
//             FlyingCircuitUtils.followPath("Right Source to Bottom Right"),
//             scoreOnReefCommand(charlie::getRequestedFieldOrientedVelocity, () -> ReefBranch.BRANCH_D4)
//         );

//     }
// }
