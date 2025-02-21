// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefFaceLED extends Command {

  Leds leds;
  Drivetrain drivetrain;
  
  public ReefFaceLED(Leds leds, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    this.drivetrain = drivetrain;
    addRequirements(leds);
  }

  
  @Override
  public void execute() {
    String closestStalkName = drivetrain.getClosestReefFace().getName();
    if (closestStalkName.toLowerCase() == "a" || closestStalkName.toLowerCase() == "b"  || closestStalkName.toLowerCase() == "i" || closestStalkName.toLowerCase() == "j"|| closestStalkName.toLowerCase() == "f" || closestStalkName.toLowerCase() == "e") {
      leds.red();
    }
    else {
      leds.blue();
    }
  }
}
