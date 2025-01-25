// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoReset;
import frc.robot.commands.Drive.AutoRotateCommand;
import frc.robot.commands.Drive.DriveToApriltag;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedCenterProcessorAuton extends SequentialCommandGroup {
  /** Creates a new CenterSimpleAuton. */
  public RedCenterProcessorAuton(DriveSubsystem _DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoReset(_DriveSubsystem), // Reset position and rotation
      
      new DriveToApriltag(_DriveSubsystem, 10, 4, 0.35, 0), // Go to reef middle
      new AutoRotateCommand(_DriveSubsystem, 90,1), // Rotate to face processor
      new DriveToApriltag(_DriveSubsystem, 3, 0.7, 0.35, 90), // Go to processor
      new AutoRotateCommand(_DriveSubsystem, -45, 2), // Face reef
      new DriveToApriltag(_DriveSubsystem, 9, 4, 0.35, -45), // Go to reef
      new AutoRotateCommand(_DriveSubsystem, 90, 2), // Face processor
      new DriveToApriltag(_DriveSubsystem, 3, 0.7, 0.35, 90) // Go to processor
    );
  }
}
