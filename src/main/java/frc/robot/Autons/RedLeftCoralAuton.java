// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoAlignToReef;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.AutoReset;
import frc.robot.commands.Drive.AutoRotateCommand;
import frc.robot.lib.EReefAlignment;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeftCoralAuton extends SequentialCommandGroup {
  /** Creates a new RedRightCoralAuton. */
  public RedLeftCoralAuton(DriveSubsystem _DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoReset(_DriveSubsystem), // Reset pose and rotation

      new AutoDriveOdometry(_DriveSubsystem, 180, 0, -240, 0.01),
      new AutoRotateCommand(_DriveSubsystem, 120, 1),
      new AutoAlignToReef(_DriveSubsystem, EReefAlignment.CENTER_REEF, 3, "front", 2, false),
      new AutoAlignToReef(_DriveSubsystem, EReefAlignment.CENTER_REEF, 0.8, "back", 2, true),
      new AutoAlignToReef(_DriveSubsystem, EReefAlignment.CENTER_REEF, 3, "front", 2, false)
      // new AutoRotateCommand(_DriveSubsystem, 60, 1),
      // new DriveToApriltag(_DriveSubsystem, 2z, 4, 0.35, 60),
      // new AutoRotateCommand(_DriveSubsystem, -120, 1)
    );
  }
}
