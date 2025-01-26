// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AlignToReef;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.AutoReset;
import frc.robot.commands.Drive.AutoRotateCommand;
import frc.robot.commands.Drive.DriveToApriltag;
import frc.robot.lib.EReefAlignment;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedRightCoralAuton extends SequentialCommandGroup {
  /** Creates a new RedRightCoralAuton. */
  public RedRightCoralAuton(DriveSubsystem _DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoReset(_DriveSubsystem), // Reset pose and rotation

      new AutoDriveOdometry(_DriveSubsystem, 180 / 1.32, 0, 0, 0.01)//,
      // new AutoRotateCommand(_DriveSubsystem, -120, 1),
      // new DriveToApriltag(_DriveSubsystem, 8, 4, 0.35, -120),
      // new AutoRotateCommand(_DriveSubsystem, 60, 1),
      // new DriveToApriltag(_DriveSubsystem, 2, 4, 0.35, 60),
      // new AutoRotateCommand(_DriveSubsystem, -120, 1)
    );
  }
}
