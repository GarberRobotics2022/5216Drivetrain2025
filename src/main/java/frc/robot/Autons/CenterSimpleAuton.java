// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.DriveToApriltag;
import frc.robot.commands.Drive.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterSimpleAuton extends SequentialCommandGroup {
  DriveSubsystem m_DriveSubsystem; 
  /** Creates a new CenterSimpleAuton. */
  public CenterSimpleAuton(DriveSubsystem _DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_DriveSubsystem = _DriveSubsystem;
    addCommands(
      new ResetGyroCommand(m_DriveSubsystem),
      // new AutoDriveOdometry(_DriveSubsystem, 69, 0, 0, 12)
      new DriveToApriltag(m_DriveSubsystem, 1, 4, 0.25, 0)
    );
  }
}
