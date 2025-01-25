// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.AutoRotateCommand;
import frc.robot.commands.Drive.ResetGyroCommand;
import frc.robot.commands.Drive.ResetOdometry;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestingOdometry extends SequentialCommandGroup {
  /** Creates a new TestingOdometry. */
  public TestingOdometry(DriveSubsystem m_DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(m_DriveSubsystem),
      new ResetGyroCommand(m_DriveSubsystem),

      // new AutoRotateCommand(m_DriveSubsystem, 90, 1),
      new AutoRotateCommand(m_DriveSubsystem, 90, 1),
      new AutoDriveOdometry(m_DriveSubsystem, 10, 0, 90, 0.01),
      new AutoRotateCommand(m_DriveSubsystem, 0, 1),
      new AutoDriveOdometry(m_DriveSubsystem, 10, 10, 0, 0.01),
      new AutoRotateCommand(m_DriveSubsystem, -90, 1),
      new AutoDriveOdometry(m_DriveSubsystem, 0, 10, -90, 0.01),
      new AutoRotateCommand(m_DriveSubsystem, 180, 1),
      new AutoDriveOdometry(m_DriveSubsystem, 0, 0, 180, 0.01)
    );
  }
}
