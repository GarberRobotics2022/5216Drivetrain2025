// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveOdometry;
import frc.robot.commands.Drive.AutoRotateCommand;
import frc.robot.commands.Drive.DriveToApriltag;
import frc.robot.commands.Drive.ResetGyroCommand;
import frc.robot.commands.Drive.ResetOdometry;
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
      new ResetOdometry(_DriveSubsystem), // Reset position
      new ResetGyroCommand(m_DriveSubsystem), // Reset angle
      
      new DriveToApriltag(_DriveSubsystem, 10, 4, 0.35, 0),
      // new AutoDriveOdometry(_DriveSubsystem, 30, 0, 0, 0.01), // Go to origin
      new AutoRotateCommand(_DriveSubsystem, 90,1), // Rotate to face processor
      // new AutoDriveOdometry(_DriveSubsystem, 30, 90, 90, 0.01), // Go to processor
      // new AutoRotateCommand(_DriveSubsystem, 90, 1)
      new DriveToApriltag(_DriveSubsystem, 3, 0.7, 0.35, 90),
      // new AutoDriveOdometry(_DriveSubsystem, 30, 90, 90, 0.01)
      new AutoRotateCommand(_DriveSubsystem, -45, 2),
      new DriveToApriltag(_DriveSubsystem, 9, 4, 0.35, -45),
      new AutoRotateCommand(_DriveSubsystem, 90, 2),
      new DriveToApriltag(_DriveSubsystem, 3, 0.7, 0.35, 90)
    );
  }
}
