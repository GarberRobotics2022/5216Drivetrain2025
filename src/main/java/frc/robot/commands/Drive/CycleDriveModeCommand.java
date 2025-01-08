// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CycleDriveModeCommand extends Command {
  /** Creates a new CycleDriveModeCommand. */
  DriveSubsystem _DriveSubsystem;
  public CycleDriveModeCommand(DriveSubsystem m_driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    _DriveSubsystem = m_driveSubsystem;
    addRequirements(_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _DriveSubsystem.cycleDriveMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
