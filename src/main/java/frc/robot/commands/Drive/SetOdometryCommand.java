// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetOdometryCommand extends Command {
  DriveSubsystem m_DriveSubsystem;

  double x;
  double y;

  /** Creates a new SetGyroCommand. 
   * 
   * @param x New x position in inches
   * @param y New y position in inches
  */
  public SetOdometryCommand(DriveSubsystem _DriveSubsystem, double x, double y) {
    this.x = x;
    this.y = y;

    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = _DriveSubsystem;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.setPos(x, y);
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
