// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RightClimberCommand extends Command {
  ClimberSubsystem _climberSubsystem;
  double _speed; 
  // Creates new RightClimberCommand \\
  /**
   * 
   * @param m_climberSubsystem The climber subsystem.
   * @param m_speed Negative number for down, positive for up.
   */
  public RightClimberCommand(ClimberSubsystem m_climberSubsystem, double m_speed) {
    _climberSubsystem = m_climberSubsystem;
    _speed = m_speed;
    // addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _climberSubsystem.controlRightClimber(_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _climberSubsystem.controlRightClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
