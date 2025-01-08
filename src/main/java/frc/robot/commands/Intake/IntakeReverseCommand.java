// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends Command {
  
  IntakeSubsystem _intakeSubsystem;
  boolean done = false; 

/** Creates a new Intake. */
  public IntakeReverseCommand(IntakeSubsystem m_intakeSubsystem) {
    _intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      _intakeSubsystem.setIntakeDesiredSpeed(-0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
