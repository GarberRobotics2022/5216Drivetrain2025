// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShootCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem;

  /** Speed the shooter has to get to to shoot */
  double shooterShootSpeed = 0.75;
  boolean done;

  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(IntakeSubsystem _IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = _IntakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    m_IntakeSubsystem.setShooterSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_IntakeSubsystem.shooterTopMotor.getVelocity().getValueAsDouble() + m_IntakeSubsystem.shooterBottomMotor.getVelocity().getValueAsDouble() / 2 >= 55) {
      m_IntakeSubsystem.setIntakeDesiredSpeed(0.85);
      if (!m_IntakeSubsystem.beamSensorTriggered()) {
        done = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopIntake();
    m_IntakeSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
