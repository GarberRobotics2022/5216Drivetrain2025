// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeIntakeCommand extends Command {
  
  IntakeSubsystem _IntakeSubsystem;
  boolean done = false; 
  double artificialInit = 0;
  boolean noteHasBeenSeen = false;
  double frameInt;
  Timer timer = new Timer();

/** Creates a new Intake. */
  public IntakeIntakeCommand(IntakeSubsystem m_IntakeSubsystem) {
    addRequirements(m_IntakeSubsystem);
    _IntakeSubsystem = m_IntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    noteHasBeenSeen = false;
    frameInt = 0;
    timer.reset();
    artificialInit = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (artificialInit < 3) { // 1
      noteHasBeenSeen = false;
    } else {
      if (!_IntakeSubsystem.beamSensorTriggered() && !noteHasBeenSeen) {
        _IntakeSubsystem.setIntakeDesiredSpeed(0.85); // 0.5
        // When obstructed, set noteHasBeenSeen to true
      } else if (_IntakeSubsystem.beamSensorTriggered() && !noteHasBeenSeen) {
        noteHasBeenSeen = true;
        timer.restart();
        // When obstruction occurs, rewind note at 15% power
      } else if (noteHasBeenSeen && frameInt < 4) { // !m_intakeSubsystem.getBeamBreakSensor() &&
        _IntakeSubsystem.setIntakeDesiredSpeed(-0.15);
        frameInt++;
      } else if (_IntakeSubsystem.beamSensorTriggered() && noteHasBeenSeen && frameInt >= 4) {
        done = true;
        noteHasBeenSeen = false;
      }
    }
    artificialInit++;
    SmartDashboard.putNumber("frameInt", frameInt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _IntakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
