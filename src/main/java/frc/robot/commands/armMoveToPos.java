// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Imports */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.EArmPos;
import frc.robot.subsystems.ArmSubsystem;

/* armMoveToPos Creation */

public class armMoveToPos extends Command {

  ArmSubsystem _armSubsystem; // The arm subsystem.
  EArmPos _armPos; // The requested arm position to be set.
  double _speed = 0.5; // Adjust this for the speed of the arm moving using this command.

  double armTimeout = 0.25;

  Timer time = new Timer();

  /* Creates a new armMoveToPos. */
  /**
   * 
   * @param m_armSubsystem The arm subsystem. (Requirement)
   * @param m_armPos The arm position the arm will be set to.
   */
  public armMoveToPos(ArmSubsystem m_armSubsystem, EArmPos m_armPos) {
    _armSubsystem = m_armSubsystem;
    _armPos = m_armPos;
    // addRequirements(_armSubsystem);
  }

  /* Creates a new armMoveToPos. */
  /**
   * 
   * @param m_armSubsystem The arm subsystem. (Requirement)
   * @param m_armPos The arm position the arm will be set to.
   * @param _armTimeout The amount of seconds to wait before moving on
   */
  public armMoveToPos(ArmSubsystem m_armSubsystem, EArmPos m_armPos, double _armTimout) {
    _armSubsystem = m_armSubsystem;
    _armPos = m_armPos;
    armTimeout = _armTimout;
    // addRequirements(_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.moveArmPID(_armPos);
    time.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.armPos = _armPos;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.hasElapsed(armTimeout); //_armSubsystem.moveArmPID(_armPos); // Is done when the arm subsystem is done moving the arm.
  }
}
