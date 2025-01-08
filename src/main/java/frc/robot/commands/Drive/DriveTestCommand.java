// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTestCommand extends Command {
  DriveSubsystem m_driveSubsystem;
  double speed;
  double driveAngle;
  double robotAngle;
  Timer time = new Timer();
  double timeout;

  /** Set to true when command should end */
  boolean isFinished = false;

  /** Creates a new DriveTestCommand. */
  public DriveTestCommand(DriveSubsystem _driveSubsystem, double _timeout, double _speed, double _driveAngle, double _robotAngle) {
    m_driveSubsystem = _driveSubsystem;
    timeout = _timeout;
    speed = _speed;
    // ySpeed = _ySpeed;
    driveAngle = _driveAngle;
    robotAngle = _robotAngle;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drivePolarFieldCentric(driveAngle, robotAngle, speed, true, true);

    // Rotation2d rotation = new Rotation2d(robotAngle);
    // m_driveSubsystem.driveAngleFieldCentric(xSpeed, ySpeed, rotation, true, true);

    RobotContainer.timerTime = time.get();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.hasElapsed(timeout)) {
      return true;
    } else {
      return false;
    }
  }
}
