// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRotateCommand extends Command {
DriveSubsystem driveSubsystem;
double robotAngle;
double delay;
Timer timer = new Timer();

  /** Creates a new AutoRotateCommand. */
  public AutoRotateCommand(DriveSubsystem _driveSubsystem, double _robotAngle, double _delay) {
    driveSubsystem = _driveSubsystem;
    robotAngle = _robotAngle;
    delay = _delay;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.driveAngleFieldCentric(0, 0, new Rotation2d(Math.toRadians(robotAngle)),true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(delay)) {
      return true;
    } else {
      return false;
    }
  }
}
