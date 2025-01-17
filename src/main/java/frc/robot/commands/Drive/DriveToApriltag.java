// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToApriltag extends Command {
  int targetTag;
  double targetArea;
  double speed;
  double driveAngle;
  DriveSubsystem driveSubsystem;
  PIDController drivePID;
  /** Creates a new DriveToApriltag. */
  public DriveToApriltag(DriveSubsystem _driveSubsystem, int _targetTag, double _targetArea, double _speed, double _driveAngle) {
    targetTag = _targetTag;
    targetArea = _targetArea;
    speed = _speed;
    driveAngle = _driveAngle;
    driveSubsystem = _driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID = new PIDController(1,0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTagID("5216Limelight001") == targetTag) {
      if ((LimelightHelpers.getTA("5216Limelight001") > targetArea++) || (LimelightHelpers.getTA("5216Limelight001") < targetArea--)) {
        drivePID.calculate(targetArea * speed);
      } else {
        speed = 0;
      }
      
      driveSubsystem.drivePolarFieldCentric(LimelightHelpers.getTX("5216Limelight001"), driveAngle, speed, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
