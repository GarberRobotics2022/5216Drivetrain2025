// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GotoAprilTag extends Command {
  DriveSubsystem m_DriveSubsystem;

  int apriltag;
  double distance;

  /** Creates a new GotoAprilTag. */
  public GotoAprilTag(DriveSubsystem _DriveSubsystem, int inputAprilTag, double _distance) {
    m_DriveSubsystem = _DriveSubsystem;
    addRequirements(m_DriveSubsystem);

    distance = _distance;
    apriltag = inputAprilTag;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var vision = LimelightHelpers.getRawDetections("FrontLimelight");
    for (int i = 0; i < vision.length; i++) {
      if (vision[i].classId == apriltag) {
        // TODO: do something
        if (distance < LimelightHelpers.getTA("FrontLimelight")) {
          // move back
        } else if (distance > LimelightHelpers.getTA("FrontLimelight")) {
          // more forward
        } else {
          // stop
        }
      }
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
