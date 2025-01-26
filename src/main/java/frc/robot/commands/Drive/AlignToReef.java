// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.lib.EReefAlignment;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  DriveSubsystem driveSubsystem;
  EReefAlignment reefAlignment;
  PIDController drivePID;
  double robotAngle;
  double speed;
  double driveAngle;
  double requestedTA = 3;

  /** Creates a new AlignToReef. */
  public AlignToReef(DriveSubsystem _driveSubsystem, EReefAlignment _reefAlignment) {
    driveSubsystem = _driveSubsystem;
    reefAlignment = _reefAlignment;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID = new PIDController(0.4, 0, 0);
    switch ((int) LimelightHelpers.getFiducialID("")) {
      case 1:
        robotAngle = 0;
        break;
      case 2:
        robotAngle = 0;
        break;
      case 3:
        robotAngle = 0;
        break;
      case 4:
        robotAngle = 0;
        break;
      case 5:
        robotAngle = 0;
        break;
      case 6:
        robotAngle = 0;
        break;
      case 7:
        robotAngle = 0;
        break;
      case 8:
        robotAngle = 0;
        break;
      case 9:
        robotAngle = 0;
        break;
      case 10:
        robotAngle = 0;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (reefAlignment == EReefAlignment.CENTER_REEF) {
      driveAngle = -LimelightHelpers.getTX("") + driveSubsystem.m_robotDrive.getRobotYaw();
    } else if (reefAlignment == EReefAlignment.LEFT_REEF) {
      driveAngle = -LimelightHelpers.getTX("") + driveSubsystem.m_robotDrive.getRobotYaw() + 9;
    } else if (reefAlignment == EReefAlignment.RIGHT_REEF) {
      driveAngle = -LimelightHelpers.getTX("") + driveSubsystem.m_robotDrive.getRobotYaw() - 9;
    }

    if (((LimelightHelpers.getTA("") > requestedTA - 0.1) && (LimelightHelpers.getTA("") < requestedTA + 0.1))
        && driveAngle != 0) {
      speed = drivePID.calculate(0, driveAngle * 0.1);
    } else if (LimelightHelpers.getTA("") != 0) {
      speed = drivePID.calculate(0, -LimelightHelpers.getTA("") + requestedTA);
    } else {
      speed = 0;
    }

    speed = MathUtil.clamp(speed, -2, 2);

    if (((LimelightHelpers.getTA("") > requestedTA - 0.1) && (LimelightHelpers.getTA("") < requestedTA + 0.1))
        && driveAngle != 0) {
      driveSubsystem.drivePolarFieldCentric(90 + driveSubsystem.m_robotDrive.getRobotYaw(), robotAngle, speed,
          true, true);
    } else {
      driveSubsystem.drivePolarFieldCentric(driveAngle, robotAngle, speed, true, true);
    }

    SmartDashboard.putNumber("driveAngle", driveAngle);
    SmartDashboard.putBoolean("atSetpoint",
        ((LimelightHelpers.getTA("") > requestedTA - 0.1) && (LimelightHelpers.getTA("") < requestedTA + 0.1)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
