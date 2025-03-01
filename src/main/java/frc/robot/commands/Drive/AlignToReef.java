// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.lib.EReefAlignment;
import frc.robot.lib.ERobotMode;
import frc.robot.lib.GD;
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

  /**
   * ID == Index
   * Ex. for ID 1, get rotations[1]
   * Relative to facing red drivers == 0
   */

  // Array for Tag Rotations. Index 0 should never be used.
  private double[] rotations = {
      0, // Id should never be 0

      // Red Side

      -60, // 1
      60, // 2
      -90, // 3
      -180, // 4
      -180, // 5
      120, // 6
      0, // 7
      60, // 8
      120, // 9
      180, // 10
      60, // 11

      // Blue Side

      -120, // 12
      120, // 13
      0, // 14
      0, // 15
      90, // 16
      60, // 17
      0, // 18
      -60, // 19
      120, // 20
      180, // 21
      -120 // 22
  };

  // Convert rotation to the direction its supposed to be
  private double getRotation(int tagId) {
    if (tagId > 0) {
      double blueRotation = GD.G_Alliance == Alliance.Blue ? 180 : 0; // If you're on blue team, rotate angles by 180
                                                                      // because you start facing the other way
      double teleopRotation = GD.G_RobotMode == ERobotMode.TELEOP ? 180 : 0; // If you're in teleop, rotate by 180 to
                                                                             // make it easier for the drivers

      return rotations[tagId] + blueRotation + teleopRotation;
    } else {
      return 0;
    }
  }

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
    drivePID = new PIDController(0.4, 0, 0); // Initialize PID.

    robotAngle = getRotation((int) LimelightHelpers.getFiducialID("")); // Set the robot's angle based on the tag ID.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (reefAlignment == EReefAlignment.CENTER_REEF) { // If the requested position for the robot position is the center of the reef:
      driveAngle = -LimelightHelpers.getTX("") + driveSubsystem.m_robotDrive.getRobotYaw(); // Set the offset for driving.
    } else if (reefAlignment == EReefAlignment.LEFT_REEF) { // If the requested position for the robot position is the left of the reef:
      driveAngle = -LimelightHelpers.getTX("") + driveSubsystem.m_robotDrive.getRobotYaw() + 9; // Set the offset for driving.
    } else if (reefAlignment == EReefAlignment.RIGHT_REEF) { // If the requested position for the robot position is the right of the reef:
      driveAngle = -LimelightHelpers.getTX("") + driveSubsystem.m_robotDrive.getRobotYaw() - 9; // Set the offset for driving.
    }

    if (((LimelightHelpers.getTA("") > requestedTA - 0.1) && (LimelightHelpers.getTA("") < requestedTA + 0.1))
                                                                                                               // If the robot
                                                                                                               // is within the
                                                                                                               // requested TA
        && driveAngle != 0) { // And the drive angle is not 0:
      speed = drivePID.calculate(0, (driveAngle - driveSubsystem.m_robotDrive.getRobotYaw()) * 0.1); // Calculate the drive
                                                                                                     // speed based off of TX.
    } else if (LimelightHelpers.getTA("") != 0) { // Else: If TA is not 0:
      speed = drivePID.calculate(0, -LimelightHelpers.getTA("") + requestedTA); // Calculate the drive speed 
                                                                                // based off of TA.
    } else { // Else:
      speed = 0; // Stop the robot.
    }

    speed = MathUtil.clamp(speed, -2, 2); // Set the maximum speed to 2 and minimum speed to -2.

    if (((LimelightHelpers.getTA("") > requestedTA - 0.1) && (LimelightHelpers.getTA("") < requestedTA + 0.1)) // If the
                                                                                                               // robot is within the
                                                                                                               // requested TA
        && driveAngle != 0) { // And drive angle is not 0:
      driveSubsystem.drivePolarFieldCentric(90 + driveSubsystem.m_robotDrive.getRobotYaw(), robotAngle, speed, // Drive sideways to get
                                                                                                               // within the requested TX.
          true, true);
    } else { // Else:
      driveSubsystem.drivePolarFieldCentric(driveAngle, robotAngle, speed, true, true); // Drive normally.
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
