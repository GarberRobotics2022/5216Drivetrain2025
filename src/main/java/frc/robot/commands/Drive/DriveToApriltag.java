// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToApriltag extends Command {
  int targetTag;
  double targetArea;
  double speed;
  double robotAngle;
  DriveSubsystem driveSubsystem;
  PIDController drivePID;
  boolean isFinished;
  double i;
  /** Creates a new DriveToApriltag. */
  public DriveToApriltag(DriveSubsystem _driveSubsystem, int _targetTag, double _targetArea, double _speed, double _robotAngle) {
    targetTag = _targetTag;
    targetArea = _targetArea;
    speed = _speed;
    robotAngle = _robotAngle;
    driveSubsystem = _driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivePID = new PIDController(1,0,0);
    SmartDashboard.putNumber("tag stuff", 0);
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int t = (int)LimelightHelpers.getFiducialID("FrontLimelight");
    SmartDashboard.putNumber("tag stuff", t);
    if (t == targetTag) {
      SmartDashboard.putNumber("tag stuff", t+1);
      if ((LimelightHelpers.getTA("FrontLimelight") > targetArea + 0.1) || (LimelightHelpers.getTA("FrontLimelight") < targetArea - 0.1)) {
        drivePID.calculate(targetArea * speed);
      } else {
        speed = 0;
        isFinished = false;
      }
      
      driveSubsystem.drivePolarFieldCentric(SmartDashboard.getNumber("limelight/tx", 0), robotAngle, speed, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
