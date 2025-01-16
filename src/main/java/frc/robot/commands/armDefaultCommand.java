// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.EArmPos;
import frc.robot.subsystems.ArmSubsystem;

public class armDefaultCommand extends Command {

  ArmSubsystem m_armSubsystem;
  // Timer timer = new Timer();
  SlewRateLimiter limiter = new SlewRateLimiter(0.05);

  // Tune values
  PIDController pid = new PIDController(0.05, 0, 0);
  double d = 0;
  int i = 0;
  boolean hingeCalibrated = false;
  boolean timerStarted = false;

  // double previousEArmPos = m_armSubsystem.floorPosition;
  // double currentSetpoint = m_armSubsystem.floorPosition;

  /** Creates a new ExtenderDefaultCommand. */
  public armDefaultCommand(ArmSubsystem _armSubsystem) {
    m_armSubsystem = _armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hingeCalibrated = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hingeCalibrated) {
      if (m_armSubsystem.limitSwitchGet()) {
        hingeCalibrated = true;
        m_armSubsystem.calibrateArmPos();
      }
      m_armSubsystem.lowerHinge();
    } else {
      if (RobotContainer.armPos == EArmPos.FLOOR_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.floorPosition);

      } else if (RobotContainer.armPos == EArmPos.SPEAKER_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.speakerPosition);
      } else if (RobotContainer.armPos == EArmPos.AUTON_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.autonPosition);
      } else if (RobotContainer.armPos == EArmPos.HALFWAY_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.halfwayPosition);
      } else if (RobotContainer.armPos == EArmPos.HIGH_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.higherPosition);
      } else if (RobotContainer.armPos == EArmPos.AMP_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.ampPosition);
      } else if (RobotContainer.armPos == EArmPos.FAR_SPEAKER_POS) {
        d = pid.calculate(m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble(),
            m_armSubsystem.farSpeakerPosition);
      }

      // if (RobotContainer.buttonPressed) {
      // timer.restart();
      // } else {
      // RobotContainer.buttonPressed = false;
      // }

      if (d < 0) {
        if (m_armSubsystem.leftArmMotor.getPosition().getValueAsDouble() < 5) {
          m_armSubsystem.move(-0.05); // -0.075
        } else {
          d = limiter.calculate(d);
          m_armSubsystem.move(d / 3.9);
        }
      } else if (d > 0) {
        m_armSubsystem.move(d / 1.25);
      }

      // if (timer.hasElapsed(10)) {
      // timer.reset();
      // }

      SmartDashboard.putNumber("Hinge Running", i++);
      SmartDashboard.putBoolean("Hinge Calibrated", hingeCalibrated);
      SmartDashboard.putNumber("d", d);
      // SmartDashboard.putNumber("Big Chomp Time", timer.get());
    }
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

// There are 10 types of people in this world: those who understand binary and
// those who don't.
// "Decimal bastard." - Carter to Grant, 02/17