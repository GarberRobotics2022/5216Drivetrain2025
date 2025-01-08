// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  // Motor Creation and Identification \\
  CANSparkMax leftClimber = new CANSparkMax(51, MotorType.kBrushless);
  CANSparkMax rightClimber = new CANSparkMax(52, MotorType.kBrushless);

  // Climber Limits \\
  private final double lowerLimit = 0.0;
  private final double upperLimit = 250.0;

  // Speeds \\
  double leftSpeed = 0;
  double rightSpeed = 0;
  
  // Creates new Climber Subsystem \\
  public ClimberSubsystem() {

    // Set Position to 0 \\
    leftClimber.getEncoder().setPosition(0);
    rightClimber.getEncoder().setPosition(0);

    // Climber Inversion \\
    leftClimber.setInverted(true);
    rightClimber.setInverted(false);

    // Climber Mode \\
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
  }

  // Climber Control
  public void controlLeftClimber(double speed) {
    leftSpeed = speed;
    if((leftSpeed >= 0 && leftClimber.getEncoder().getPosition() < upperLimit) || (leftSpeed <= 0 && leftClimber.getEncoder().getPosition() > lowerLimit)) {
      leftClimber.set(leftSpeed);
    } else {
      leftClimber.set(0);
    }
  }
  public void controlRightClimber(double speed) {
    rightSpeed = speed;
    if((rightSpeed >= 0 && rightClimber.getEncoder().getPosition() < upperLimit) || (rightSpeed <= 0 && rightClimber.getEncoder().getPosition() > lowerLimit)) {
      rightClimber.set(rightSpeed);
    } else {
      rightClimber.set(0);
    }
  }

  public double getLeftPos() {
    return leftClimber.getEncoder().getPosition();
  }

  public double getRightPos() {
    return rightClimber.getEncoder().getPosition();
  }

  public double getLeftSpeed() {
    return leftSpeed;
  }

  public double getRightSpeed() {
    return rightSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
