// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  // Motor Creation and Identification \\
  SparkMax leftClimber = new SparkMax(51, MotorType.kBrushless);
  SparkMax rightClimber = new SparkMax(52, MotorType.kBrushless);

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

    // Climber config \\
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);

    leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightClimber.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
