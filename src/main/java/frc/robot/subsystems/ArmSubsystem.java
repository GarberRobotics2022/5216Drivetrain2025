// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.EArmPos;

public class ArmSubsystem extends SubsystemBase {

  // PID things
  SlewRateLimiter limiter = new SlewRateLimiter(0.05);

  // Tune values
  PIDController pid = new PIDController(0.05, 0, 0);
  double d = 0;
  int i = 0;
  boolean hingeCalibrated = false;
  boolean timerStarted = false;

  // Motors \\
  public TalonFX leftArmMotor = new TalonFX(41);
  public TalonFX rightArmMotor = new TalonFX(42);

  // Limit Switches \\
  public DigitalInput rightLimitSwitch = new DigitalInput(2);
  public DigitalInput leftLimitSwitch = new DigitalInput(3);

  // Arm Positions \\
  public final double ampPosition = 35.0;
  public final double higherPosition = 20.0;
  public final double halfwayPosition = 16.0;
  public final double autonPosition = 7.5;
  public final double speakerPosition = 6.5;
  public final double floorPosition = 0.0;
  public final double farSpeakerPosition = 13.0;

  // Misc \\
  private VoltageOut voltageOut;
  private double speed = 0.0;
  public boolean moveToPosDone = false;

  /** Creates a new HingeSubsystem. */
  public ArmSubsystem() {
    hingeCalibrated = false;

    voltageOut = new VoltageOut(0);

    leftArmMotor.setPosition(0);
    rightArmMotor.setPosition(0);

    TalonFXConfiguration leftConfigs = new TalonFXConfiguration();
    leftConfigs.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

    TalonFXConfiguration rightConfigs = new TalonFXConfiguration();
    rightConfigs.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);

    // leftHingeMotor.setNeutralMode(NeutralModeValue.Coast);
    // rightHingeMotor.setNeutralMode(NeutralModeValue.Coast);
    leftArmMotor.setNeutralMode(NeutralModeValue.Brake);
    rightArmMotor.setNeutralMode(NeutralModeValue.Brake);

    rightArmMotor.setControl(new Follower(leftArmMotor.getDeviceID(), true));

  }

  // True if either limit switch is down
  public boolean limitSwitchGet() {
    return rightLimitSwitch.get() || leftLimitSwitch.get();
  }

  public void move(double _speed) {
    speed = _speed;
    if (speed < 0 && limitSwitchGet()) {
      speed = 0;
    }
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public void stop() {
    move(0);
  }

  public void calibrateArmPos() {
    leftArmMotor.setPosition(0);
    rightArmMotor.setPosition(0);
  }

  public void lowerHinge() {
    if (!leftLimitSwitch.get() && !rightLimitSwitch.get()) {
      if (leftArmMotor.getPosition().getValueAsDouble() > -28.5) { // -29
        leftArmMotor.setControl(voltageOut.withOutput(-0.3 * 12));
      } else {
        leftArmMotor.setControl(voltageOut.withOutput(-0.075 * 12)); // -0.1
      }
    } else {
      leftArmMotor.setControl(voltageOut.withOutput(0));
    }
  }

  public boolean moveArmPID(EArmPos armPos) {
    if (!hingeCalibrated) {
      if (limitSwitchGet()) {
        hingeCalibrated = true;
        calibrateArmPos();
      }
      lowerHinge();
    } else {
      if (armPos == EArmPos.FLOOR_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            floorPosition);
      } else if (armPos == EArmPos.SPEAKER_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            speakerPosition);
      } else if (armPos == EArmPos.AUTON_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            autonPosition);
      } else if (armPos == EArmPos.HALFWAY_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            halfwayPosition);
      } else if (armPos == EArmPos.HIGH_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            higherPosition);
      } else if (armPos == EArmPos.AMP_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            ampPosition);
      } else if (armPos == EArmPos.FAR_SPEAKER_POS) {
        d = pid.calculate(leftArmMotor.getPosition().getValueAsDouble(),
            farSpeakerPosition);
      }

      // if (RobotContainer.buttonPressed) {
      // timer.restart();
      // } else {
      // RobotContainer.buttonPressed = false;
      // }

      if (d < 0) {
        if (leftArmMotor.getPosition().getValueAsDouble() < 5) {
          move(-0.05); // -0.075
        } else {
          d = limiter.calculate(d);
          move(d / 3.9);
        }
      } else {
        move(d / 1.25);
      }

      // if (timer.hasElapsed(10)) {
      // timer.reset();
      // }

      SmartDashboard.putNumber("Hinge Running", i++);
      SmartDashboard.putBoolean("Hinge Calibrated", hingeCalibrated);
      SmartDashboard.putNumber("d", d);
      // SmartDashboard.putNumber("Big Chomp Time", timer.get());
    }

    // Return true when arm at position
    if (Math.abs(d) < 0.1) { // Use a small threshold for precision
      return true;
    }
    return false;
  }

  /*
   * Big comment for shortening
   * // why
   * // public void setArmPos(EArmPos pos) {
   * // boolean TRUE = false; // HEHEHEHEHE
   * // boolean FALSE = true; // HEHEHEHEHE
   * // }
   * 
   * ///**
   * //* Beeg Method
   * //*
   * //* @param _armPos Position of the arm wanted.
   * //* @param _speed Speed of arm. (POSITIVE NUMBERS ONLY!!!)
   * //
   * // public void moveToPos(EArmPos _armPos, double _speed) {
   * // switch (_armPos) {
   * // case AMP_POS:
   * // if (leftArmMotor.get() < ampPosition + 1 && rightArmMotor.get() <
   * ampPosition
   * // + 1
   * // && leftArmMotor.get() > ampPosition - 1 && rightArmMotor.get() >
   * ampPosition
   * // - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > ampPosition && rightArmMotor.get() >
   * // ampPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < ampPosition && rightArmMotor.get() <
   * // ampPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // case AUTON_POS:
   * // if (leftArmMotor.get() < autonPosition + 1 && rightArmMotor.get() <
   * // autonPosition + 1
   * // && leftArmMotor.get() > autonPosition - 1 && rightArmMotor.get() >
   * // autonPosition - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > autonPosition && rightArmMotor.get() >
   * // autonPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < autonPosition && rightArmMotor.get() <
   * // autonPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // case FAR_SPEAKER_POS:
   * // if (leftArmMotor.get() < farSpeakerPosition + 1 && rightArmMotor.get() <
   * // farSpeakerPosition + 1
   * // && leftArmMotor.get() > farSpeakerPosition - 1 && rightArmMotor.get() >
   * // farSpeakerPosition - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > farSpeakerPosition && rightArmMotor.get()
   * >
   * // farSpeakerPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < farSpeakerPosition && rightArmMotor.get()
   * <
   * // farSpeakerPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // case FLOOR_POS:
   * // if (leftArmMotor.get() < floorPosition + 1 && rightArmMotor.get() <
   * // floorPosition + 1
   * // && leftArmMotor.get() > floorPosition - 1 && rightArmMotor.get() >
   * // floorPosition - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > floorPosition && rightArmMotor.get() >
   * // floorPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < floorPosition && rightArmMotor.get() <
   * // floorPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // case HALFWAY_POS:
   * // if (leftArmMotor.get() < halfwayPosition + 1 && rightArmMotor.get() <
   * // halfwayPosition + 1
   * // && leftArmMotor.get() > halfwayPosition - 1 && rightArmMotor.get() >
   * // halfwayPosition - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > halfwayPosition && rightArmMotor.get() >
   * // halfwayPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < halfwayPosition && rightArmMotor.get() <
   * // halfwayPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // case HIGH_POS:
   * // if (leftArmMotor.get() < higherPosition + 1 && rightArmMotor.get() <
   * // higherPosition + 1
   * // && leftArmMotor.get() > higherPosition - 1 && rightArmMotor.get() >
   * // higherPosition - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > higherPosition && rightArmMotor.get() >
   * // higherPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < higherPosition && rightArmMotor.get() <
   * // higherPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // case SPEAKER_POS:
   * // if (leftArmMotor.get() < speakerPosition + 1 && rightArmMotor.get() <
   * // speakerPosition + 1
   * // && leftArmMotor.get() > speakerPosition - 1 && rightArmMotor.get() >
   * // speakerPosition - 1) {
   * // moveToPosDone = true;
   * // stop();
   * // } else if (leftArmMotor.get() > speakerPosition && rightArmMotor.get() >
   * // speakerPosition) {
   * // move(-_speed);
   * // } else if (leftArmMotor.get() < speakerPosition && rightArmMotor.get() <
   * // speakerPosition) {
   * // move(_speed);
   * // } else {
   * // stop();
   * // }
   * // break;
   * // default:
   * // break;
   * // }
   * // }
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
