// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.EDriveMode;
import frc.robot.lib.Swerve.SwerveDrive;

public class DriveSubsystem extends SubsystemBase {
  public SwerveDrive m_robotDrive;
  public EDriveMode m_driveMode = EDriveMode.FIELD_CENTRIC;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_robotDrive = new SwerveDrive();
  
  }

  /* SET DRIVE MODE COMMANDS */
  public void setDriveMode_AngleFieldCentric() {
    m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
  }

  public void setDriveMode_FieldCentric() {
    m_driveMode = EDriveMode.FIELD_CENTRIC;
  }

  public void setDriveMode_PolarCentric() {
    m_driveMode = EDriveMode.POLAR_CENTRIC;
  }

  public void setDriveMode_RobotCentric() {
    m_driveMode = EDriveMode.ROBOT_CENTRIC;
  }

  public void setDriveMode_RotateFieldCentric() {
    m_driveMode = EDriveMode.ROTATE_FIELD_CENTRIC;
  }

  public void cycleDriveMode() {
    switch (m_driveMode) {
      case ANGLE_FIELD_CENTRIC:
        m_driveMode = EDriveMode.FIELD_CENTRIC;
        break;
      case FIELD_CENTRIC:
        m_driveMode = EDriveMode.ANGLE_FIELD_CENTRIC;
        break;
      default:
        /* nothing lol */
        break;
    }
  }

  public EDriveMode getDriveMode() {
    return m_driveMode;
  }

  public void driveStopMotion(){
    m_robotDrive.driveStopMotion();
  }
  public void driveTotalStop() {
    m_robotDrive.driveTotalStop();
  }
  public void driveRobotCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveRobotCentric(_speeds);
  }

  public void driveFieldCentric(ChassisSpeeds _speeds){
    m_robotDrive.driveFieldCentric(_speeds);
  }

  public void driveAngleFieldCentric(double _x, double _y, Rotation2d _targetAngle, boolean _enableSteer, boolean _enableDrive){
    m_robotDrive.driveAngleFieldCentric(_x, _y, _targetAngle, _enableSteer, _enableDrive);
  }

  public void drivePolarFieldCentric(double _driveAngle_deg, double _robotAngle_deg, double _speed, boolean _enableSteer, boolean _enableDrive){
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    driveAngleFieldCentric(x, y, new Rotation2d(Math.toRadians(_robotAngle_deg)), _enableSteer, _enableDrive);
  }

  public void resetOdometry() {
    m_robotDrive.resetPose();
  }

  public void resetGyro() {
    m_robotDrive.resetYaw();
  }

  public void setGyro(double degrees) {
    m_robotDrive.setYaw(degrees);
  }

  /** Sets the new position in inches */
  public void setPos(Pose2d newPos) {
      m_robotDrive.setPos(newPos);
  }

  /**
   * Sets the new position in inches
   * 
   * @param newX The new X position in inches
   * @param newY The new Y position in inches
   */
  public void setPos(double newX, double newY) {
      m_robotDrive.setPos(newX, newY);
  }

  public void stopMotors() {
    m_robotDrive.driveStopMotion();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCancoderPos(int i) {
    return m_robotDrive.getModules()[i].getCancoderPos();
  }

  public void updateDashboard() {
    
  }
}
