// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class TargetAngle {
    // BEGIN Class Data
    private Rotation2d m_targetAngle;
    private double m_actualAngle;
    private double m_hyp;
    
    // END  Class Data

    // BEGIN Class Constructors 
    public TargetAngle(){
      m_targetAngle = new Rotation2d(0);
    }

    // END Class Constructors

    // BEGIN Class Methods
    /** Set the target angle by giving a x and y value and creating a Rotation 2D object.
     * A Rotation2d object is just a fancy class for holding an angle.
     * 
     * @param _x The x value that usually comes from the joystick
     * @param _y The y value that usually comes from the joystick
     */
    public Rotation2d setTargetAngle(double _x, double _y){
       
        m_hyp = Math.hypot(_x, _y);
        if(Math.abs(m_hyp) > k.DRIVE.TARGET_ANGLE_DEADBAND){
          m_actualAngle = Math.toDegrees(Math.atan2(_x,_y));
          //SmartDashboard.putNumber("m_actualAngle", m_actualAngle);
          if(m_actualAngle >= -22.5 && m_actualAngle <= 22.5){  // North
            m_targetAngle = Rotation2d.fromDegrees(0.0);
          }else if(m_actualAngle >= -67.5 && m_actualAngle < -22.5){  // North East
            m_targetAngle =  GD.G_Alliance == Alliance.Blue ? Rotation2d.fromDegrees(-60.0) : Rotation2d.fromDegrees(-60);

            // TODO: THIS IS SO MUCH BETTER, USE THIS *PLEASE*
            // if (GD.G_Alliance == Alliance.Blue) {
            //   m_targetAngle = Rotation2d.fromDegrees(-60.0);
            // } else {
            //   m_targetAngle = Rotation2d.fromDegrees(-60);
            // }

           }else if(m_actualAngle >= -112.5 && m_actualAngle < -67.5){ // East
            m_targetAngle = Rotation2d.fromDegrees(-90);
          }else if(m_actualAngle >= -157.5 && m_actualAngle < -112.5){ // South East
            m_targetAngle =  GD.G_Alliance == Alliance.Blue ? Rotation2d.fromDegrees(-120.0) : Rotation2d.fromDegrees(60);
          }else if((m_actualAngle >= 157.5 && m_actualAngle <= 180.0) || (m_actualAngle <= -157.5 && m_actualAngle > -179.99)){ // South
            m_targetAngle = Rotation2d.fromDegrees(180);
          }else if(m_actualAngle <= 67.5 && m_actualAngle > 22.5){ // North West
            m_targetAngle =  GD.G_Alliance == Alliance.Blue ? Rotation2d.fromDegrees(60.0) : Rotation2d.fromDegrees(60);
          }else if(m_actualAngle <= 112.5 && m_actualAngle > 67.5){ // West
            m_targetAngle = Rotation2d.fromDegrees(90);
          }else if(m_actualAngle <= 157.5 && m_actualAngle > 112.5){ // South West
            m_targetAngle =  GD.G_Alliance == Alliance.Blue ? Rotation2d.fromDegrees(-60.0) : Rotation2d.fromDegrees(120.0);
          }
        }
        return m_targetAngle;
    }
    public Rotation2d setTargetAngle(double _angle){
      m_hyp = 0.0;
      m_actualAngle = _angle;
      m_targetAngle = Rotation2d.fromDegrees(_angle);
      return m_targetAngle;
    }
    public Rotation2d getTargetAngle(){
        return m_targetAngle;
    }
    public double getActualAngle(){
        return m_actualAngle;
    }
    public double getHyp(){
        return m_hyp;
    }
    // END Class Constrcutors
}