//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.k;


public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_cancoder;
    private String m_name;
    private double m_driveSetVelocity_mps = 0;
    private double m_steerSetAngle_deg = 0;
    private double m_driveActualVelocity_mps = 0;
    private double m_steerActualAngle_deg = 0;
    private double m_steerVolts = 0.0;
    private double m_driveVolts = 0.0;
     private PIDController m_steerPID = new PIDController(k.STEER.PID_Kp, k.STEER.PID_Ki, 0);
    // TODO Determine PID for mps to volts
    private PIDController m_drivePID = new PIDController(k.DRIVE.PID_Kp,k.DRIVE.PID_Ki, 0.0);
    // TODO Determine FF kv,ks for Volts per mps. First no load single motor test showed .11 volts per RPS
    // TODO Test by setting various voltages and measuring drive velocity to get kv. ks is the amount it takes to move the robot 
    private SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(k.DRIVE.PID_Ks, k.DRIVE.PID_Kv);
    private VoltageOut m_steerVoltageOut = new VoltageOut(0.0);
    private VoltageOut m_driveVoltageOut = new VoltageOut(0.0);
    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    public SwerveModule(SwerveModuleConstants _constants) {
        
        m_driveMotor = new TalonFX(_constants.m_driveMotorId, k.ROBORIO_CAN_IDS.NAME);     // Create a TalonFX object for the Drive Motor. Give it the CAN ID and Name of the CAN bus
        m_steerMotor = new TalonFX(_constants.m_steerMotorId,  k.ROBORIO_CAN_IDS.NAME);     // Create a TalonFX object for the Steer Motor. Give it the CAN ID and Name of the CAN bus
        m_cancoder = new CANcoder(_constants.m_CANcoderId,  k.ROBORIO_CAN_IDS.NAME);        // Create a CANCoder object. The CANCoder is used for the absolute wheel angle

        m_name = _constants.m_name;                                             // Set the name of this module to the module level variable
        // Configure Drive Motor
        TalonFXConfiguration talonDriveConfigs = new TalonFXConfiguration();    // Create a new Configuration for a Talon motor
        
        talonDriveConfigs.MotorOutput.Inverted =                                // If the motor is inverted set the configuration parameter
            _constants.m_isDriveMotorReversed 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
        m_driveMotor.getConfigurator().apply(talonDriveConfigs);                // Apply the drive motor configuration
       
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(50).withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(50);
        m_driveMotor.getConfigurator().apply(currentConfigs);

        // Configure Steer Motor
        m_steerPID.enableContinuousInput(-180.0, +180.0);                    // Set the PID to allow role overs and adjust for optimization
        TalonFXConfiguration talonSteerConfigs = new TalonFXConfiguration();    // Create a new Talon configuration object to store the changes
        
        talonSteerConfigs.MotorOutput.Inverted = 
            _constants.m_isSteerMotorReversed 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        m_steerMotor.getConfigurator().apply(talonSteerConfigs);
        m_steerMotor.setNeutralMode(NeutralModeValue.Brake);
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = _constants.m_CANcoderOffset_deg; // Set the CANCoder to the straight ahead value in rotations
        // cancoderConfigs.MagnetSensor.MagnetOffset = 0.0; // Uncomment to check actual offset. Comment the above line
        m_cancoder.getConfigurator().apply(cancoderConfigs);

        m_steerMotor.setPosition(m_cancoder.getPosition().getValueAsDouble() * k.STEER.GEAR_RATIO);
        
    }

    /** Set the desired Drive Speed and Steer Angle. This method commands the motors.
     * 
     * @param _state The module state which contains Steer Angle and Drive Speed
     */
    public void setDesiredState(SwerveModuleState _state, boolean _enableSteer, boolean _enableDrive) {
        // Optimize the angle so the wheel will not rotate more than 90 deg
        SwerveModuleState optimized = _state;//.optimize(_state, m_internalState.angle);
        optimized.optimize(m_internalState.angle);
        
        if(_enableSteer){
            m_steerSetAngle_deg = optimized.angle.getDegrees(); // Get the angle in degrees that we want to set
            m_steerActualAngle_deg = m_steerMotor.getPosition().getValueAsDouble() * 360.0 / k.STEER.GEAR_RATIO;
            m_steerVolts = m_steerPID.calculate(m_steerActualAngle_deg,m_steerSetAngle_deg);
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(m_steerVolts).withEnableFOC(true));
        }
        if(_enableDrive){
            m_driveSetVelocity_mps = optimized.speedMetersPerSecond; // Get the velocity we want to set
            m_driveActualVelocity_mps = m_driveMotor.getVelocity().getValueAsDouble() /  k.DRIVE.WHEEL_MotRotPerMeter;
            m_driveVolts = m_drivePID.calculate(m_driveActualVelocity_mps, m_driveSetVelocity_mps);
            m_driveVolts = MathUtil.clamp(m_driveVolts, -6, 6); // Limit the amount the PID can contribute to the Feedforward
            m_driveVolts = m_driveVolts + m_driveFF.calculate(m_driveSetVelocity_mps);
            
            
            m_driveMotor.setControl(m_driveVoltageOut.withOutput(m_driveVolts).withEnableFOC(true));
        }
    }
    public void stopMotors(){
        m_steerMotor.setControl(m_steerVoltageOut.withOutput(0).withEnableFOC(true));
        m_driveMotor.setControl(m_driveVoltageOut.withOutput(0).withEnableFOC(true));
    }
    public SwerveModulePosition getPosition(boolean _refresh) {
        double drive_rot =  m_driveMotor.getPosition().getValueAsDouble();
        double angle_rot =  m_steerMotor.getPosition().getValueAsDouble();
        // anagle_rot is the Motor rotations. Apply the gear ratio to get wheel rotations for steer
        angle_rot = angle_rot / k.STEER.GEAR_RATIO;
        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / k.DRIVE.WHEEL_MotRotPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public void updateDashboard(){
        //SmartDashboard.putNumber(m_name+"_set_deg", m_steerSetAngle_deg);
        SmartDashboard.putNumber(m_name+"_set_mps", m_driveSetVelocity_mps);
        SmartDashboard.putNumber(m_name+"_act_deg", m_steerActualAngle_deg);
        SmartDashboard.putNumber(m_name+"_act_mps", m_driveActualVelocity_mps);
       
        //SmartDashboard.putNumber(m_name+"_CC_rot", m_cancoder.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber(m_name + "_steerVolts", m_steerVolts);
        SmartDashboard.putNumber(m_name + "_driveVolts", m_driveVolts);
        SmartDashboard.putNumber(m_name + "_driveOutputVolts", m_driveMotor.getMotorVoltage().getValueAsDouble());
    }

    public double getCancoderPos() {
        return m_cancoder.getAbsolutePosition().getValue().in(Degrees);
    }
}