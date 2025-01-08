package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX shooterTopMotor = new TalonFX(31); // broken motor ig
    public TalonFX shooterBottomMotor = new TalonFX (32);

    private DigitalInput beamBreakSensor = new DigitalInput(0);

    private CANSparkMax intakeMotor = new CANSparkMax(33, MotorType.kBrushless);

    /** Creates a new Intake. */
    public IntakeSubsystem() {
        // Initialize motor controllers with their IDs
        shooterTopMotor.setNeutralMode(NeutralModeValue.Brake);
        shooterBottomMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // Shooter \\
    /**
     * Sets the speed of the intake motors.
     * @param speed how fast you want the thingy to spinny lol.
     */
    public void setShooterSpeed(double speed) {
        shooterTopMotor.set(speed);
        shooterBottomMotor.set(speed);
    }
    public void stopShooter() {
        setShooterSpeed(0);
    }
    /**
     * 
     * @return The average of the shooter motor speeds
     */
    public double getShooterSpeed() {
      return ((shooterTopMotor.getRotorVelocity().getValueAsDouble() / 512 * 5) + (shooterBottomMotor.getRotorVelocity().getValueAsDouble() / 512 * 5)) / 2;
    }

    // Intake \\

    public void setIntakeDesiredSpeed(double speed) {
      intakeMotor.set(speed);
    }

    public void stopIntake() {
      intakeMotor.set(0);
    }

    public boolean beamSensorTriggered() { 
      return !beamBreakSensor.get();
    }

    @Override
    public void periodic() {
      SmartDashboard.putBoolean("Speaker Ready",
        (shooterTopMotor.getVelocity().getValueAsDouble() + shooterBottomMotor.getVelocity().getValueAsDouble()) / 2 >= 55);
        // This method will be called once per scheduler run
    }
}
