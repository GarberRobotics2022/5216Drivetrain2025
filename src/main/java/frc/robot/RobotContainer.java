// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import frc.robot.Autons.BlueCenterProcessorAuton;
import frc.robot.Autons.BlueLeftCoralAuton;
import frc.robot.Autons.BlueRightCoralAuton;
import frc.robot.Autons.TestingOdometry;
import frc.robot.commands.Drive.AlignToReef;
import frc.robot.commands.Drive.CycleDriveModeCommand;
import frc.robot.commands.Drive.DrivetrainDefaultCommand;
import frc.robot.commands.Drive.ResetGyroCommand;
import frc.robot.lib.EReefAlignment;
import frc.robot.lib.ISubsystem;
import frc.robot.lib.k;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static double timerTime = 0.0;

  // The robot's subsystems and commands are defined here...
  public static Set<ISubsystem> subsystems = new HashSet<>();

  private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand(
      m_driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(k.OI.DRIVER_CONTROLLER_PORT);
  public static final CommandXboxController m_operatorController = new CommandXboxController(
      k.OI.OPERATOR_CONTROLLER_PORT);

  private Notifier m_telemetry;

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public void updateDashboard() {
    SmartDashboard.putNumber("Driver Joystick Y", -m_driverController.getLeftY());
    SmartDashboard.putNumber("Driver Joystick X", -m_driverController.getLeftX());
    SmartDashboard.putNumber("Operator Joystick Y", -m_operatorController.getLeftY());
    SmartDashboard.putNumber("Operator Joystick X", -m_operatorController.getLeftX());
    SmartDashboard.putString("Drive Mode", m_driveSubsystem.m_driveMode.toString());

    SmartDashboard.putNumber("Timer time", timerTime);
    SmartDashboard.putNumber("FR Pos", m_driveSubsystem.getCancoderPos(0));
    SmartDashboard.putNumber("BR Pos", m_driveSubsystem.getCancoderPos(2));

    SmartDashboard.putNumber("Test", LimelightHelpers.getTX(""));

    Iterator<ISubsystem> it = subsystems.iterator();
    while (it.hasNext()) {
      it.next().updateDashboard(); // Comment this line out if you want ALL smartdashboard data to be stopped.
    }
  }


  // public static ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_IntakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(m_IntakeSubsystem));

    m_driveSubsystem.setDefaultCommand(m_drivetrainDefaultCommand);

    m_telemetry = new Notifier(this::updateDashboard);
    m_telemetry.startPeriodic(0.1);
    // Configure the trigger bindings
    configureBindings();

    // Set auto chooser stuff
    autoChooser.setDefaultOption("Blue Center Processor", new BlueCenterProcessorAuton(m_driveSubsystem));
    autoChooser.addOption("Blue Right Coral", new BlueRightCoralAuton(m_driveSubsystem));
    autoChooser.addOption("Odometry test", new TestingOdometry(m_driveSubsystem));
    autoChooser.addOption("Red Left Coral", new BlueLeftCoralAuton(m_driveSubsystem));

    // Put auto chooser on dashboard
    SmartDashboard.putData("Autonomous Play", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    m_driverController.y().onTrue(new CycleDriveModeCommand(m_driveSubsystem));
    m_driverController.x().onTrue(new ResetGyroCommand(m_driveSubsystem));
    m_driverController.b().whileTrue(new AlignToReef(m_driveSubsystem, EReefAlignment.RIGHT_REEF));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
}