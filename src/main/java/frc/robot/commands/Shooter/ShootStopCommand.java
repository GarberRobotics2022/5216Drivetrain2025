// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;

// public class ShootStopCommand extends Command {
  
//   IntakeSubsystem _IntakeSubsystem;
//   boolean done = false; 

// /** Creates a new ShootStopCommand. */
//   public ShootStopCommand(IntakeSubsystem m_IntakeSubsystem) {
//     addRequirements(m_IntakeSubsystem);
//     _IntakeSubsystem = m_IntakeSubsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
     
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     _IntakeSubsystem.stopShooter();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return done;
//   }
// }
