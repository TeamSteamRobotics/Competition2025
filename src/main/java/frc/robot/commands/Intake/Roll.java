// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Roll extends Command {
  private  IntakeSubsystem m_IntakeSubsystem; // The subsystem this command controls.
  private double m_targetSpeed;
  //private int counter;
  /** Creates a new Intake. */
  public Roll(IntakeSubsystem intakeSubsystem, double targetSpeed) {
    m_IntakeSubsystem = intakeSubsystem;
    m_targetSpeed = targetSpeed;
    
  }
  //public Roll(IntakeSubsystem intakeSubsystem) {
   // m_IntakeSubsystem = intakeSubsystem;
   // m_targetSpeed = SmartDashboard.getNumber(getName(), m_targetSpeed);
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.roller(m_targetSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      m_IntakeSubsystem.roller(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
