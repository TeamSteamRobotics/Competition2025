// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Pivots extends Command {
  private  IntakeSubsystem m_IntakeSubsystem; // The subsystem this command controls.
  private double m_targetPosition;
  /** Creates a new Pivots. */
  public Pivots(IntakeSubsystem intakeSubsystem, double targetPosition) {
    m_IntakeSubsystem = intakeSubsystem;
    m_targetPosition = targetPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.rollerPID(m_targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeSubsystem.rollerPID(m_targetPosition);
  }
}
