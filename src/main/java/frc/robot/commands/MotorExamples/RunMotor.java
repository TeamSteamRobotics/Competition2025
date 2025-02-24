// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MotorExamples;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motors.GenericMotor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunMotor extends Command {
  /** Creates a new MotorExamples. */
  GenericMotor m_motor;
  double m_speed;

  public RunMotor(GenericMotor motor, double speed) {
    m_motor = motor;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled. (Useful for "one and done" commands)
  @Override
  public void initialize() {
    m_motor.set(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled. (Not neccesary if the command instantly ends)
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
