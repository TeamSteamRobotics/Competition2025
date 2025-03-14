// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Agitator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.AgitatorSubsystem.AgitatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitatorPutCoral extends Command {
  AgitatorSubsystem m_AgitatorSubsystem;
  AgitatorState desiredState;
  /** Creates a new AgitatorPutCoral. */
  public AgitatorPutCoral(AgitatorSubsystem agitatorSubsystem) {
    m_AgitatorSubsystem = agitatorSubsystem;
    addRequirements(agitatorSubsystem);
    desiredState = m_AgitatorSubsystem.StateList.get("PUT_CORAL");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AgitatorSubsystem.rollRollerMotor(desiredState.rollMotorSpeed);
    m_AgitatorSubsystem.rotatorPid(desiredState.rotMotorParam);
    m_AgitatorSubsystem.elevatorPid(desiredState.elevMotorPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AgitatorSubsystem.currentState = desiredState;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
