// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Agitator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AgitatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/** Possibly overkill for now, but could be useful later */
public class AgitatorStateMachine extends InstantCommand {
  AgitatorSubsystem m_AgitatorSubsystem;
  public AgitatorStateMachine(AgitatorSubsystem agitatorSubsystem) {
    m_AgitatorSubsystem = agitatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public Command transitState(String state){
    return m_AgitatorSubsystem.commandTable.get(m_AgitatorSubsystem.StateList.get(state).name).get(); // worst line of code i ever wrote
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
