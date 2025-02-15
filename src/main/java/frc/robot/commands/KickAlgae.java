// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KickerConsts;
import frc.robot.subsystems.AlgaeKicker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KickAlgae extends Command {
  AlgaeKicker m_kicker;
  boolean atMaxHeight;
  /** Creates a new KickAlgae. */
  public KickAlgae(AlgaeKicker kicker) {
    atMaxHeight = false;
    // Use addRequirements() here to declare subsystem dependencies.
    m_kicker = kicker;
    addRequirements(m_kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_kicker.getKickerHeight() >= KickerConsts.KickerMaxHeight){
      m_kicker.runKickerRaiseMotor(0);
      atMaxHeight = true;
      return;
    }
    m_kicker.raiseKicker();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atMaxHeight;
  }
}
