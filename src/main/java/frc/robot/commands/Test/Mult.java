// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import frc.robot.subsystems.Test.ButtonTest;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Mult extends Command {
  /** Creates a new Add. */

  private ButtonTest test;

  public Mult(ButtonTest k_test) {
    test = k_test;
    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    test.mult();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    test.stopButtonTest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
