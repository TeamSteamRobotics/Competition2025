// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopMotors extends InstantCommand {
  ClimbSubsystem m_climb;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shoot;
  public StopMotors(ClimbSubsystem climb, IntakeSubsystem intake, ShooterSubsystem shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb, intake, shoot);
    m_climb = climb;
    m_intake = intake;
    m_shoot = shoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.stopClimb();
    m_climb.stopWinch();
    m_intake.pivot(0);
    m_intake.roller(0);
    m_shoot.RunGreenRoller(0);
    m_shoot.RunOrangeRollers(0);
  }
}
