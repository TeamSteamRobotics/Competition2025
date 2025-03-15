// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PathPlanner;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeIn extends InstantCommand {
  IntakeSubsystem m_Intake;
  double m_pivotInitialPosition;
  double setPosition;
  IntakeSubsystem pivotMotor;

  public IntakeIn(IntakeSubsystem intake, double pivotInitialPosition){
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    m_pivotInitialPosition = pivotInitialPosition;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.pivotPID(m_pivotInitialPosition);  
  }
}
