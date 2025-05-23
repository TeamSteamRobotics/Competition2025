// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollGreen extends Command {
  ShooterSubsystem m_shooter;
  double m_speed;
  boolean m_ignoreBeambreak;
  boolean beamBroken;

  /** Creates a new RollGreen. */
  public RollGreen(ShooterSubsystem shooter, double speed, boolean ignoreBeambreak) {
    
    m_shooter = shooter;
    m_speed = speed;
    m_ignoreBeambreak = ignoreBeambreak;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.RunGreenRoller(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.RunGreenRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    beamBroken = m_shooter.beamBroken();
    if(m_ignoreBeambreak){
      return false;
    }else{
      return beamBroken;
    }
  }
}
