// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollGreen extends Command {
  ShooterSubsystem m_shooter;
  double m_speed;
  boolean m_ignoreBeambreak;
  CommandXboxController m_controller;

  /** Creates a new RollGreen. */
  public RollGreen(ShooterSubsystem shooter, double speed, boolean ignoreBeambreak, CommandXboxController controller) {
    
    m_shooter = shooter;
    m_speed = speed;
    m_ignoreBeambreak = ignoreBeambreak;
    m_controller = controller;

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

    if(!m_ignoreBeambreak){
      try{
      m_controller.setRumble(RumbleType.kBothRumble, 0.5);
      Thread.sleep(500);
      m_controller.setRumble(RumbleType.kBothRumble, 0);
      } catch(Exception e){
        System.out.println(e);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ignoreBeambreak){
      return false;
    }else{
      return m_shooter.beamBroken();
    }
  }
}
