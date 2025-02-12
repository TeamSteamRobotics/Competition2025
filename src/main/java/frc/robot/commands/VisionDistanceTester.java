// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionDistanceTester extends InstantCommand {
  VisionSubsystem m_vis;
  Drive m_drive;
  public VisionDistanceTester(VisionSubsystem vis, Drive driveSub) {
    // screw this
    m_vis = vis;
    m_drive = driveSub;

    addRequirements(m_drive);
    addRequirements(m_vis);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double fidEightDist = m_vis.getFiducialDistanceToCamera()[7];
    if(fidEightDist == 0.0d){
      return;
    }
    m_drive.runVelocity(new ChassisSpeeds(Math.min(fidEightDist, 1), 0, 0)); //what
    return;
  }
}
