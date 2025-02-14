// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionDistanceTester extends Command {
  /** Creates a new VisionDistanceTester. */
  // UNTESTED
  public VisionSubsystem m_vis;
  public Drive m_drive;
  double propOffset;
  public VisionDistanceTester(VisionSubsystem vis, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vis = vis;
    m_drive = drive;
    addRequirements(m_vis);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distToAprilTagEight = m_vis.getFiducialDistanceToCamera()[7];
    propOffset = ProportionalVelControlBasedOnDistance(distToAprilTagEight);
    m_drive.runVelocity(new ChassisSpeeds(Math.max(Math.min(propOffset, 1), -1), 0, 0));
  }

  private double ProportionalVelControlBasedOnDistance(double dist){
    double offset = dist - 1;
    return offset * 0.35; // who knows what this constant should be
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(propOffset < 0.1 || propOffset > -0.1){
      return true;
    }
    return false;
  }
}
