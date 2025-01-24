// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFind {

  private PathFind() {}

  public static Command toPose(Pose2d targetPose) {

    return Commands.runOnce(
        () ->
            AutoBuilder.pathfindToPose(
                targetPose, new PathConstraints(2, 2, Math.toRadians(540), Math.toRadians(720))));
  }
}
