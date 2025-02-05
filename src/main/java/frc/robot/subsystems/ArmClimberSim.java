// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmClimberSim extends SubsystemBase {
  double currentAngle;
  double anglePerIteration;
  double maxAngle;
  double minAngle;

  /** Creates a new ArmClimber. */
  public ArmClimberSim() {

    currentAngle = 90;
    maxAngle = 90;
    minAngle = 0;
    anglePerIteration = 10;
  }

  Rotation3d armRotation = new Rotation3d(0, 0, 0);


  @Override
  public void periodic() {
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    // Logger.recordOutput("FinalComponentPoses", new Pose3d[]{
    //   new Pose3d(-0.309, -0.015, 0.135, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0))
    // });
    Logger.recordOutput("FinalComponentPoses", new Pose3d[]{
      new Pose3d(-0.309, -0.015, 0.135, new Rotation3d(0.0, Math.toRadians(currentAngle), 0.0))
    });
    //runArmMotor();
    // This method will be called once per scheduler run
  }

  public void runArmMotor(){
    currentAngle += Math.min(Math.max(currentAngle + anglePerIteration, minAngle), maxAngle);
      


  }
}
