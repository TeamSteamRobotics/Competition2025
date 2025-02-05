// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSim extends SubsystemBase {
  double currentAngle;
  double anglePerIteration;
  double maxAngle;
  double minAngle;
  int iterationDirection;

  /** Creates a new ArmClimber. */
  public ShooterSim() {

    currentAngle = 0;
    maxAngle = 90;
    minAngle = -90;
    anglePerIteration = 1;
    iterationDirection = 1;
  }

  Rotation3d armRotation = new Rotation3d(0, 0, 0);


  @Override
  public void periodic() {
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    // Logger.recordOutput("FinalComponentPoses", new Pose3d[]{
    //   new Pose3d(-0.309, -0.015, 0.135, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0))
    // });
    Logger.recordOutput("Shooter/FinalComponentPoses", new Pose3d[]{
      new Pose3d(-0.18, 0.25, 0.15, new Rotation3d(Math.toRadians(currentAngle), 0.0, Math.toRadians(0)))
    });
    if(currentAngle + (anglePerIteration * iterationDirection) > maxAngle || currentAngle + (anglePerIteration * iterationDirection) < minAngle){
      iterationDirection = -iterationDirection;
    }
    runArmMotor();

   
    // This method will be called once per scheduler run
  }

  public void runArmMotor(){
    currentAngle = Math.min(Math.max(currentAngle +(anglePerIteration * iterationDirection), minAngle), maxAngle) ;
    

  }
}
