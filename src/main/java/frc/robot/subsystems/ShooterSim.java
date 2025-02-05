// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSim extends SubsystemBase {
  double currentAngle;
  double anglePerIteration;
  double maxAngle;
  double minAngle;
  int shooter_iterationDirection;

  double currentExtension;
  double maxExtension;
  double minExtension;
  double extensionIteration;
  int extender_iterationDirection;

  /** Creates a new ArmClimber. */
  public ShooterSim() {

    currentAngle = 45;
    maxAngle = 90;
    minAngle = -90;
    anglePerIteration = 1;
    shooter_iterationDirection = 1;

    currentExtension = 0.39;
    maxExtension = 0.39;
    minExtension = 0;
    extensionIteration = 0.005;
    extender_iterationDirection = 1;
  }

  Rotation3d armRotation = new Rotation3d(0, 0, 0);


  @Override
  public void periodic() {
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    // Logger.recordOutput("FinalComponentPoses", new Pose3d[]{
    //   new Pose3d(-0.309, -0.015, 0.135, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0))
    // });

     // Define the shooter arm base (it rotates)
    Pose3d shooterPose = new Pose3d(
        -0.18, 0.25, 0.15,  // Base arm position
        new Rotation3d(Math.toRadians(currentAngle), 0.0, 0.0) // Arm rotation
    );

    // Define the extension *relative to* the shooterPose
    Pose3d extensionPose = shooterPose.transformBy(new Transform3d(
        new Translation3d(0.0, 0.0, currentExtension), // Extend outward along local X-axis
        new Rotation3d(0.0, 0.0, 0.0) // No extra rotation needed
    ));
    Logger.recordOutput("Shooter/FinalComponentPoses", new Pose3d[]{shooterPose
    });
    Logger.recordOutput("Shooter/Extension/FinalComponentPoses", new Pose3d[] {
    extensionPose
  });
    if(currentAngle + (anglePerIteration * shooter_iterationDirection) > maxAngle || currentAngle + (anglePerIteration * shooter_iterationDirection) < minAngle){
      shooter_iterationDirection = -shooter_iterationDirection;
    }
    if(currentExtension + (extensionIteration * extender_iterationDirection) > maxExtension || currentExtension + (extensionIteration * extender_iterationDirection) < minExtension){
      extender_iterationDirection = -extender_iterationDirection;
    }
    runArmMotor();

   
    // This method will be called once per scheduler run
  }

  public void runArmMotor(){
    currentAngle = Math.min(Math.max(currentAngle +(anglePerIteration * shooter_iterationDirection), minAngle), maxAngle);
    currentExtension = Math.min(Math.max(currentExtension +(extensionIteration * extender_iterationDirection), minExtension), maxExtension);
    

  }
}
