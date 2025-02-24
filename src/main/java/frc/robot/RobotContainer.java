// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MotorExamples.RunMotor;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.SparkFlexMotor;
import frc.robot.subsystems.Motors.TalonFXMotor;
public class RobotContainer {

  GenericMotor talonExampleMotor;
  GenericMotor revExampleMotor;
  
  public RobotContainer() {

    talonExampleMotor = new TalonFXMotor(0, "rio");
    revExampleMotor = new SparkFlexMotor(1);

    configureBindings();
  }

  private void configureBindings() {
   new Trigger(() -> true)
      .whileTrue(new RunMotor(revExampleMotor, 0.5));
  
   new Trigger(() -> true)
      .whileTrue(new RunMotor(talonExampleMotor, 0.5));
  }
}
