// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Newton;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  SparkFlex rollerMotor = new SparkFlex(0, MotorType.kBrushless);
  SparkFlex pivotMotor = new SparkFlex(0, MotorType.kBrushless);
  
  AbsoluteEncoder rollerEncoder;
  RelativeEncoder pivotEncoder;

  PIDController rollerPid = new PIDController(0, 0, 0);
  PIDController pivotPid = new PIDController(0, 0, 0);

  double m_targetRollerSpeed;
  double m_targetPivotSpeed;

  public IntakeSubsystem() {
    rollerPid.setTolerance(0);
    pivotPid.setTolerance(0);

    rollerEncoder = rollerMotor.getAbsoluteEncoder();
    pivotEncoder = pivotMotor.getEncoder();


  }

  public boolean rollerPID(double targetrollerSpeed) {
    m_targetRollerSpeed = targetrollerSpeed;

    double pidOutputRoller = rollerPid.calculate(rollerEncoder.getVelocity(), targetrollerSpeed);
    
    rollerMotor.set(pidOutputRoller);

    return (rollerPid.atSetpoint());
  }
  public boolean pivotPID(double targetpivotSpeed) {
    m_targetPivotSpeed = targetpivotSpeed;

    double pidOutputpivot = pivotPid.calculate(pivotEncoder.getVelocity(), targetpivotSpeed);
    
    pivotMotor.set(pidOutputpivot);

    return (pivotPid.atSetpoint());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
