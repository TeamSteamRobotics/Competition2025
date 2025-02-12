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
  
  // Motor controllers for the intake roller and pivot mechanism
  SparkFlex rollerMotor = new SparkFlex(0, MotorType.kBrushless);
  SparkFlex pivotMotor = new SparkFlex(0, MotorType.kBrushless);
  
  // Encoders for tracking the position and speed of the motors
  RelativeEncoder rollerEncoder;
  AbsoluteEncoder pivotEncoder;

  // PID controllers for controlling the roller and pivot speeds
  PIDController rollerPid = new PIDController(0, 0, 0);
  PIDController pivotPid = new PIDController(0, 0, 0);

  // Variables to store target speeds
  double m_targetRollerSpeed;
  double m_targetPivotSpeed;

  public IntakeSubsystem() {
    // Setting initial PID tolerances
    rollerPid.setTolerance(0);
    pivotPid.setTolerance(0);

    // Initializing encoders from motor controllers
    rollerEncoder = rollerMotor.getEncoder();
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
  }

  /**
   * Controls the roller motor using a PID controller.
   * @param targetrollerSpeed The desired speed for the roller motor.
   * @return true if the roller motor speed is at the target setpoint.
   */
  public boolean rollerPID(double targetrollerSpeed) {
    m_targetRollerSpeed = targetrollerSpeed;

    // Compute the PID output for the roller motor based on encoder velocity feedback
    double pidOutputRoller = rollerPid.calculate(rollerEncoder.getVelocity(), targetrollerSpeed);
    
    // Apply the computed PID output to the roller motor
    rollerMotor.set(pidOutputRoller);

    // Return whether the PID controller has reached the setpoint
    return (rollerPid.atSetpoint());
  }
  
  /**
   * Controls the pivot motor using a PID controller.
   * @param targetpivotSpeed The desired speed for the pivot motor.
   * @return true if the pivot motor speed is at the target setpoint.
   */
  public boolean pivotPID(double targetpivotSpeed) {
    m_targetPivotSpeed = targetpivotSpeed;

    // Compute the PID output for the pivot motor based on encoder velocity feedback
    double pidOutputpivot = pivotPid.calculate(pivotEncoder.getVelocity(), targetpivotSpeed);
    
    // Apply the computed PID output to the pivot motor
    pivotMotor.set(pidOutputpivot);

    // Return whether the PID controller has reached the setpoint
    return (pivotPid.atSetpoint());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
