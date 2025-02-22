// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Newton;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorIdConstants;
import frc.robot.Constants.Intake.*;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  // Motor controllers for the intake roller and pivot mechanism
  private final CANBus kCANBus = new CANBus("rio");

  TalonFX rollerMotor = new TalonFX(motorIdConstants.rollerId, kCANBus);
  SparkFlex pivotMotor = new SparkFlex(motorIdConstants.pivotId, MotorType.kBrushless);
  
  // Encoders for tracking the position and speed of the motors
  RelativeEncoder rollerEncoder;
  AbsoluteEncoder pivotEncoder;
  

  // PID controllers for controlling the roller and pivot speeds
  PIDController rollerPid = new PIDController(Roller.kP, Roller.kI, Roller.kD);
  PIDController pivotPid = new PIDController(Pivot.kP, Pivot.kI, Pivot.kD);

  // Variables to store target speeds
  double m_targetRollerSpeed;
  double m_targetPivotPosition;

  public IntakeSubsystem() {

     var rollerConfig = new TalonFXConfiguration();
    
    // Setting initial PID tolerances
    rollerPid.setTolerance(Roller.tolerance);
    pivotPid.setTolerance(Pivot.tolerance);

    rollerMotor.getConfigurator().apply(rollerConfig);
    // Initializing encoders from motor controllers
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
  }

  /**
   * Controls the roller motor using a PID controller.
   * @param targetRollerSpeed The desired speed for the roller motor.
   * @return true if the roller motor speed is at the target setpoint.
   */
  public boolean rollerPID(double targetRollerSpeed) {
    m_targetRollerSpeed = targetRollerSpeed;

    // Compute the PID output for the roller motor based on encoder velocity feedback
    double pidOutputRoller = 0.2;
    
    // Apply the computed PID output to the roller motor
    rollerMotor.setControl(new DutyCycleOut(pidOutputRoller));
  
    // Return whether the PID controller has reached the setpoint
    return false;
  }
  
  /**
   * Controls the pivot motor using a PID controller.
   * @param targetPivotPosition The desired speed for the pivot motor.
   * @return true if the pivot motor speed is at the target setpoint.
   */
  public boolean pivotPID(double targetPivotPosition) {
    m_targetPivotPosition = targetPivotPosition;

    // Compute the PID output for the pivot motor based on encoder velocity feedback
    double pidOutputPivot = pivotPid.calculate(pivotEncoder.getPosition(), targetPivotPosition);
    
    // Apply the computed PID output to the pivot motor
    pivotMotor.set(pidOutputPivot);

    // Return whether the PID controller has reached the setpoint
    return (pivotPid.atSetpoint());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}