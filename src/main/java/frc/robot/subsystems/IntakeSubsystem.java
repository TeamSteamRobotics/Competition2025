// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Newton;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeMotors.PivotPid;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.TalonFXMotor;
import frc.robot.subsystems.Motors.SparkFlexMotor;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  // Motor controllers for the intake roller and pivot mechanism

  GenericMotor rollerMotor;
  GenericMotor pivotMotor;
  
  // PID controllers for controlling the roller and pivot speeds
  //PIDController rollerPid = new PIDController(Roller.kP, Roller.kI, Roller.kD);
  PIDController pivotPid = new PIDController(PivotPid.kP, PivotPid.kI, PivotPid.kD);

  // Variables to store target speeds
  double m_targetRollerSpeed;
  double m_targetPivotPosition;

  public IntakeSubsystem() {
    rollerMotor = new TalonFXMotor(Constants.IntakeMotors.rollerId, "rio");
    pivotMotor = new SparkFlexMotor(Constants.IntakeMotors.pivotId);

    pivotMotor.setPosition(0);
    
    // Setting initial PID tolerances
    //rollerPid.setTolerance(Roller.tolerance);
    pivotPid.setTolerance(Constants.IntakeMotors.PivotPid.tolerance);

  }

  /**
   * Controls the roller motor using a PID controller.
   * @param targetRollerSpeed The desired speed for the roller motor.
   * @return true if the roller motor speed is at the target setpoint.
   */
  // public boolean rollerPID(double targetRollerSpeed) {
  //   m_targetRollerSpeed = targetRollerSpeed;

  //   // Compute the PID output for the roller motor based on encoder velocity feedback
  //   double pidOutputRoller = rollerPid.calculate(rollerMotor.getVelocity(), m_targetRollerSpeed);
    
  //   // Apply the computed PID output to the roller motor
  //   rollerMotor.set(pidOutputRoller);
  
  //   // Return whether the PID controller has reached the setpoint
  //   return pivotPid.atSetpoint();
  // }

  public void roller(double rollerSpeed){
    Logger.recordOutput("Intake/RollerSpeed", rollerMotor.getVelocity());
    m_targetRollerSpeed = rollerSpeed;
    rollerMotor.set(m_targetRollerSpeed);
  }
  
  
  /**
   * Controls the pivot motor using a PID controller.
   * @param targetPivotPosition The desired speed for the pivot motor.
   * @return true if the pivot motor speed is at the target setpoint.
   */
  public boolean pivotPID(double targetPivotPosition) {
    m_targetPivotPosition = targetPivotPosition;

    // Compute the PID output for the pivot motor based on encoder
    double pidOutputPivot = pivotPid.calculate(pivotMotor.getPosition() / Constants.IntakeMotors.pivotGearboxRatio, m_targetPivotPosition);
    
    // Apply the computed PID output to the pivot motor
    pivotMotor.set(pidOutputPivot);

    // Return whether the PID controller has reached the setpoint
    return pivotPid.atSetpoint();
  }

  public void pivot(double pivotSpeed){
    pivotMotor.set(pivotSpeed);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/FinalComponentPoses", new Pose3d[]{
      new Pose3d(-0.25, -0.3, 0.16, new Rotation3d(Math.toRadians((pivotMotor.getPosition() / Constants.IntakeMotors.pivotGearboxRatio)* 360), 0.0, 0.0))
    });
    // This method will be called once per scheduler run
  }
}