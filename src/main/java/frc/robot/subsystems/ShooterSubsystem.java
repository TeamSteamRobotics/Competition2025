// ShooterSubsystem.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.TalonFXMotor;
import frc.robot.subsystems.Motors.SparkFlexMotor;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.ShooterPid;

/**
 * ShooterSubsystem manages the motors and control logic for the robot's shooter.
 * It includes PID (Proportional-Integral-Derivative) controllers to maintain precise motor speeds.
 */
public class ShooterSubsystem extends SubsystemBase {
  // These are motor controllers for the shooter. They control the front and back motors.
  GenericMotor frontShooterMotor = new SparkFlexMotor(Constants.Shooter.frontRollerId);
  GenericMotor backShooterMotor = new SparkFlexMotor(Constants.Shooter.backRollerId);

  // These are encoders that measure the rotation speed of the motors.
  AbsoluteEncoder topShooterEncoder;
  AbsoluteEncoder bottomShooterEncoder;

  // PID controllers adjust motor speed to match the desired target speed.
  PIDController topShooterPid = new PIDController(ShooterPid.kP, ShooterPid.kI, ShooterPid.kD);
  PIDController bottomShooterPid = new PIDController(ShooterPid.kP, ShooterPid.kI, ShooterPid.kD);

  double m_targetSpeed;

  public boolean overrideDefault;

  /**
   * Constructor: Sets up the shooter subsystem.
   * This includes initializing the encoders and configuring the PID controllers with a tolerance.
   */
  public ShooterSubsystem() {
    overrideDefault = false;
    // Configure the PID controllers to stop adjusting when close enough to the target.
    topShooterPid.setTolerance(ShooterPid.tolerance);
    bottomShooterPid.setTolerance(ShooterPid.tolerance);
  }

  /**
   * Runs the shooter motors at a target speed using PID control.
   * PID control ensures the motors reach and maintain the desired speed.
   * @param targetSpeed The speed we want the motors to achieve (measured in RPM or similar).
   * @return True if both motors are at the target speed, false otherwise.
   */
  public boolean ShootPID(double targetSpeed) {
    m_targetSpeed = targetSpeed;
    // Calculate how much to adjust the motor speed to reach the target.
    double pidOutputFront = topShooterPid.calculate(frontShooterMotor.getVelocity(), m_targetSpeed);
    //FIXME: MAY BE THE OTHER WAY ARROUNDDD
    double pidOutputBack = bottomShooterPid.calculate(backShooterMotor.getVelocity(), -m_targetSpeed);

    // Set the motors to the calculated speeds.
    frontShooterMotor.set(pidOutputFront);
    backShooterMotor.set(pidOutputBack);

    // Check if both motors have reached the desired speed.
    return (topShooterPid.atSetpoint() && bottomShooterPid.atSetpoint());
  }

  
  /**
   * Stops the motors completely. This is used when the shooter is no longer needed.
   */
  public void StopMotor() {
    frontShooterMotor.set(0); // Stop the top motor.
    backShooterMotor.set(0); // Stop the bottom motor.
  }

  /**
   * Retrieves the current speeds of both motors.
   * This can be useful for debugging or monitoring during a match.
   * @return An array where index 0 is the top motor speed and index 1 is the bottom motor speed.
   */
  public double[] getSpeeds() {
    double[] speeds = {frontShooterMotor.getVelocity(), backShooterMotor.getVelocity()};
    return speeds;
  }
  public double getTargetSpeed(){ return m_targetSpeed; }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/FrontMotorSpeed", frontShooterMotor.getVelocity());
    Logger.recordOutput("Shooter/BackMotorSpeed", backShooterMotor.getVelocity());
    // This method will be called once per scheduler run
  }
}
