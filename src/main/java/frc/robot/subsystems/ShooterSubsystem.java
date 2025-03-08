// ShooterSubsystem.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.TalonFXMotor;
import frc.robot.subsystems.Motors.SparkFlexMotor;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.ShooterPid;
import java.util.Map.Entry;

/**
 * ShooterSubsystem manages the motors and control logic for the robot's shooter.
 * It includes PID (Proportional-Integral-Derivative) controllers to maintain precise motor speeds.
 */
public class ShooterSubsystem extends SubsystemBase {
  // These are motor controllers for the shooter. They control the front and back motors.
  GenericMotor frontShooterMotor = new SparkFlexMotor(Constants.Shooter.frontRollerId);
  GenericMotor backShooterMotor = new SparkFlexMotor(Constants.Shooter.backRollerId);
  GenericMotor greenShooterMotor = new SparkFlexMotor(Constants.Shooter.greenRollerId);

  // These are encoders that measure the rotation speed of the motors.
  AbsoluteEncoder topShooterEncoder;
  AbsoluteEncoder bottomShooterEncoder;

  // PID controllers adjust motor speed to match the desired target speed.
  PIDController topShooterPid = new PIDController(ShooterPid.kP, ShooterPid.kI, ShooterPid.kD);
  PIDController bottomShooterPid = new PIDController(ShooterPid.kP, ShooterPid.kI, ShooterPid.kD);
  PIDController greenShooterPid = new PIDController(ShooterPid.kP, ShooterPid.kI, ShooterPid.kD);

  double m_targetSpeed;

  public boolean overrideDefault;

  DigitalInput beamBreak;
  TreeMap<Double, Double> speedLookupTable = new TreeMap<Double, Double>();

  AprilVisionSubsystem vision;

  /**
   * Constructor: Sets up the shooter subsystem.
   * This includes initializing the encoders and configuring the PID controllers with a tolerance.
   */
  public ShooterSubsystem() {
    vision = new AprilVisionSubsystem();
    beamBreak = new DigitalInput(4);
    overrideDefault = false;
    // Configure the PID controllers to stop adjusting when close enough to the target.
    topShooterPid.setTolerance(ShooterPid.tolerance);
    bottomShooterPid.setTolerance(ShooterPid.tolerance);
    speedLookupTable.put(0.3048, 0.22);
    speedLookupTable.put(0.381, 0.28);
    speedLookupTable.put(0.508, 0.33);
    speedLookupTable.put(0.635, 0.37);
    speedLookupTable.put(0.762, 0.4);
    speedLookupTable.put(0.847725, 0.45);
    speedLookupTable.put(0.889, 0.45);
    speedLookupTable.put(1.016, 0.47);
    speedLookupTable.put(1.143, 0.48);
    speedLookupTable.put(1.27, 0.5);
    speedLookupTable.put(1.524, 0.6);
    speedLookupTable.put(1.778, 0.77);

  }

  /**
   * Runs the shooter motors at a target speed using PID control.
   * PID control ensures the motors reach and maintain the desired speed.
   * @param targetSpeed The speed we want the motors to achieve (measured in RPM or similar).
   * @return True if both motors are at the target speed, false otherwise.
   */
  public boolean Shoot(double targetSpeed) {
    m_targetSpeed = targetSpeed;
    // Calculate how much to adjust the motor speed to reach the target.
    double pidOutputFront = topShooterPid.calculate(frontShooterMotor.getVelocity(), m_targetSpeed);
    //FIXME: MAY BE THE OTHER WAY ARROUNDDD
    double pidOutputBack = bottomShooterPid.calculate(backShooterMotor.getVelocity(), -m_targetSpeed);

    double pidGreenOutput = topShooterPid.calculate(greenShooterMotor.getVelocity(), m_targetSpeed);

    // Set the motors to the calculated speeds.
    //greenShooterMotor.set(pidGreenOutput);
    //frontShooterMotor.set(pidOutputFront);
    //backShooterMotor.set(pidOutputBack);
    //greenShooterMotor.set(-m_targetSpeed);
    frontShooterMotor.set(-m_targetSpeed);
    backShooterMotor.set(-m_targetSpeed);

    // Check if both motors have reached the desired speed.
    return (topShooterPid.atSetpoint() && bottomShooterPid.atSetpoint());
    
  }

  public void RunGreenRoller(double speed){
    greenShooterMotor.set(-speed);
  }
  
  /**
   * Stops the motors completely. This is used when the shooter is no longer needed.
   */
  public void StopMotor() {
    frontShooterMotor.set(0); // Stop the top motor.
    backShooterMotor.set(0); // Stop the bottom motor.
    greenShooterMotor.set(0); // Stop the green motor.
  }

  public boolean beamBroken(){
    return !beamBreak.get();
  }

  /*public double[] getVelocities(){
    double[] returnValues = {greenShooterMotor.getVelocity(), frontShooterMotor.getVelocity(), backShooterMotor.getVelocity()};
    return returnValues;
  }*/

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

    Logger.recordOutput("Shooter/Beambreak", beamBroken());

    Logger.recordOutput("AprilTag4 Distance", vision.getCoordinates(4, ReturnTarget.TARGET).z);
 
    // This method will be called once per scheduler run
  }
  public double lookupShootSpeed(double dist){
    if(dist < 0.3048 || dist > 1.778){
      // System.out.println("Distance exceeds bounds. Returning default speed");
      // System.out.println(dist);
      return Constants.Shooter.defaultSpeed;
    }
    Entry<Double, Double> lower = speedLookupTable.floorEntry(dist); // just copy-pasted from Zach's code
    Entry<Double, Double> upper = speedLookupTable.ceilingEntry(dist);
    if(lower == null)
      return upper.getValue();
    if(upper == null)
      return lower.getValue();
    double slope = (upper.getValue() - lower.getValue()) / (upper.getKey() - lower.getKey());
    return (slope * (dist - lower.getKey()) + lower.getValue());
  }
}

