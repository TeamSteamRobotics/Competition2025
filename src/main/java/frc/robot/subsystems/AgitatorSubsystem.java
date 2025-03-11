// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.SparkMaxMotor;



public class AgitatorSubsystem extends SubsystemBase {

  GenericMotor elevatorMotor;
  GenericMotor rollerMotor;
  GenericMotor rotatorMotor;

  double targetElevatorPosition;
  double rollerSpeed;
  double targetRotatorPosition;
  double elevatorSpeed;
  /** Creates a new AgitatorSubsystem. */
  public AgitatorSubsystem() {

    elevatorMotor = new SparkMaxMotor(Constants.AgitatorMotors.elevatorId, Constants.AgitatorMotors.ElevatorPid.kP, Constants.AgitatorMotors.ElevatorPid.kI, Constants.AgitatorMotors.ElevatorPid.kD, -Constants.AgitatorMotors.ElevatorPid.maxPower, Constants.AgitatorMotors.ElevatorPid.maxPower, false);
    rollerMotor = new SparkMaxMotor(Constants.AgitatorMotors.wheelId);
    rotatorMotor = new SparkMaxMotor(Constants.AgitatorMotors.rotateId, Constants.AgitatorMotors.RotatorPid.kP, Constants.AgitatorMotors.RotatorPid.kI, Constants.AgitatorMotors.RotatorPid.kD, -Constants.AgitatorMotors.ElevatorPid.maxPower, Constants.AgitatorMotors.ElevatorPid.maxPower, false);

  }

  @Override
  public void periodic() {
    Logger.recordOutput("Agitator/Elevator/PositionTarget", targetElevatorPosition);
    Logger.recordOutput("Agitator/Elevator/PositionActual", elevatorMotor.getPosition());
    Logger.recordOutput("Agitator/Elevator/SpeedTarget", elevatorSpeed);
    Logger.recordOutput("Agitator/Elevator/SpeedActual", elevatorMotor.getVelocity());
    Logger.recordOutput("Agitator/Rotator/PositionTarget", targetRotatorPosition);
    Logger.recordOutput("Agitator/Rotator/PositionActual", rotatorMotor.getPosition());
    Logger.recordOutput("Agitator/Roller/SpeedTarget", rollerSpeed);
    Logger.recordOutput("Agitator/Roller/SpeedActual", rollerMotor.getVelocity());
    // This method will be called once per scheduler run
  }

  public boolean elevatorPid(double position){
    targetElevatorPosition = position;
    elevatorMotor.setPosition(targetElevatorPosition);

    return false;
  }
  public boolean elevatorManual(double speed){
    elevatorSpeed = speed;
    elevatorMotor.set(elevatorSpeed);

    return false;
  }
  //pivot of the arm
  public boolean rotatorPid(double position){
    targetRotatorPosition = position;
    elevatorMotor.setPosition(targetRotatorPosition);

    return false;
  }
  public void rollRollerMotor(double speed){
    rollerSpeed = speed;
    rollerMotor.set(rollerSpeed);
  }
}
