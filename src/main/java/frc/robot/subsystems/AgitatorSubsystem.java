// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Parameter;
import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Agitator.AgitatorDefault;
import frc.robot.commands.Agitator.AgitatorGetCoral;
import frc.robot.commands.Agitator.AgitatorPutCoral;
import frc.robot.commands.Agitator.ExtendAgitator;
import frc.robot.subsystems.AgitatorSubsystem.AgitatorState.RotatorParam;
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
  public HashMap<String, Supplier<Command>> commandTable; // The things I do to avoid if statements
  public HashMap<String, AgitatorState> StateList;
  public AgitatorState currentState;

  public class AgitatorState {
    public String name;
    public double rollMotorSpeed;
    /** Dictates what the rotate motor parameter is */
    public enum RotatorParam{
      SPEED,
      POSITION
    }
    RotatorParam rotateState;
    public double rotMotorParam;
    /** UNUSED */
    public double elevMotorPos;

    public AgitatorState(String f_name, double rotParam, double rolSpeed, double elePos, RotatorParam parameter){
      name = f_name;
      rotMotorParam = rotParam;
      rollMotorSpeed = rolSpeed;
      elevMotorPos = elePos;
      rotateState = parameter;
    }
  }

  private void initializeStates(){
    StateList = new HashMap<String, AgitatorState>();
    StateList.put("INIT", new AgitatorState("INIT", 0, 0, 0, RotatorParam.POSITION));
    StateList.put("DEFAULT", new AgitatorState("DEFAULT", Constants.AgitatorMotors.rotatorDefaultPosition, 0, 0, RotatorParam.POSITION));
    StateList.put("GET_CORAL", new AgitatorState("GET_CORAL", Constants.AgitatorMotors.rotatorGetCoralPosition, -Constants.AgitatorMotors.rollerSpeed, 0, RotatorParam.POSITION));
    StateList.put("PUT_CORAL", new AgitatorState("PUT_CORAL", Constants.AgitatorMotors.rotatorPutCoralPosition, Constants.AgitatorMotors.rollerSpeed, 0, RotatorParam.POSITION));
    currentState = StateList.get("INIT");
    commandTable = new HashMap<>(); 
    //Transit out of starting state immediately
    commandTable.put("INIT", () -> new AgitatorDefault(this)); //I hope this works first try, because if it doesnt, we'll need to examine my shitty code
    commandTable.put("DEFAULT", () -> new AgitatorDefault(this));
    commandTable.put("GET_CORAL", () -> new AgitatorGetCoral(this));
    commandTable.put("PUT_CORAL", () -> new AgitatorPutCoral(this));
  }
  /** Creates a new AgitatorSubsystem. */
  public AgitatorSubsystem() {

    elevatorMotor = new SparkMaxMotor(Constants.AgitatorMotors.elevatorId, Constants.AgitatorMotors.ElevatorPid.kP, Constants.AgitatorMotors.ElevatorPid.kI, Constants.AgitatorMotors.ElevatorPid.kD, -Constants.AgitatorMotors.ElevatorPid.maxPower, Constants.AgitatorMotors.ElevatorPid.maxPower, false);
    rollerMotor = new SparkMaxMotor(Constants.AgitatorMotors.wheelId);
    rotatorMotor = new SparkMaxMotor(Constants.AgitatorMotors.rotateId, Constants.AgitatorMotors.RotatorPid.kP, Constants.AgitatorMotors.RotatorPid.kI, Constants.AgitatorMotors.RotatorPid.kD, -Constants.AgitatorMotors.ElevatorPid.maxPower, Constants.AgitatorMotors.ElevatorPid.maxPower, false);
    initializeStates();
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
