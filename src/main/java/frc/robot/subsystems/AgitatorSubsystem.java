// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.AgitatorConsts;


public class AgitatorSubsystem extends SubsystemBase {
 
  SparkMax agitatorPivot, agitatorWheel, agitatorElevator;

  RelativeEncoder elevatorEncoder; //we might need this for mid motor
  /** Creates a new AgitatorSubsystem. */

  public AgitatorSubsystem() {
    agitatorPivot = new SparkMax(AgitatorConsts.AgitatorPivotID, MotorType.kBrushless);
    agitatorWheel = new SparkMax(AgitatorConsts.AgitatorWheelID, MotorType.kBrushless);
    agitatorElevator = new SparkMax(AgitatorConsts.AgitatorElevatorID, MotorType.kBrushless);
    elevatorEncoder = agitatorElevator.getAlternateEncoder();
  }

  //sets motor speeds
  public void raiseAgitator() {
    agitatorElevator.set(AgitatorConsts.AgitatorRaiseSpeed);
  }
  public void runAgitatorRaiseMotor(double speed) {
    agitatorElevator.set(speed);
  }
  public void lowerAgitator() {
    agitatorElevator.set(-AgitatorConsts.AgitatorRaiseSpeed);
  }

  //gets agitator position
  public double getAgitatorHeight(){
    return elevatorEncoder.getPosition();
  }

  /* 
  public double getAgitatorRotation(){
    return agitatorPivot.;
  }
  */

  public void spinWheels(){
    agitatorWheel.set(AgitatorConsts.AgitatorWheelSpinSpeed);
  }

  public void stopWheels(){
    agitatorWheel.set(0);
  }

  public double getWheelSpeed(){
    return agitatorWheel.get();
  }

  public void rotateAgitator(double speed){
    agitatorPivot.set(speed);
  }
  
  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}
