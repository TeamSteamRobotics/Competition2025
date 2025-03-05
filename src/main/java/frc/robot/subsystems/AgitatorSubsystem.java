// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.AgitatorConsts;


public class AgitatorSubsystem extends SubsystemBase {
  TalonFX krakenMotor;
  SparkMax neoWheel, neoElevator;

  RelativeEncoder midEncoder; //we might need this for mid motor
  /** Creates a new AgitatorSubsystem. */

  public AgitatorSubsystem() {
    krakenMotor = new TalonFX(AgitatorConsts.KrakenID);
    neoWheel = new SparkMax(AgitatorConsts.WheelNeoID, MotorType.kBrushless);
    neoElevator = new SparkMax(AgitatorConsts.ElevatorNeoID, MotorType.kBrushless);
    midEncoder = neoElevator.getAlternateEncoder();
  }

  //sets motor speeds
  public void raiseAgitator() {
    neoElevator.set(AgitatorConsts.AgitatorRaiseSpeed);
  }
  public void runAgitatorRaiseMotor(double speed) {
    neoElevator.set(speed);
  }
  public void lowerAgitator() {
    neoElevator.set(-AgitatorConsts.AgitatorRaiseSpeed);
  }

  //gets agitator position
  public double getAgitatorHeight(){
    return midEncoder.getPosition();
  }
  public double getAgitatorRotation(){
    return krakenMotor.getPosition().getValueAsDouble();
  }

  public void spinWheels(){
    neoWheel.set(AgitatorConsts.AgitatorWheelSpinSpeed);
  }

  public void stopWheels(){
    neoWheel.set(0);
  }

  public double getWheelSpeed(){
    return neoWheel.get();
  }

  public void rotateAgitator(double speed){
    krakenMotor.set(speed);
  }
  
  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}
