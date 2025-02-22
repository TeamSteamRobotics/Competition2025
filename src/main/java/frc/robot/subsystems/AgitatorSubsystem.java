// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.KickerConsts;


public class AlgaeKicker extends SubsystemBase {
  TalonFX krakenMotor;
  SparkMax neoTop, neoMid; // TODO: Change motor names

  RelativeEncoder midEncoder; //we might need this for mid motor
  /** Creates a new AlgaeKicker. */

  public AlgaeKicker() {
    krakenMotor = new TalonFX(KickerConsts.KrakenID);
    neoTop = new SparkMax(KickerConsts.TopNeoID, MotorType.kBrushless);
    neoMid = new SparkMax(KickerConsts.MidNeoID, MotorType.kBrushless);
    midEncoder = neoMid.getAlternateEncoder();
  }

  //sets motor speeds
  public void raiseKicker() {
    neoMid.set(KickerConsts.KickerRaiseSpeed);
  }
  public void runKickerRaiseMotor(double speed) {
    neoMid.set(speed);
  }
  public void lowerKicker() {
    neoMid.set(-KickerConsts.KickerRaiseSpeed);
  }

  //gets kicker position
  public double getKickerHeight(){
    return midEncoder.getPosition();
  }
  public double getKickerRotation(){
    return krakenMotor.getPosition().getValueAsDouble();
  }

  public void spinWheels(){
    neoTop.set(KickerConsts.KickerWheelSpinSpeed);
  }

  public void stopWheels(){
    neoTop.set(0);
  }

  public double getWheelSpeed(){
    return neoTop.get();
  }

  public void rotateKicker(double speed){
    krakenMotor.set(speed);
  }
  
  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}
