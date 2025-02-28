// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonTest extends SubsystemBase {
  
  private double test;

  /** Creates a new ButtonTest. */
  public ButtonTest() {
    test = SmartDashboard.getNumber("test", -1);
  }

  public void rollerSpeed() {
    test = SmartDashboard.getNumber("Roller", 1);
  }



  public void add() {
    SmartDashboard.putNumber("test_add", test + 2);
  }

  public void sub() {
    SmartDashboard.putNumber("test_sub", test - 2);
  }

  public void mult() {
    SmartDashboard.putNumber("test_mult", test * 2);
  }

  public void div() {
    SmartDashboard.putNumber("test_div", test / 2);
  }

  public void stopButtonTest() {
    SmartDashboard.putBoolean("Running", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
