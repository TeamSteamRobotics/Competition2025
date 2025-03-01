// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  AddressableLEDBuffer m_ledBuffer;
  LEDPattern currentPattern;
  Distance ledSpacing;
  int ledBrightness;

  GenericEntry solidColorPreview;

  ShuffleboardTab tab;
  public LEDSubsystem() {
    m_ledBuffer = new AddressableLEDBuffer(Constants.ledCount);
    ledSpacing = Meters.of(1 / Constants.ledsPerMeter);
    ledBrightness = Constants.ledDefaultBrightness;
    currentPattern = LEDPattern.solid(Color.kBlack);

    tab = Shuffleboard.getTab("LED Controls");
    GenericEntry solidR = tab.add("Solid Color - R", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry solidG = tab.add("Solid Color - G", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry solidB = tab.add("Solid Color - B", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    
    GenericEntry gcR1 = tab.add("Gradient Color 1 - R", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry gcG1 = tab.add("Gradient Color 1 - G", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry gcB1 = tab.add("Gradient Color 1 - B", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry gcR2 = tab.add("Gradient Color 2 - R", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry gcG2 = tab.add("Gradient Color 2 - G", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    GenericEntry gcB2 = tab.add("Gradient Color 2 - B", 255).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    
    GenericEntry gradientSpeed = tab.add("Gradient Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.1, "max", 5.0)).getEntry();
    GenericEntry rainbowSpeed = tab.add("Rainbow Speed", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.1, "max", 5.0)).getEntry();
    GenericEntry brightnessLevel = tab.add("Brightness Level", 128).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
    
    tab.add("Apply Solid Color", SetSolidColor(new Color(solidR.getDouble(255), solidG.getDouble(255), solidB.getDouble(255))).withName("Apply")).withWidget(BuiltInWidgets.kCommand);
    tab.add("Apply Rainbow", SetRainbow(255, rainbowSpeed.getDouble(1)).withName("Queerify(TM)")).withWidget(BuiltInWidgets.kCommand);
    tab.add("Apply Continuous Gradient", SetGradient(true, new Color(gcR1.getDouble(255), gcG1.getDouble(255), gcB1.getDouble(255)), new Color(gcR2.getDouble(255), gcG2.getDouble(255), gcB2.getDouble(255))).withName("Apply")).withWidget(BuiltInWidgets.kCommand);
    tab.add("Apply Discontinuous Gradient", SetGradient(false, new Color(gcR1.getDouble(255), gcG1.getDouble(255), gcB1.getDouble(255)), new Color(gcR2.getDouble(255), gcG2.getDouble(255), gcB2.getDouble(255))).withName("Apply")).withWidget(BuiltInWidgets.kCommand);
    tab.add("Apply Gradient With Speed", SetGradient(new Color(gcR1.getDouble(255), gcG1.getDouble(255), gcB1.getDouble(255)), new Color(gcR2.getDouble(255), gcG2.getDouble(255), gcB2.getDouble(255)), gradientSpeed.getDouble(1)).withName("Apply")).withWidget(BuiltInWidgets.kCommand);
    tab.add("Turn Off LED", TurnOffLed().withName("Turn Off")).withWidget(BuiltInWidgets.kCommand);
    tab.add("Apply Brightness", SetBrightness((int) brightnessLevel.getDouble(128)).withName("Apply")).withWidget(BuiltInWidgets.kCommand);
  }

  public void setSolidColor(Color color) {
    currentPattern = LEDPattern.solid(color);
  }

  public void setRainbow(int saturation, double speed) {
    LEDPattern rainbowPattern = LEDPattern.rainbow(saturation, ledBrightness);
    currentPattern = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(speed), ledSpacing);
  }

  public void setContinuousGradient(Color firstColor, Color secondColor, double speed) {
    currentPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, firstColor, secondColor)
        .scrollAtAbsoluteSpeed(MetersPerSecond.of(speed), ledSpacing);
  }

  public void setContinuousGradient(Color firstColor, Color secondColor) {
    currentPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, firstColor, secondColor);
  }

  public void setDiscontinuousGradient(Color firstColor, Color secondColor) {
    currentPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, firstColor, secondColor);
  }

  public void setTwoColors(Color firstColor, Color secondColor, double secondColorStartPosition) {
    currentPattern = LEDPattern.steps(Map.of(0, firstColor, secondColorStartPosition, secondColor));
  }




  public Command SetSolidColor(Color color) {
    return runOnce(() -> setSolidColor(color));
  }

  public Command SetRainbow(int saturation, double speed) {
    return runOnce(() -> setRainbow(saturation, speed));
  }

  public Command SetGradient(boolean isContinuous, Color firstColor, Color secondColor) {
    return runOnce(() -> {
      if (isContinuous) {
        setContinuousGradient(firstColor, secondColor);
      } else {
        setDiscontinuousGradient(firstColor, secondColor);
      }
    });
  }

  public Command SetGradient(Color firstColor, Color secondColor, double speed) {
    return runOnce(() -> setContinuousGradient(firstColor, secondColor, speed));
  }

  public Command TurnOffLed() {
    return runOnce(() -> setSolidColor(Color.kBlack));
  }

  public Command SetTwoColor(Color firstColor, Color secondColor, double secondColorStartPosition) {
    return runOnce(() -> setTwoColors(firstColor, secondColor, secondColorStartPosition));
  }

  public Command SetBrightness(int brightness) {
    return runOnce(() -> {
      ledBrightness = brightness;
      currentPattern.atBrightness(Percent.of(ledBrightness/255));
  });
  }

  @Override
  public void periodic() {
    //currentPattern = currentPattern.atBrightness(Percent.of((double) ledBrightness / 255));
    currentPattern.applyTo(m_ledBuffer);
  }

  @Override
  public void simulationPeriodic() {
    //System.out.println("[SIMULATION] LED Pattern Applied: " + currentPattern.toString());
  }
}