// PrimeShooter.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * PrimeShooter is a command that prepares the shooter subsystem to fire.
 * It calculates and sets the appropriate motor speeds based on input.
 */
public class PrimeShooter extends Command {
  private ShooterSubsystem m_Shooter; // The subsystem this command controls.
  private double speed; // The calculated speed for the shooter.
  private double inputSpeed; // A fixed speed provided directly.
  private Supplier<Double> distanceSupplier = () -> null; // A function that gives the distance to the target.
  private boolean hasDistanceSupplier; // Indicates whether distance-based speed calculation is used.

  /**
   * Constructor: Sets up the command to calculate speed dynamically based on distance.
   * @param shooter The shooter subsystem to control.
   * @param distanceSupplier A function that provides the distance to the target.
   */
  public PrimeShooter(ShooterSubsystem shooter, Supplier<Double> distanceSupplier) {
    m_Shooter = shooter;
    this.distanceSupplier = distanceSupplier;
    hasDistanceSupplier = true; // Enable distance-based speed calculation.
  }

  /**
   * Constructor: Sets up the command with a fixed speed for the shooter.
   * @param shooter The shooter subsystem to control.
   * @param speed The fixed speed to use.
   */
  public PrimeShooter(ShooterSubsystem shooter, double speed) {
    m_Shooter = shooter;
    //sets speed based on smartdashboard value
    inputSpeed = SmartDashboard.getNumber("Shooter Speed", 0.45);
  //  inputSpeed = m_Shooter.overrideDefault ? m_Shooter.getTargetSpeed() : speed;
    
    hasDistanceSupplier = false; // Use the fixed speed instead of calculating from distance.
  }
// change shooter speed in smart dashboard

  /**
   * This method runs once when the command is first scheduled.
   * Use it for any setup needed before the command starts.
   */
  @Override
  public void initialize() {}

  /**
   * This method runs repeatedly while the command is scheduled.
   * It calculates the appropriate speed and applies it to the shooter subsystem.
   */
  @Override
  public void execute() {
    // Determine the speed: calculate it from distance or use the fixed input speed.
    speed = (hasDistanceSupplier ? speedFromDistance(distanceSupplier.get()) : inputSpeed);

    // Command the shooter subsystem to run at the calculated speed.
    m_Shooter.Shoot(speed);
    //double[] velArray = m_Shooter.getSpeeds();

  }

  /**
   * This method runs once when the command ends or is interrupted.
   * It ensures the shooter motors stop running.
   */
  @Override
  public void end(boolean interrupted) {
    //TODO: CHANGE THIS?
    
    if(interrupted){

    m_Shooter.StopMotor();
      if(!hasDistanceSupplier){
        m_Shooter.overrideDefault = false;
      }
    }
    
  }

  

  /**
   * Determines when the command should stop running.
   * @return True if the shooter motors have reached their target speed, false otherwise.
   */
  @Override
  public boolean isFinished() {

    //     If a distance supplier is being used (indicated by hasDistanceSupplier being true),
    //      the command also ends if the distance supplier provides a null value.
    //    - This handles cases where the distance sensor might fail or is unavailable,
    //      ensuring the command terminates safely.
    return (hasDistanceSupplier && (distanceSupplier.get() == null || distanceSupplier == null));
  }

  /**
   * Calculates the motor speed required based on the distance to the target.
   * For example, further distances might require higher speeds.
   * @param distance The distance to the target (e.g., in meters).
   * @return The calculated speed.
   */
  private double speedFromDistance(double distance) {
    // TODO: Implement this function to calculate the speed based on distance.
    return 0; // Placeholder value; update with real logic.
  }
}
