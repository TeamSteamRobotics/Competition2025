package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.commands.Climb.ClimbIn;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.SparkMaxMotor;

public class ClimbSubsystem extends SubsystemBase {
  private GenericMotor climbMotor;
  public ClimbSubsystem(){
    climbMotor = new SparkMaxMotor(Constants.ClimbMotors.climb);

  }
  public void climbOut() {
    climbMotor.set(Constants.ClimbMotors.climbSpeed);
  }

  public void climbIn() {
    climbMotor.set(-Constants.ClimbMotors.climbSpeed);
  }

  public void stopClimb() {
    climbMotor.set(0);
  }


}
