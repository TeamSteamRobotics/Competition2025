package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConsts;;

public class ClimbSubsystem extends SubsystemBase {
  private SparkMax climbMotor;
  public ClimbSubsystem(){
    climbMotor = new SparkMax(ClimbConsts.climbMotorID, MotorType.kBrushless);

  }
  public void raiseClimb() {
    climbMotor.set(-ClimbConsts.climbSpeed);
  }

  public void retractClimb() {
    climbMotor.set(ClimbConsts.climbSpeed);
  }

  public void stopClimb() {
    climbMotor.set(0);
  }
}
