package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConsts;
import frc.robot.commands.Climb.RetractClimb;

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
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RetractClimb(this));
}

}
