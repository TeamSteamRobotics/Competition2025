package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.commands.Climb.RetractClimb;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.SparkFlexMotor;

public class ClimbSubsystem extends SubsystemBase {
  private GenericMotor climbMotor;
  double retractClimbSpeed = Constants.ClimbMotors.climbSpeed;
  public ClimbSubsystem(){
    climbMotor = new SparkFlexMotor(Constants.ClimbMotors.climb);
  }

  public void raiseClimb() {
    climbMotor.set(-Constants.ClimbMotors.climbSpeed); //make sure is SLOW, don't bend metal
  }

  public void retractClimb() {
    climbMotor.set(retractClimbSpeed); //Slowly increase speed
    if (retractClimbSpeed < 0.7) {
    retractClimbSpeed = retractClimbSpeed + 0.0001; // placeholder
    }
    try{
      Thread.sleep(1);
    }
    catch(InterruptedException ex){
      Thread.currentThread().interrupt();
    }
    SmartDashboard.putNumber("Climb Speed", retractClimbSpeed);
  }

  public void stopClimb() {
    climbMotor.set(0);
    retractClimbSpeed = Constants.ClimbMotors.climbSpeed;
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RetractClimb(this));
}

}
