package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.commands.Climb.RetractClimb;
import frc.robot.subsystems.Motors.GenericMotor;
import frc.robot.subsystems.Motors.SparkFlexMotor;
import frc.robot.subsystems.Motors.SparkMaxMotor;

public class ClimbSubsystem extends SubsystemBase {
  private GenericMotor climbMotor;
  private GenericMotor winchMotor;
  //public boolean atFullSpeed;
  double retractClimbSpeed = Constants.ClimbMotors.climbSpeed;
  double retractWinchSpeed = Constants.ClimbMotors.winchSpeed;
  
  public ClimbSubsystem(){
    climbMotor = new SparkMaxMotor(Constants.ClimbMotors.climb);
    winchMotor = new SparkMaxMotor(Constants.ClimbMotors.winch);
    // atFullSpeed = false;
  }

  public void raiseClimb() {
    climbMotor.set(Constants.ClimbMotors.climbSpeed); //make sure is SLOW, don't bend metal
    winchMotor.set(Constants.ClimbMotors.winchSpeed);
  }

  public void retractClimb() {
    climbMotor.set(-retractClimbSpeed); //Slowly increase speed
    
    if (retractClimbSpeed <= 1) {
    retractClimbSpeed = Math.min(retractClimbSpeed + 0.004, 1); // placeholder
    }
    // if(retractClimbSpeed < 1.01 && retractClimbSpeed > 0.99){
    //   atFullSpeed = true;
    // }
    
    SmartDashboard.putNumber("Climb Speed", retractClimbSpeed);
  }
  public void retractWinch(){
    winchMotor.set(-retractWinchSpeed); //
    // if (retractWinchSpeed <= 0.2) {
    // retractWinchSpeed = Math.min(retractWinchSpeed + 0.004, 0.2); // placeholder
    // }
    
    SmartDashboard.putNumber("Winch Speed", retractWinchSpeed);
  }

  // public void extendWinch(){
  //   winchMotor.set(retractWinchSpeed);
  // }

  public void stopClimb() {
    climbMotor.set(0);
    retractClimbSpeed = Constants.ClimbMotors.climbSpeed;
  }
  public void stopWinch(){
    winchMotor.set(0);
    retractWinchSpeed = Constants.ClimbMotors.winchSpeed;
  }
//   public void initDefaultCommand() {
//     // Set the default command for a subsystem here.
//     setDefaultCommand(new RetractClimb(this));
// }

}
