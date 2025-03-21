package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

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
    winchMotor = new SparkFlexMotor(Constants.ClimbMotors.winch);
    // atFullSpeed = false;
  }

  public void raiseClimb() {
    climbMotor.set(Constants.ClimbMotors.climbSpeed); //make sure is SLOW, don't bend metal
    
  }

  public void raiseWinch(){
    winchMotor.set(Constants.ClimbMotors.winchOutSpeed);
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


@Override
public void periodic(){
    Logger.recordOutput("Climber/ComponentPose", new Pose3d[]{
    new Pose3d(-0.32, 0.03, 0.1, new Rotation3d(0.0, Math.toRadians(climbMotor.getPosition() / Constants.ClimbMotors.climbGearRatio), 0.0))
    });
    Logger.recordOutput("Climber/ClimbEncoderPosition", climbMotor.getPosition());
    Logger.recordOutput("Climber/WinchEncoderPosition", winchMotor.getPosition());
    Logger.recordOutput("Climber/ClimbSpeed", climbMotor.getVelocity());
    Logger.recordOutput("Climber/WinchSpeed", winchMotor.getVelocity());
}
}
