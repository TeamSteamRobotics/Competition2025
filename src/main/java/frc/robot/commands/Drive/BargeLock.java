package frc.robot.commands.Drive;

import java.io.Console;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.Coordinate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class BargeLock extends Command {
    double ShootDist = Constants.Shooter.shootingDistance;
    double ShootOff = Constants.Shooter.shootingDistanceOffset;
  /** Creates a new RetractClimb. */
    AprilVisionSubsystem vision;
  public BargeLock(AprilVisionSubsystem vis) {
    vision = vis;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   Coordinate position = vision.getCoordinates(4,AprilVisionSubsystem.ReturnTarget.TARGET);
   System.out.println(position);
   double distance = position.z;
    if ((ShootDist - ShootOff <= distance) && (distance <= ShootDist + ShootOff)) { //green
        SmartDashboard.putBoolean("In Shooting Distance", true);
    }else{//red
        SmartDashboard.putBoolean("In Shooting Distance", false);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
