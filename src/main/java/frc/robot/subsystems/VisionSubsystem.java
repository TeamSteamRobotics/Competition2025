package frc.robot.subsystems;

import java.util.function.ToIntFunction;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  public class FusedFiducialType {
    public LimelightHelpers.LimelightTarget_Fiducial processedAprilTag;
    public RawFiducial rawAprilTag;
    public FusedFiducialType(){
      processedAprilTag = null;
      rawAprilTag = null;
    }
  }
  public FusedFiducialType[] AprilTags; // List of all apriltags on the field
  // Seen ones are non-null, indexed by Apriltag ID - 1
  
  public VisionSubsystem() {
    AprilTags = new FusedFiducialType[22];
    LimelightHelpers.setPipelineIndex("", 0); // 0 is 2D Fiducials, 1 is 3D fiducials
  }


  @Override
  public void periodic() {
    LimelightHelpers.LimelightTarget_Fiducial[] temp = LimelightHelpers.getLatestResults("").targets_Fiducials;
    for(int i = 0; i < 22; i++){
      AprilTags[i] = null; // You make me very unhappy. Please stop it. What could go wrong?
    }
    if(!LimelightHelpers.getTV("")){
      return;
    }
    for(int i = 0; i < temp.length; i++)
    {
      
      AprilTags[(int) temp[i].fiducialID - 1].processedAprilTag = temp[i]; // Reorders the temp array such that all Apriltags are in the array index correspondingly to their ID
      // If Apriltag data is ending up in the wrong place, use Math.round before typecasting
      // Wrong place specifically being 1 slot before it should be
      // If Java C-style typecasting works like C typecasting, it's weird.
      AprilTags[(int) temp[i].fiducialID - 1].rawAprilTag = LimelightHelpers.getRawFiducials("")[i];
    }
    // This method will be called once per scheduler run
  }
  
  public double[] getFiducialDistanceToCamera()
  {
    double[] orderedDistances = new double[22];
    for(int i = 0; i < 22; i++)
    {
      if(AprilTags[i].rawAprilTag == null){
        continue;
      }
      orderedDistances[i] = AprilTags[i].rawAprilTag.distToCamera;
    }
    
    return orderedDistances;
  }
  //SmartDashboard.putNumber("Limelight Distance", orderedDistances);
}
