package frc.robot.subsystems;

import java.util.function.ToIntFunction;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

  public LimelightHelpers.LimelightTarget_Fiducial[] AprilTags;
  
  public VisionSubsystem() {
    AprilTags = new LimelightHelpers.LimelightTarget_Fiducial[22];
    LimelightHelpers.setPipelineIndex("", 0); // 0 is 2D Fiducials, 1 is 3D fiducials
  }


  @Override
  public void periodic() {
    LimelightHelpers.LimelightTarget_Fiducial[] temp = LimelightHelpers.getLatestResults("").targets_Fiducials;
    for(int i = 0; i < 22; i++){
      AprilTags[i] = null; // screw you
    }
    if(!LimelightHelpers.getTV("")){
      return;
    }
    for(int i = 0; i < temp.length; i++)
    {
      AprilTags[(int) temp[i].fiducialID] = temp[i];
    }
    // This method will be called once per scheduler run
  }
  
  public double[] getFiducialDistanceToCamera()
  {

    RawFiducial[] tempRawFiducial = LimelightHelpers.getRawFiducials("");
    RawFiducial[] orderedRawFiducial = new RawFiducial[22];
    for(int i = 0; i < tempRawFiducial.length; i++)
    {
      orderedRawFiducial[tempRawFiducial[i].id] = tempRawFiducial[i];
    }
    double[] orderedDistances = new double[22];
    for(int i = 0; i < 22; i++)
    {
      orderedDistances[i] = orderedRawFiducial[i].distToCamera;
    }
    return orderedDistances;
  }
}
